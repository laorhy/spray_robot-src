#include <chrono>
#include <condition_variable>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <stdexcept>

#include <pcl/common/io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/point_types.h>
#include <pcl/surface/poisson.h>

#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "pointcloud2mesh/srv/save_mesh.hpp"

/**
 * @class CloudToMeshServer
 * @brief ROS 2 服务节点，用于将点云数据转换为网格模型
 * 
 * 该节点订阅 /lidar/fused/roi 话题获取点云数据，
 * 并通过 /save_mesh 服务提供点云到网格的转换功能。
 * 支持 PLY 和 STL 格式的网格输出，使用 Poisson 重建算法。
 */
class CloudToMeshServer : public rclcpp::Node
{
public:
  /**
   * @brief 构造函数
   * 
   * 初始化节点，声明参数，创建订阅和服务。
   */
  CloudToMeshServer()
  : Node("cloud_to_mesh_server")
  {
    // 声明节点参数：作为 service 请求为空时的默认值来源。
    default_output_dir_ = this->declare_parameter<std::string>(
      "default_output_dir", "/tmp/pointcloud_mesh");
    default_filename_prefix_ = this->declare_parameter<std::string>(
      "default_filename_prefix", "roi_mesh");
    default_save_ply_ = this->declare_parameter<bool>("default_save_ply", true);
    default_save_stl_ = this->declare_parameter<bool>("default_save_stl", false);
    (void)this->declare_parameter<bool>("use_sim_time", false);

    // 创建点云订阅
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/lidar/fused/roi", rclcpp::SensorDataQoS(),
      std::bind(&CloudToMeshServer::onCloud, this, std::placeholders::_1));

    // 创建保存网格服务
    save_mesh_srv_ = this->create_service<pointcloud2mesh::srv::SaveMesh>(
      "/save_mesh",
      std::bind(
        &CloudToMeshServer::onSaveMesh, this,
        std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "cloud_to_mesh_server started.");
    RCLCPP_INFO(this->get_logger(), "Subscribed to: /lidar/fused/roi");
    RCLCPP_INFO(this->get_logger(), "Service available: /save_mesh");
  }

private:
  /**
   * @struct SaveOptions
   * @brief 保存网格的选项参数
   */
  struct SaveOptions
  {
    std::string output_dir;       ///< 输出目录
    std::string filename_prefix;  ///< 文件名前缀
    bool save_ply;                ///< 是否保存为 PLY 格式
    bool save_stl;                ///< 是否保存为 STL 格式
    int poisson_depth;            ///< Poisson 重建深度 (3-12，值越大细节越多但速度越慢)
    float scale;                  ///< 缩放因子 (通常为 1.1)
    float samples_per_node;       ///< 每个节点的样本数 (通常为 1.5)
    float point_weight;           ///< 点权重 (通常为 4.0)
    int normal_k;                 ///< 法线估计的最近邻数
    float wait_timeout_sec;       ///< 等待点云数据的超时时间
  };

  /**
   * @brief 处理接收到的点云数据
   * 
   * @param msg 点云消息
   */
  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    try {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_cloud_ = msg;
      latest_receive_time_ = this->now();
      RCLCPP_DEBUG(this->get_logger(), "Received point cloud with %u points", msg->width * msg->height);
      cv_.notify_all();
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error processing point cloud: %s", e.what());
    }
  }

  /**
   * @brief 生成时间戳字符串
   * 
   * @return 格式化的时间戳字符串
   */
  static std::string makeTimestamp()
  {
    const std::time_t now = std::time(nullptr);
    std::tm tm_now;
#ifdef _WIN32
    localtime_s(&tm_now, &now);
#else
    localtime_r(&now, &tm_now);
#endif
    std::ostringstream oss;
    oss << std::put_time(&tm_now, "%Y%m%d_%H%M%S");
    return oss.str();
  }

  /**
   * @brief 创建默认的保存选项
   * 
   * @param default_output_dir 默认输出目录
   * @param default_filename_prefix 默认文件名前缀
   * @param default_save_ply 默认是否保存为 PLY 格式
   * @param default_save_stl 默认是否保存为 STL 格式
   * @return 默认的保存选项
   */
  static SaveOptions makeDefaultOptions(
    const std::string & default_output_dir,
    const std::string & default_filename_prefix,
    bool default_save_ply,
    bool default_save_stl)
  {
    SaveOptions opts;
    opts.output_dir = default_output_dir;
    opts.filename_prefix = default_filename_prefix;
    opts.save_ply = default_save_ply;
    opts.save_stl = default_save_stl;
    opts.poisson_depth = 10;          // 默认 Poisson 深度
    opts.scale = 1.1f;                // 默认缩放因子
    opts.samples_per_node = 1.5f;     // 默认每个节点样本数
    opts.point_weight = 4.0f;         // 默认点权重
    opts.normal_k = 30;               // 默认法线估计最近邻数
    opts.wait_timeout_sec = 1.0f;     // 默认超时时间
    return opts;
  }

  /**
   * @brief 合并服务请求和默认选项
   * 
   * @param request 服务请求
   * @return 合并后的保存选项
   */
  SaveOptions mergeRequestWithDefaults(const pointcloud2mesh::srv::SaveMesh::Request::SharedPtr request) const
  {
    // 参数优先级：请求字段 > 节点参数默认值 > 代码默认常量。
    SaveOptions opts = makeDefaultOptions(
      default_output_dir_, default_filename_prefix_, default_save_ply_, default_save_stl_);

    if (!request->output_dir.empty()) {
      opts.output_dir = request->output_dir;
    }
    if (!request->filename_prefix.empty()) {
      opts.filename_prefix = request->filename_prefix;
    }

    // save_* 始终按请求覆盖默认。若两个都 false，则按约定强制 save_ply=true。
    opts.save_ply = request->save_ply;
    opts.save_stl = request->save_stl;
    if (!opts.save_ply && !opts.save_stl) {
      opts.save_ply = true;
      RCLCPP_INFO(this->get_logger(), "Both save_ply and save_stl are false, defaulting to save_ply=true");
    }

    opts.poisson_depth = (request->poisson_depth > 0) ? request->poisson_depth : opts.poisson_depth;
    opts.scale = (request->scale > 0.0f) ? request->scale : opts.scale;
    opts.samples_per_node =
      (request->samples_per_node > 0.0f) ? request->samples_per_node : opts.samples_per_node;
    opts.point_weight = (request->point_weight > 0.0f) ? request->point_weight : opts.point_weight;
    opts.normal_k = (request->normal_k > 0) ? request->normal_k : opts.normal_k;
    opts.wait_timeout_sec =
      (request->wait_timeout_sec > 0.0f) ? request->wait_timeout_sec : opts.wait_timeout_sec;

    // 日志记录最终选项
    RCLCPP_DEBUG(this->get_logger(), "Save options: output_dir=%s, filename_prefix=%s, save_ply=%d, save_stl=%d",
      opts.output_dir.c_str(), opts.filename_prefix.c_str(), opts.save_ply, opts.save_stl);
    RCLCPP_DEBUG(this->get_logger(), "Poisson options: depth=%d, scale=%f, samples_per_node=%f, point_weight=%f, normal_k=%d",
      opts.poisson_depth, opts.scale, opts.samples_per_node, opts.point_weight, opts.normal_k);

    return opts;
  }

  /**
   * @brief 等待获取新鲜的点云数据
   * 
   * @param timeout_sec 超时时间（秒）
   * @return 点云消息，如果超时返回 nullptr
   */
  sensor_msgs::msg::PointCloud2::SharedPtr waitForFreshCloud(double timeout_sec)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    const auto timeout = std::chrono::duration<double>(timeout_sec);

    const bool available = cv_.wait_for(lock, timeout, [this, timeout_sec]() {
      if (!latest_cloud_) {
        return false;
      }
      const auto age = (this->now() - latest_receive_time_).seconds();
      return age <= timeout_sec;
    });

    if (!available) {
      RCLCPP_WARN(this->get_logger(), "No fresh point cloud available within %.2f seconds", timeout_sec);
      return nullptr;
    }
    RCLCPP_DEBUG(this->get_logger(), "Found fresh point cloud, age: %.2f seconds", 
      (this->now() - latest_receive_time_).seconds());
    return latest_cloud_;
  }

  /**
   * @brief 处理保存网格的服务请求
   * 
   * @param request 服务请求
   * @param response 服务响应
   */
  void onSaveMesh(
    const pointcloud2mesh::srv::SaveMesh::Request::SharedPtr request,
    pointcloud2mesh::srv::SaveMesh::Response::SharedPtr response)
  {
    try {
      RCLCPP_INFO(this->get_logger(), "Received save mesh request");
      
      const SaveOptions opts = mergeRequestWithDefaults(request);

      // 等待点云数据
      const auto cloud_msg = waitForFreshCloud(opts.wait_timeout_sec);
      if (!cloud_msg) {
        response->success = false;
        response->mesh_path = "";
        response->num_points = 0;
        response->num_triangles = 0;
        response->message = "No fresh cached point cloud available before timeout. Please ensure the point cloud publisher is running.";
        RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
        return;
      }

      // 转换点云格式
      auto cloud_xyz = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
      try {
        pcl::fromROSMsg(*cloud_msg, *cloud_xyz);
      } catch (const std::exception& e) {
        response->success = false;
        response->mesh_path = "";
        response->num_points = 0;
        response->num_triangles = 0;
        response->message = "Failed to convert point cloud: " + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return;
      }

      if (cloud_xyz->empty()) {
        response->success = false;
        response->mesh_path = "";
        response->num_points = 0;
        response->num_triangles = 0;
        response->message = "Cached point cloud is empty. Please check the point cloud source.";
        RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
        return;
      }

      response->num_points = static_cast<int32_t>(cloud_xyz->size());
      RCLCPP_INFO(this->get_logger(), "Processing point cloud with %d points", response->num_points);

      // 估计法线
      auto normals = pcl::make_shared<pcl::PointCloud<pcl::Normal>>();
      try {
        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_estimator;
        normal_estimator.setInputCloud(cloud_xyz);
        normal_estimator.setKSearch(opts.normal_k);
        auto search_tree = pcl::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
        normal_estimator.setSearchMethod(search_tree);
        normal_estimator.compute(*normals);
        RCLCPP_DEBUG(this->get_logger(), "Computed normals for %zu points", normals->size());
      } catch (const std::exception& e) {
        response->success = false;
        response->mesh_path = "";
        response->message = "Failed to compute normals: " + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return;
      }

      // 合并点云和法线
      auto cloud_with_normals = pcl::make_shared<pcl::PointCloud<pcl::PointNormal>>();
      try {
        pcl::concatenateFields(*cloud_xyz, *normals, *cloud_with_normals);
        RCLCPP_DEBUG(this->get_logger(), "Concatenated fields, resulting in %zu points with normals", cloud_with_normals->size());
      } catch (const std::exception& e) {
        response->success = false;
        response->mesh_path = "";
        response->message = "Failed to concatenate fields: " + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return;
      }

      // Poisson 重建
      pcl::Poisson<pcl::PointNormal> poisson;
      poisson.setDepth(opts.poisson_depth);
      poisson.setScale(opts.scale);
      poisson.setSamplesPerNode(opts.samples_per_node);
      poisson.setPointWeight(opts.point_weight);
      poisson.setInputCloud(cloud_with_normals);

      pcl::PolygonMesh mesh;
      try {
        RCLCPP_INFO(this->get_logger(), "Starting Poisson reconstruction with depth=%d", opts.poisson_depth);
        poisson.reconstruct(mesh);
        RCLCPP_INFO(this->get_logger(), "Poisson reconstruction completed");
      } catch (const std::exception& e) {
        response->success = false;
        response->mesh_path = "";
        response->message = "Failed to perform Poisson reconstruction: " + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return;
      }

      response->num_triangles = static_cast<int32_t>(mesh.polygons.size());
      if (response->num_triangles <= 0) {
        response->success = false;
        response->mesh_path = "";
        response->message = "Poisson reconstruction produced an empty mesh. Try adjusting Poisson parameters.";
        RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Generated mesh with %d triangles", response->num_triangles);

      // 创建输出目录
      try {
        std::filesystem::create_directories(opts.output_dir);
        RCLCPP_DEBUG(this->get_logger(), "Created output directory: %s", opts.output_dir.c_str());
      } catch (const std::exception& e) {
        response->success = false;
        response->mesh_path = "";
        response->message = "Failed to create output directory: " + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return;
      }

      // 生成文件名
      const std::string timestamp = makeTimestamp();
      const std::string base_path = opts.output_dir + "/" + opts.filename_prefix + "_" + timestamp;
      RCLCPP_DEBUG(this->get_logger(), "Base path for output files: %s", base_path.c_str());

      // 保存网格文件
      std::vector<std::string> saved_paths;
      
      if (opts.save_ply) {
        const std::string ply_path = base_path + ".ply";
        try {
          if (pcl::io::savePLYFileBinary(ply_path, mesh) == 0) {
            saved_paths.push_back(ply_path);
            RCLCPP_INFO(this->get_logger(), "Saved PLY file: %s", ply_path.c_str());
          } else {
            RCLCPP_WARN(this->get_logger(), "Failed to save PLY file: %s", ply_path.c_str());
          }
        } catch (const std::exception& e) {
          RCLCPP_WARN(this->get_logger(), "Exception saving PLY file: %s", e.what());
        }
      }

      if (opts.save_stl) {
        const std::string stl_path = base_path + ".stl";
        try {
          if (pcl::io::saveSTLFile(stl_path, mesh) > 0) {
            saved_paths.push_back(stl_path);
            RCLCPP_INFO(this->get_logger(), "Saved STL file: %s", stl_path.c_str());
          } else {
            RCLCPP_WARN(this->get_logger(), "Failed to save STL file: %s", stl_path.c_str());
          }
        } catch (const std::exception& e) {
          RCLCPP_WARN(this->get_logger(), "Exception saving STL file: %s", e.what());
        }
      }

      if (saved_paths.empty()) {
        response->success = false;
        response->mesh_path = "";
        response->message = "Mesh generated but failed to save requested file formats. Check file permissions and disk space.";
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return;
      }

      // 成功响应
      response->success = true;
      response->mesh_path = saved_paths.front();

      std::ostringstream msg;
      msg << "Mesh saved successfully (" << response->num_points << " points, "
          << response->num_triangles << " triangles).";
      if (saved_paths.size() > 1) {
        msg << " Multiple outputs: ";
        for (size_t i = 0; i < saved_paths.size(); ++i) {
          if (i > 0) {
            msg << ", ";
          }
          msg << saved_paths[i];
        }
      }
      response->message = msg.str();
      RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
      
    } catch (const std::exception& e) {
      // 捕获所有未处理的异常
      response->success = false;
      response->mesh_path = "";
      response->num_points = 0;
      response->num_triangles = 0;
      response->message = "Unexpected error: " + std::string(e.what());
      RCLCPP_ERROR(this->get_logger(), "Unexpected error in save mesh service: %s", e.what());
    }
  }

  // 节点参数
  std::string default_output_dir_;       ///< 默认输出目录
  std::string default_filename_prefix_;  ///< 默认文件名前缀
  bool default_save_ply_;                ///< 默认是否保存为 PLY 格式
  bool default_save_stl_;                ///< 默认是否保存为 STL 格式

  // ROS 2 组件
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;  ///< 点云订阅
  rclcpp::Service<pointcloud2mesh::srv::SaveMesh>::SharedPtr save_mesh_srv_;  ///< 保存网格服务

  // 点云缓存
  std::mutex mutex_;                                  ///< 互斥锁
  std::condition_variable cv_;                        ///< 条件变量
  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;  ///< 最新点云
  rclcpp::Time latest_receive_time_;                  ///< 最新点云接收时间
};

/**
 * @brief 主函数
 * 
 * @param argc 命令行参数数量
 * @param argv 命令行参数
 * @return 退出码
 */
int main(int argc, char ** argv)
{
  try {
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("main"), "Initializing cloud_to_mesh_server");
    rclcpp::spin(std::make_shared<CloudToMeshServer>());
    rclcpp::shutdown();
    RCLCPP_INFO(rclcpp::get_logger("main"), "cloud_to_mesh_server shut down");
    return 0;
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("main"), "Fatal error: %s", e.what());
    return 1;
  }
}