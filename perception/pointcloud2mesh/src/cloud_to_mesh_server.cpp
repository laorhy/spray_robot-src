#include <chrono>
#include <condition_variable>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>

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

class CloudToMeshServer : public rclcpp::Node
{
public:
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

    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/lidar/fused/roi", rclcpp::SensorDataQoS(),
      std::bind(&CloudToMeshServer::onCloud, this, std::placeholders::_1));

    save_mesh_srv_ = this->create_service<pointcloud2mesh::srv::SaveMesh>(
      "/save_mesh",
      std::bind(
        &CloudToMeshServer::onSaveMesh, this,
        std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "cloud_to_mesh_server started.");
  }

private:
  struct SaveOptions
  {
    std::string output_dir;
    std::string filename_prefix;
    bool save_ply;
    bool save_stl;
    int poisson_depth;
    float scale;
    float samples_per_node;
    float point_weight;
    int normal_k;
    float wait_timeout_sec;
  };

  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_cloud_ = msg;
    latest_receive_time_ = this->now();
    cv_.notify_all();
  }

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
    opts.poisson_depth = 10;
    opts.scale = 1.1f;
    opts.samples_per_node = 1.5f;
    opts.point_weight = 4.0f;
    opts.normal_k = 30;
    opts.wait_timeout_sec = 1.0f;
    return opts;
  }

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
    }

    opts.poisson_depth = (request->poisson_depth > 0) ? request->poisson_depth : opts.poisson_depth;
    opts.scale = (request->scale > 0.0f) ? request->scale : opts.scale;
    opts.samples_per_node =
      (request->samples_per_node > 0.0f) ? request->samples_per_node : opts.samples_per_node;
    opts.point_weight = (request->point_weight > 0.0f) ? request->point_weight : opts.point_weight;
    opts.normal_k = (request->normal_k > 0) ? request->normal_k : opts.normal_k;
    opts.wait_timeout_sec =
      (request->wait_timeout_sec > 0.0f) ? request->wait_timeout_sec : opts.wait_timeout_sec;

    return opts;
  }

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
      return nullptr;
    }
    return latest_cloud_;
  }

  void onSaveMesh(
    const pointcloud2mesh::srv::SaveMesh::Request::SharedPtr request,
    pointcloud2mesh::srv::SaveMesh::Response::SharedPtr response)
  {
    const SaveOptions opts = mergeRequestWithDefaults(request);

    const auto cloud_msg = waitForFreshCloud(opts.wait_timeout_sec);
    if (!cloud_msg) {
      response->success = false;
      response->mesh_path = "";
      response->num_points = 0;
      response->num_triangles = 0;
      response->message = "No fresh cached point cloud available before timeout.";
      return;
    }

    auto cloud_xyz = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(*cloud_msg, *cloud_xyz);

    if (cloud_xyz->empty()) {
      response->success = false;
      response->mesh_path = "";
      response->num_points = 0;
      response->num_triangles = 0;
      response->message = "Cached point cloud is empty.";
      return;
    }

    response->num_points = static_cast<int32_t>(cloud_xyz->size());

    auto normals = pcl::make_shared<pcl::PointCloud<pcl::Normal>>();
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setInputCloud(cloud_xyz);
    normal_estimator.setKSearch(opts.normal_k);
    auto search_tree = pcl::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
    normal_estimator.setSearchMethod(search_tree);
    normal_estimator.compute(*normals);

    auto cloud_with_normals = pcl::make_shared<pcl::PointCloud<pcl::PointNormal>>();
    pcl::concatenateFields(*cloud_xyz, *normals, *cloud_with_normals);

    pcl::Poisson<pcl::PointNormal> poisson;
    poisson.setDepth(opts.poisson_depth);
    poisson.setScale(opts.scale);
    poisson.setSamplesPerNode(opts.samples_per_node);
    poisson.setPointWeight(opts.point_weight);
    poisson.setInputCloud(cloud_with_normals);

    pcl::PolygonMesh mesh;
    poisson.reconstruct(mesh);

    response->num_triangles = static_cast<int32_t>(mesh.polygons.size());
    if (response->num_triangles <= 0) {
      response->success = false;
      response->mesh_path = "";
      response->message = "Poisson reconstruction produced an empty mesh.";
      return;
    }

    std::filesystem::create_directories(opts.output_dir);
    const std::string timestamp = makeTimestamp();
    const std::string base_path = opts.output_dir + "/" + opts.filename_prefix + "_" + timestamp;

    std::vector<std::string> saved_paths;
    if (opts.save_ply) {
      const std::string ply_path = base_path + ".ply";
      if (pcl::io::savePLYFileBinary(ply_path, mesh) == 0) {
        saved_paths.push_back(ply_path);
      } else {
        RCLCPP_WARN(this->get_logger(), "Failed to save PLY: %s", ply_path.c_str());
      }
    }

    if (opts.save_stl) {
      const std::string stl_path = base_path + ".stl";
      if (pcl::io::savePolygonFileSTL(stl_path, mesh) > 0) {
        saved_paths.push_back(stl_path);
      } else {
        RCLCPP_WARN(this->get_logger(), "Failed to save STL: %s", stl_path.c_str());
      }
    }

    if (saved_paths.empty()) {
      response->success = false;
      response->mesh_path = "";
      response->message = "Mesh generated but failed to save requested file formats.";
      return;
    }

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
  }

  std::string default_output_dir_;
  std::string default_filename_prefix_;
  bool default_save_ply_;
  bool default_save_stl_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Service<pointcloud2mesh::srv::SaveMesh>::SharedPtr save_mesh_srv_;

  std::mutex mutex_;
  std::condition_variable cv_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;
  rclcpp::Time latest_receive_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CloudToMeshServer>());
  rclcpp::shutdown();
  return 0;
}
