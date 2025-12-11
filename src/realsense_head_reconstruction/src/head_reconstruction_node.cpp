#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/concatenate.h>
#include <pcl/registration/icp.h>
#include <pcl_msgs/msg/polygon_mesh.hpp>

namespace realsense_head_reconstruction
{

class HeadReconstructionNode : public rclcpp::Node
{
public:
  HeadReconstructionNode()
  : Node("head_reconstruction_node"),
    accumulated_cloud_(new pcl::PointCloud<pcl::PointXYZRGB>()),
    accumulated_frames_(0)
  {
    pointcloud_topic_ = this->declare_parameter<std::string>("pointcloud_topic", "/camera/depth/color/points");
    frames_to_accumulate_ = this->declare_parameter<int>("frames_to_accumulate", 30);
    min_point_threshold_ = this->declare_parameter<int>("min_point_threshold", 12000);
    voxel_leaf_size_ = this->declare_parameter<double>("voxel_leaf_size", 0.005);
    auto z_limits = this->declare_parameter<std::vector<double>>("z_limits", {0.2, 1.2});
    auto y_limits = this->declare_parameter<std::vector<double>>("y_limits", {-0.2, 0.4});
    reconstruction_frame_id_ = this->declare_parameter<std::string>("reconstruction_frame_id", "camera_link");
    publish_mesh_ = this->declare_parameter<bool>("publish_mesh", true);
    save_mesh_path_ = this->declare_parameter<std::string>("save_mesh_path", "");
    auto_reconstruct_ = this->declare_parameter<bool>("auto_reconstruct", true);
    use_registration_ = this->declare_parameter<bool>("use_registration", true);
    icp_max_correspondence_distance_ = this->declare_parameter<double>("icp_max_correspondence_distance", 0.03);
    icp_max_iterations_ = this->declare_parameter<int>("icp_max_iterations", 40);
    icp_transformation_epsilon_ = this->declare_parameter<double>("icp_transformation_epsilon", 1e-8);
    icp_fitness_epsilon_ = this->declare_parameter<double>("icp_fitness_epsilon", 1e-6);
    capture_active_ = auto_reconstruct_;

    if (z_limits.size() == 2) {
      z_min_ = z_limits[0];
      z_max_ = z_limits[1];
    }
    if (y_limits.size() == 2) {
      y_min_ = y_limits[0];
      y_max_ = y_limits[1];
    }

    auto qos = rclcpp::SensorDataQoS();
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic_, qos,
      std::bind(&HeadReconstructionNode::pointCloudCallback, this, std::placeholders::_1));

    filtered_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "filtered_pointcloud", rclcpp::SystemDefaultsQoS());
    accumulated_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "accumulated_pointcloud", rclcpp::SystemDefaultsQoS());
    registered_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "registered_pointcloud", rclcpp::SystemDefaultsQoS());
    mesh_pub_ = this->create_publisher<pcl_msgs::msg::PolygonMesh>(
      "reconstructed_mesh", rclcpp::SystemDefaultsQoS());

    start_capture_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "start_head_capture",
      std::bind(&HeadReconstructionNode::startCaptureService, this, std::placeholders::_1,
      std::placeholders::_2));
    stop_and_reconstruct_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "stop_and_reconstruct",
      std::bind(&HeadReconstructionNode::stopAndReconstructService, this, std::placeholders::_1,
      std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(),
      "Listening to %s and accumulating %d frames for reconstruction", pointcloud_topic_.c_str(),
      frames_to_accumulate_);
  }

private:
  void startCaptureService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    accumulated_cloud_->clear();
    accumulated_frames_ = 0;
    capture_active_ = true;
    have_last_header_ = false;
    response->success = true;
    response->message = "Head capture session started";
    RCLCPP_INFO(this->get_logger(), "Head capture session started");
  }

  void stopAndReconstructService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    capture_active_ = false;
    if (!have_last_header_) {
      response->success = false;
      response->message = "No head point cloud received during the session";
      return;
    }

    if (!runReconstruction(last_header_)) {
      response->success = false;
      response->message = "Failed to reconstruct mesh (not enough points?)";
      return;
    }

    capture_active_ = auto_reconstruct_;
    response->success = true;
    response->message = "Head reconstruction finished";
  }

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!capture_active_) {
      return;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*msg, *cloud);

    auto filtered = filterCloud(cloud);
    if (filtered->empty()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Filtered cloud is empty, check limits or sensor input");
      return;
    }

    publishPointCloud(filtered, filtered_cloud_pub_, msg->header);
    last_header_ = msg->header;
    have_last_header_ = true;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to_integrate = filtered;
    if (use_registration_ && !accumulated_cloud_->empty()) {
      if (!registerCloud(filtered, cloud_to_integrate)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "ICP failed to converge, skipping this frame");
        return;
      }
      publishPointCloud(cloud_to_integrate, registered_cloud_pub_, msg->header);
    }

    *accumulated_cloud_ += *cloud_to_integrate;
    accumulated_frames_++;

    if (auto_reconstruct_ &&
      (accumulated_cloud_->size() >= static_cast<size_t>(min_point_threshold_) ||
      accumulated_frames_ >= frames_to_accumulate_))
    {
      runReconstruction(msg->header);
    }
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterCloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud)
  {
    // Limit raw input to a loose head-sized ROI before downsampling.
    auto pass_z = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PassThrough<pcl::PointXYZRGB> pass_filter;
    pass_filter.setInputCloud(input_cloud);
    pass_filter.setFilterFieldName("z");
    pass_filter.setFilterLimits(z_min_, z_max_);
    pass_filter.filter(*pass_z);

    auto pass_y = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    pass_filter.setInputCloud(pass_z);
    pass_filter.setFilterFieldName("y");
    pass_filter.setFilterLimits(y_min_, y_max_);
    pass_filter.filter(*pass_y);

    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
    voxel_filter.setInputCloud(pass_y);
    voxel_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    auto downsampled = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    voxel_filter.filter(*downsampled);

    return downsampled;
  }

  bool runReconstruction(const std_msgs::msg::Header &header)
  {
    if (accumulated_cloud_->empty()) {
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Starting reconstruction using %zu points",
      accumulated_cloud_->size());

    // Estimate normals needed by the greedy triangulation.
    auto normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>());
    pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud(accumulated_cloud_);
    auto kdtree_rgb = pcl::search::KdTree<pcl::PointXYZRGB>::Ptr(new pcl::search::KdTree<pcl::PointXYZRGB>());
    ne.setSearchMethod(kdtree_rgb);
    ne.setKSearch(20);
    ne.compute(*normals);

    auto cloud_with_normals = pcl::PointCloud<pcl::PointNormal>::Ptr(
      new pcl::PointCloud<pcl::PointNormal>());
    pcl::concatenateFields(*accumulated_cloud_, *normals, *cloud_with_normals);

    auto kdtree = pcl::search::KdTree<pcl::PointNormal>::Ptr(new pcl::search::KdTree<pcl::PointNormal>());
    kdtree->setInputCloud(cloud_with_normals);

    constexpr double pi = 3.14159265358979323846;
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;  // Simple surface reconstruction
    gp3.setSearchRadius(voxel_leaf_size_ * 4.0);
    gp3.setMu(2.5);
    gp3.setMaximumNearestNeighbors(200);
    gp3.setMaximumSurfaceAngle(pi / 4.0);
    gp3.setMinimumAngle(pi / 18.0);
    gp3.setMaximumAngle(2.0 * pi / 3.0);
    gp3.setNormalConsistency(false);
    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(kdtree);

    pcl::PolygonMesh mesh;
    gp3.reconstruct(mesh);

    if (accumulated_cloud_pub_->get_subscription_count() > 0) {
      auto accumulated_msg = sensor_msgs::msg::PointCloud2();
      pcl::toROSMsg(*accumulated_cloud_, accumulated_msg);
      accumulated_msg.header.stamp = header.stamp;
      accumulated_msg.header.frame_id = reconstruction_frame_id_;
      accumulated_cloud_pub_->publish(accumulated_msg);
    }

    if (publish_mesh_ && mesh_pub_->get_subscription_count() > 0 && !mesh.polygons.empty()) {
      pcl_msgs::msg::PolygonMesh mesh_msg;
      pcl_conversions::fromPCL(mesh, mesh_msg);
      mesh_msg.header.stamp = header.stamp;
      mesh_msg.header.frame_id = reconstruction_frame_id_;
      mesh_pub_->publish(mesh_msg);
    }

    if (!save_mesh_path_.empty() && !mesh.polygons.empty()) {
      if (pcl::io::savePLYFileBinary(save_mesh_path_, mesh) == 0) {
        RCLCPP_INFO(this->get_logger(), "Mesh written to %s", save_mesh_path_.c_str());
      } else {
        RCLCPP_WARN(this->get_logger(), "Failed to save mesh to %s", save_mesh_path_.c_str());
      }
    }

    accumulated_cloud_->clear();
    accumulated_frames_ = 0;
    have_last_header_ = false;
    return true;
  }

  void publishPointCloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pub,
    const std_msgs::msg::Header &header)
  {
    if (!pub || pub->get_subscription_count() == 0) {
      return;
    }

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header = header;
    pub->publish(msg);
  }

  bool registerCloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &source_cloud,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &aligned_cloud)
  {
    if (!source_cloud || source_cloud->empty() || accumulated_cloud_->empty()) {
      aligned_cloud = source_cloud;
      return true;
    }

    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    icp.setMaxCorrespondenceDistance(icp_max_correspondence_distance_);
    icp.setMaximumIterations(icp_max_iterations_);
    icp.setTransformationEpsilon(icp_transformation_epsilon_);
    icp.setEuclideanFitnessEpsilon(icp_fitness_epsilon_);
    icp.setInputSource(source_cloud);
    icp.setInputTarget(accumulated_cloud_);

    pcl::PointCloud<pcl::PointXYZRGB> output;
    icp.align(output);

    if (!icp.hasConverged()) {
      RCLCPP_WARN(this->get_logger(), "ICP did not converge (fitness %.5f)", icp.getFitnessScore());
      return false;
    }

    aligned_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>(output));
    return true;
  }

  std::string pointcloud_topic_;
  std::string reconstruction_frame_id_;
  std::string save_mesh_path_;
  double voxel_leaf_size_;
  double z_min_{0.0};
  double z_max_{3.0};
  double y_min_{-0.5};
  double y_max_{0.5};
  int frames_to_accumulate_;
  int min_point_threshold_;
  bool publish_mesh_;
  bool auto_reconstruct_;
  bool use_registration_;
  bool capture_active_;
  bool have_last_header_{false};
  std_msgs::msg::Header last_header_;
  double icp_max_correspondence_distance_;
  int icp_max_iterations_;
  double icp_transformation_epsilon_;
  double icp_fitness_epsilon_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr accumulated_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr registered_cloud_pub_;
  rclcpp::Publisher<pcl_msgs::msg::PolygonMesh>::SharedPtr mesh_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_capture_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_and_reconstruct_srv_;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr accumulated_cloud_;
  int accumulated_frames_;
};

}  // namespace realsense_head_reconstruction

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<realsense_head_reconstruction::HeadReconstructionNode>());
  rclcpp::shutdown();
  return 0;
}
