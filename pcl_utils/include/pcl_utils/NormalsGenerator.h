#include <Perception/Implementations/Visualizer.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_msgs/msg/point_indices.hpp>
#include <mutex>
#include <g3log/g3log.hpp>

namespace pcl_utils
{

namespace sync_policies = message_filters::sync_policies;

class NormalsGenerator
{
 public:
  NormalsGenerator(const rclcpp::NodeOptions &options);
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const;


 private:
  void CloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud);
  void ProcessCloud();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr normals_cloud_publisher_;

  std::mutex mutex_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::PointCloud2::SharedPtr input_cloud_;
  sensor_msgs::msg::PointCloud2 output_normals_cloud_;
  std::atomic_bool cloud_processed_;
  std::unique_ptr<g3::LogWorker> log_worker_;
};
}