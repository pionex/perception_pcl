#include <Perception/Implementations/Visualizer.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <pcl_msgs/msg/point_indices.hpp>
#include <mutex>

namespace pcl_utils
{

namespace sync_policies = message_filters::sync_policies;

class NormalsVisualizer : public Visualizer
{
 public:
  NormalsVisualizer(const rclcpp::NodeOptions &options);
  void RunOnce() override;
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const;

 protected:
  void SetupRun() override;
  void ExtendedKeyboardCallback(const pcl::visualization::KeyboardEvent &event, void *viewer_void) override;

 private:
  void ShowNormals();
  void NewCloudsCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud,
                         const sensor_msgs::msg::PointCloud2::ConstSharedPtr &normals_cloud);

  rclcpp::Node::SharedPtr node_;
  //rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_input_filter_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> normals_cloud_input_filter_;
  std::shared_ptr<message_filters::Synchronizer<sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2,
                                                                               sensor_msgs::msg::PointCloud2>>>
      sync_clouds_a_;


  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr normals_cloud_msg_;

  pcl::PCLPointCloud2Ptr cloud2_;
  pcl::PCLPointCloud2Ptr normals_cloud2_;
  NormalsPointCloudPtr normals_cloud_;
  PointCloudSTDPtr cloud_;
  int max_queue_size_ = 10;
  std::mutex mutex_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}