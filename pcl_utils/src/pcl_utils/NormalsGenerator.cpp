#include "pcl_utils/NormalsGenerator.h"
#include <pcl_conversions/pcl_conversions.h>
#include <Perception/Implementations/NormalCloudProvider.h>

pcl_utils::NormalsGenerator::NormalsGenerator(const rclcpp::NodeOptions &options) : cloud_processed_(false)
{
  node_ = rclcpp::Node::make_shared("NormalsGenerator", options);
  subscription_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>("raw_cloud",
                                                                            10,
                                                                            std::bind(&NormalsGenerator::CloudCallback,
                                                                                      this,
                                                                                      std::placeholders::_1));
  cloud_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_input", 10);
  normals_cloud_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("normals_cloud_input", 10);
  timer_ = node_->create_wall_timer(std::chrono::milliseconds(100), [this]()
  {
    ProcessCloud();
    if (input_cloud_)
    {
      cloud_publisher_->publish(*input_cloud_);
      normals_cloud_publisher_->publish(output_normals_cloud_);
    }
  });
}

void pcl_utils::NormalsGenerator::ProcessCloud()
{
  if (!cloud_processed_ && input_cloud_)
  {
    CloudData data;
    pcl::PCLPointCloud2 cld;
    pcl_conversions::toPCL(*input_cloud_, cld);
    data.CloudPtr = pcl::make_shared<PointCloudSTD>();
    pcl::fromPCLPointCloud2(cld, *data.CloudPtr);
    auto ncp = NormalCloudProvider(data);
    auto ncloud = ncp.GetNormalsCloud();
    pcl::PCLPointCloud2 ncld;
    pcl::toPCLPointCloud2(*ncloud, ncld);
    pcl_conversions::fromPCL(ncld, output_normals_cloud_);
    cloud_processed_ = true;
  }
}

void pcl_utils::NormalsGenerator::CloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!input_cloud_ || (input_cloud_ && *cloud != *input_cloud_))
    {
      input_cloud_ = cloud;
      cloud_processed_ = false;
    }
  }  // lock guard

}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr pcl_utils::NormalsGenerator::get_node_base_interface() const
{
  return this->node_->get_node_base_interface();
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_utils::NormalsGenerator)