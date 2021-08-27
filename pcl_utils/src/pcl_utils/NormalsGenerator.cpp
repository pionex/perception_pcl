#include "pcl_utils/NormalsGenerator.h"
#include <pcl_conversions/pcl_conversions.h>
#include <Perception/Implementations/NormalCloudProvider.h>
#include <Perception/Implementations/ConfigurationProvider.h>
#include <g3log/logworker.hpp>



pcl_utils::NormalsGenerator::NormalsGenerator(const rclcpp::NodeOptions &options) : cloud_processed_(false)
{
  node_ = rclcpp::Node::make_shared("NormalsGenerator", options);
  subscription_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>("raw_cloud",
                                                                            10,
                                                                            std::bind(&NormalsGenerator::CloudCallback,
                                                                                      this,
                                                                                      std::placeholders::_1));
  cloud_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_output", 10);
  normals_cloud_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("normals_cloud_output", 10);

  rcl_interfaces::msg::ParameterDescriptor max_depth_descriptor;
  rcl_interfaces::msg::FloatingPointRange max_depth_range;
  max_depth_range.set__from_value(0.0f).set__to_value(10.0f).set__step(.1);
  max_depth_descriptor.floating_point_range = {max_depth_range};

  node_->declare_parameter<double>("normal_max_depth_change", NORMAL_MAX_DEPTH_CHANGE * 1000.0f, max_depth_descriptor);


  rcl_interfaces::msg::ParameterDescriptor smoothing_descriptor;
  rcl_interfaces::msg::FloatingPointRange smoothing_range;
  smoothing_range.set__from_value(0.0f).set__to_value(50.0f).set__step(1.0f);
  smoothing_descriptor.floating_point_range = {smoothing_range};

  node_->declare_parameter<double>("normal_smoothing", NORMAL_SMOOTHING, smoothing_descriptor);

  node_->get_parameter("normal_max_depth_change", normal_max_depth_change_);
  node_->get_parameter("normal_smoothing", normal_smoothing_);

  param_cb_handle_ = node_->add_on_set_parameters_callback(std::bind(&NormalsGenerator::ParametersCallback, this, std::placeholders::_1));

  timer_ = node_->create_wall_timer(std::chrono::milliseconds(100), [this]()
  {
    ProcessCloud();
    if (input_cloud_)
    {
      input_cloud_->header.stamp = node_->get_clock()->now();
      output_normals_cloud_.header.stamp = input_cloud_->header.stamp;

      cloud_publisher_->publish(*input_cloud_);
      normals_cloud_publisher_->publish(output_normals_cloud_);
    }
  });

  using namespace g3;
  //g3::overrideSetupSignals({{SIGABRT, "SIGABRT"}, {SIGFPE, "SIGFPE"}, {SIGILL, "SIGILL"}});
  log_worker_ = LogWorker::createLogWorker();
  //systemd_sink_ = log_worker_->addSink(std::make_unique<SystemdSink>(), &SystemdSink::ReceiveLogMessage);
  initializeLogging(log_worker_.get());
}

rcl_interfaces::msg::SetParametersResult pcl_utils::NormalsGenerator::ParametersCallback(const std::vector<rclcpp::Parameter> &parameters)
{
  auto result = rcl_interfaces::msg::SetParametersResult();
  result.successful = true;
  result.reason = "success";

  RCLCPP_INFO(node_->get_logger(), "Params Callback");

  std::lock_guard<std::mutex> lock(mutex_);
  {
    for (const auto &p : parameters)
    {
      if (p.get_name() == "normal_max_depth_change")
        normal_max_depth_change_ = p.as_double();
      else if (p.get_name() == "normal_smoothing")
        normal_smoothing_ = p.as_double();
    }

    cloud_processed_ = false;
  }

  return result;
}

void pcl_utils::NormalsGenerator::ProcessCloud()
{
  if (!cloud_processed_ && input_cloud_)
  {
    RCLCPP_INFO(node_->get_logger(), "Processing Cloud");
    auto config_provider = std::make_shared<ConfigurationProvider>();
    AppConfig::SetProvider(config_provider);
    NormalCloudProvider::Config config_NormalCloudProvider{
        .MaxDepthChange = static_cast<float>(normal_max_depth_change_ / 1000.0f),
        .NormalSmoothing = static_cast<float>(normal_smoothing_)
    };
    AppConfig::SetConfig<NormalCloudProvider>(config_NormalCloudProvider);

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

void pcl_utils::NormalsGenerator::CloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (input_cloud_)
    {
      // set the timestamp of the old cloud to incoming so that when we compare below, we don't care if the stamps are different
      input_cloud_->header.stamp = cloud->header.stamp;
    }

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