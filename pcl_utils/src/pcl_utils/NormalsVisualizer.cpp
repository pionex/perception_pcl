#include "pcl_utils/NormalsVisualizer.h"
#include <pcl_conversions/pcl_conversions.h>

pcl_utils::NormalsVisualizer::NormalsVisualizer(const rclcpp::NodeOptions &options)
{
  node_ = rclcpp::Node::make_shared("Normals", options);
  cloud_input_filter_.subscribe(node_, "cloud_input");
  normals_cloud_input_filter_.subscribe(node_, "normals_cloud_input");
  sync_clouds_a_ =
      std::make_shared<message_filters::Synchronizer<sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2,
                                                                                    sensor_msgs::msg::PointCloud2>>>(
          5);


  sync_clouds_a_->connectInput(cloud_input_filter_, normals_cloud_input_filter_);
  sync_clouds_a_->registerCallback(std::bind(&NormalsVisualizer::NewCloudsCallback,
                                             this,
                                             std::placeholders::_1,
                                             std::placeholders::_2));



  timer_ = node_->create_wall_timer(std::chrono::milliseconds(100), [this](){ RunOnce(); });
}

void pcl_utils::NormalsVisualizer::SetupRun()
{
  //Add
  //AddVertex(process_->cloud_data_.CapturePose, .08, "CameraBasis", VisualizerModule::CameraBasis);

  //Add steps that will be executed repeatedly
  AddRunExecutionStep({std::bind(&NormalsVisualizer::ShowNormals, this)});
}

void pcl_utils::NormalsVisualizer::ExtendedKeyboardCallback(const pcl::visualization::KeyboardEvent &event, void *viewer_void)
{

}

void pcl_utils::NormalsVisualizer::NewCloudsCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud, const sensor_msgs::msg::PointCloud2::ConstSharedPtr & normals_cloud)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (cloud && normals_cloud)
    {
      if (!cloud2_)
        cloud2_ = pcl::make_shared<pcl::PCLPointCloud2>();

      if (!normals_cloud2_)
        normals_cloud2_ = pcl::make_shared<pcl::PCLPointCloud2>();

      pcl_conversions::toPCL(*cloud, *cloud2_);
      pcl_conversions::toPCL(*normals_cloud, *normals_cloud2_);

      cloud_ = pcl::make_shared<PointCloudSTD>();
      normals_cloud_ = pcl::make_shared<NormalsPointCloud>();

      pcl::fromPCLPointCloud2(*cloud2_, *cloud_);
      pcl::fromPCLPointCloud2(*normals_cloud2_, *normals_cloud_);
    }

    }  // lock guard

  global_update_ = true;

  }



void pcl_utils::NormalsVisualizer::ShowNormals()
{
  const auto kNormalModule = 1;

  if (IsEnviromentUpdateRequested())
  {
    RemoveObjects(kNormalModule);

    if (cloud_ && normals_cloud_)
    {
      AddPointCloud(cloud_,
                    normals_cloud_,
                    "NormalsCloud", kNormalModule);
    }
  }
}



rclcpp::node_interfaces::NodeBaseInterface::SharedPtr pcl_utils::NormalsVisualizer::get_node_base_interface() const
{
  return this->node_->get_node_base_interface();
}

void pcl_utils::NormalsVisualizer::RunOnce()
{
  Visualizer::RunOnce();
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_utils::NormalsVisualizer)