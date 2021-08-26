///*
// * Software License Agreement (BSD License)
// *
// *  Copyright (c) 2009, Willow Garage, Inc.
// *  All rights reserved.
// *
// *  Redistribution and use in source and binary forms, with or without
// *  modification, are permitted provided that the following conditions
// *  are met:
// *
// *   * Redistributions of source code must retain the above copyright
// *     notice, this list of conditions and the following disclaimer.
// *   * Redistributions in binary form must reproduce the above
// *     copyright notice, this list of conditions and the following
// *     disclaimer in the documentation and/or other materials provided
// *     with the distribution.
// *   * Neither the name of Willow Garage, Inc. nor the names of its
// *     contributors may be used to endorse or promote products derived
// *     from this software without specific prior written permission.
// *
// *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// *  POSSIBILITY OF SUCH DAMAGE.
// *
// * $Id: filter.cpp 35876 2011-02-09 01:04:36Z rusu $
// *
// */
//
#include <pcl_ros/transforms.hpp>
#include "pcl_ros/filters/PeceptionPipeline.hpp"
#include <Perception/Implementations/PlaneProcessor.h>


//#include <pcl/io/io.h>
//#include <vector>
//#include "pcl_ros/transforms.hpp"
//
///*//#include <pcl/filters/pixel_grid.h>
////#include <pcl/filters/filter_dimension.h>
//*/
//
///*//typedef pcl::PixelGrid PixelGrid;
////typedef pcl::FilterDimension FilterDimension;
//*/
//
// Include the implementations instead of compiling them separately to speed up compile time
// #include "extract_indices.cpp"
// #include "passthrough.cpp"
// #include "project_inliers.cpp"
// #include "radius_outlier_removal.cpp"
// #include "statistical_outlier_removal.cpp"
// #include "voxel_grid.cpp"

/*//PLUGINLIB_EXPORT_CLASS(PixelGrid,nodelet::Nodelet);
//PLUGINLIB_EXPORT_CLASS(FilterDimension,nodelet::Nodelet);
*/

///////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::PerceptionPipeline::computePublish(const PointCloud2::ConstSharedPtr & input, const IndicesPtr & indices)
{

  CloudData data;


  PointCloud2 output;
  // Call the virtual method in the child
  filter(input, indices, output);

  PointCloud2::SharedPtr cloud_tf(new PointCloud2(output));     // set the output by default
  // Check whether the user has given a different output TF frame
  if (!tf_output_frame_.empty() && output.header.frame_id != tf_output_frame_) {
    NODELET_DEBUG(
      "[%s::computePublish] Transforming output dataset from %s to %s.",
      this->get_name(), output.header.frame_id.c_str(), tf_output_frame_.c_str());
    // Convert the cloud into the different frame
    PointCloud2 cloud_transformed;
    if (!pcl_ros::transformPointCloud(tf_output_frame_, output, cloud_transformed, tf_buffer_)) {
//      NODELET_ERROR(
//        "[%s::computePublish] Error converting output dataset from %s to %s.",
//        this->get_name(), output.header.frame_id.c_str(), tf_output_frame_.c_str());
//      return;
    }
    cloud_tf.reset(new PointCloud2(cloud_transformed));
  }
  if (tf_output_frame_.empty() && output.header.frame_id != tf_input_orig_frame_) {
    // no tf_output_frame given, transform the dataset to its original frame
    NODELET_DEBUG(
      "[%s::computePublish] Transforming output dataset from %s back to %s.",
      this->get_name(), output.header.frame_id.c_str(), tf_input_orig_frame_.c_str());
    // Convert the cloud into the different frame
    PointCloud2 cloud_transformed;
    if (!pcl_ros::transformPointCloud(
        tf_input_orig_frame_, output, cloud_transformed,
        tf_buffer_))
    {
//      NODELET_ERROR(
//        "[%s::computePublish] Error converting output dataset from %s back to %s.",
//        getName().c_str(), output.header.frame_id.c_str(), tf_input_orig_frame_.c_str());
      return;
    }
    cloud_tf.reset(new PointCloud2(cloud_transformed));
  }

  // Copy timestamp to keep it
  cloud_tf->header.stamp = input->header.stamp;

  // Publish a boost shared ptr
  pub_output_->publish(*cloud_tf);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::PerceptionPipeline::subscribe()
{
  // If we're supposed to look for PointIndices (indices)
  //use_indices_ = false; // todo - hard coding this until fix race condition with params

  if (use_indices_)
  {
    RCLCPP_INFO(this->get_logger(), "Complex Sub");
    // Subscribe to the input using a filter

    sub_input_filter_.subscribe(this, "input"); //, max_queue_size_);
    sub_indices_filter_.subscribe(this, "indices"); //, max_queue_size_);

    if (approximate_sync_) {
      sync_input_indices_a_ =
        std::make_shared<message_filters::Synchronizer<sync_policies::ApproximateTime<PointCloud2,
          pcl_msgs::msg::PointIndices>>>(max_queue_size_);
      sync_input_indices_a_->connectInput(sub_input_filter_, sub_indices_filter_);
      sync_input_indices_a_->registerCallback(std::bind(&PerceptionPipeline::input_indices_callback, this, std::placeholders::_1, std::placeholders::_2));
    } else {
      sync_input_indices_e_ =
        std::make_shared<message_filters::Synchronizer<sync_policies::ExactTime<PointCloud2,
          pcl_msgs::msg::PointIndices>>>(max_queue_size_);
      sync_input_indices_e_->connectInput(sub_input_filter_, sub_indices_filter_);
      sync_input_indices_e_->registerCallback(std::bind(&PerceptionPipeline::input_indices_callback, this, std::placeholders::_1, std::placeholders::_2));
    }
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Basic Sub");
     //Subscribe in an old fashion to input only (no filters)
     rclcpp::QoS depth_qos(10);
     depth_qos.keep_last(10);
     depth_qos.best_effort();
     depth_qos.durability_volatile();
     sub_input_ = this->create_subscription<PointCloud2>("input", 10, std::bind(&PerceptionPipeline::input_cloud_callback, this, std::placeholders::_1));;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::PerceptionPipeline::unsubscribe()
{
  if (use_indices_) {
    sub_input_filter_.unsubscribe();
    sub_indices_filter_.unsubscribe();
  } else {
    sub_input_.reset();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::PerceptionPipeline::onInit()
{

}

//////////////////////////////////////////////////////////////////////////////////////////////
rcl_interfaces::msg::SetParametersResult pcl_ros::PerceptionPipeline::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters)
    {
    // The following parameters are updated automatically for all PCL_ROS Nodelet Filters as they are
    // inexistent in PCL

    RCLCPP_INFO(this->get_logger(), "In Params Callback");

      for (const auto &p : parameters)
      {
        if (p.get_name() == "input_frame")
          tf_input_frame_ = p.as_string();
        else if (p.get_name() == "output_frame")
          tf_output_frame_ = p.as_string();
      }

      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      result.reason = "success";
      // Here update class attributes, do some actions, etc.
      return result;
    }

void
pcl_ros::PerceptionPipeline::input_cloud_callback(const PointCloud2::ConstSharedPtr cloud)
{
  input_indices_callback(cloud, PointIndicesConstPtr());
}
//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::PerceptionPipeline::input_indices_callback(
  const PointCloud2::ConstSharedPtr & cloud,
  const PointIndicesConstPtr & indices)
{
  // If cloud is given, check if it's valid
  if (!isValid(cloud)) {
    NODELET_ERROR("[%s::input_indices_callback] Invalid input!", getName().c_str());
    return;
  }
  // If indices are given, check if they are valid
  if (indices && !isValid(indices)) {
    NODELET_ERROR("[%s::input_indices_callback] Invalid indices!", getName().c_str());
    return;
  }

  /// DEBUG
  if (indices) {
    NODELET_DEBUG(
      "[%s::input_indices_callback]\n"
      "                                 - PointCloud with %d data points (%s), stamp %f, and "
      "frame %s on topic %s received.\n"
      "                                 - PointIndices with %zu values, stamp %f, and "
      "frame %s on topic %s received.",
      getName().c_str(),
      cloud->width * cloud->height, pcl::getFieldsList(*cloud).c_str(),
      cloud->header.stamp.toSec(), cloud->header.frame_id.c_str(), pnh_->resolveName(
        "input").c_str(),
      indices->indices.size(), indices->header.stamp.toSec(),
      indices->header.frame_id.c_str(), pnh_->resolveName("indices").c_str());
  } else {
    NODELET_DEBUG(
      "[%s::input_indices_callback] PointCloud with %d data points and frame %s on "
      "topic %s received.",
      getName().c_str(), cloud->width * cloud->height,
      cloud->header.frame_id.c_str(), pnh_->resolveName("input").c_str());
  }
  ///

  // Check whether the user has given a different input TF frame
  tf_input_orig_frame_ = cloud->header.frame_id;
  PointCloud2::ConstSharedPtr cloud_tf;
  if (!tf_input_frame_.empty() && cloud->header.frame_id != tf_input_frame_) {
    NODELET_DEBUG(
      "[%s::input_indices_callback] Transforming input dataset from %s to %s.",
      getName().c_str(), cloud->header.frame_id.c_str(), tf_input_frame_.c_str());
    // Save the original frame ID
    // Convert the cloud into the different frame
    PointCloud2 cloud_transformed;
    if (!pcl_ros::transformPointCloud(tf_input_frame_, *cloud, cloud_transformed, tf_buffer_)) {
      NODELET_ERROR(
        "[%s::input_indices_callback] Error converting input dataset from %s to %s.",
        getName().c_str(), cloud->header.frame_id.c_str(), tf_input_frame_.c_str());
      return;
    }
    cloud_tf = std::make_shared<PointCloud2>(cloud_transformed);
  } else {
    cloud_tf = cloud;
  }

  // Need setInputCloud () here because we have to extract x/y/z
  IndicesPtr vindices;
  if (indices) {
    vindices.reset(new std::vector<int>(indices->indices));
  }

  computePublish(cloud_tf, vindices);
}
