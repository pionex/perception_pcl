/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: filter.h 35876 2011-02-09 01:04:36Z rusu $
 *
 */

#ifndef PCL_ROS__FILTERS__FILTER_HPP_
#define PCL_ROS__FILTERS__FILTER_HPP_

#include <pcl/filters/filter.h>
//#include <dynamic_reconfigure/server.h>
#include <string>
#include "pcl_ros/pcl_nodelet.hpp"
//#include "pcl_ros/FilterConfig.hpp"

namespace pcl_ros
{
namespace sync_policies = message_filters::sync_policies;

/** \brief @b Filter represents the base filter class. Some generic 3D operations that are
  * applicable to all filters are defined here as static methods.
  * \author Radu Bogdan Rusu
  */
class Filter : public PCLNodelet
{
public:
  typedef sensor_msgs::msg::PointCloud2 PointCloud2;

  typedef pcl::IndicesPtr IndicesPtr;
  typedef pcl::IndicesConstPtr IndicesConstPtr;

  Filter(const std::string &node_name, const rclcpp::NodeOptions& options) : PCLNodelet(node_name, options), got_initial_params_(false)

   {
     RCLCPP_INFO(this->get_logger(), "Filter ctor");

     this->declare_parameter<double>("filter_limit_min", std::numeric_limits<double>::min());
     this->declare_parameter<double>("filter_limit_max", std::numeric_limits<double>::max());
     this->declare_parameter<std::string>("filter_field_name", "z");
     this->declare_parameter<bool>("keep_organized", false);
     this->declare_parameter<bool>("negative", false);
     this->declare_parameter<std::string>("input_frame", "");
     this->declare_parameter<std::string>("output_frame", "");

     this->get_parameter<double>("filter_limit_min", filter_limit_min_);
     this->get_parameter<double>("filter_limit_max", filter_limit_max_);
     this->get_parameter<std::string>("filter_field_name", filter_field_name_);
     this->get_parameter<bool>("keep_organized", filter_keep_organized_);
     this->get_parameter<bool>("negative", filter_limit_negative_);
     this->get_parameter<std::string>("input_frame", tf_input_frame_);
     this->get_parameter<std::string>("output_frame", tf_output_frame_);

     param_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&Filter::parametersCallback, this, std::placeholders::_1));

     subscribe();

     pub_output_ = this->create_publisher<PointCloud2>("filter_output",  10); //max_queue_size_);
     //timer_ = this->create_wall_timer(std::chrono::seconds(1), [this]() -> void { RCLCPP_INFO(this->get_logger(), "Staying alive"); });
   }

protected:
  /** \brief The input PointCloud subscriber. */
//  rclcpp::Subscription<PointCloud2>::SharedPtr sub_input_;
//
//  message_filters::Subscriber<PointCloud2> sub_input_filter_;

  /** \brief The desired user filter field name. */
  std::string filter_field_name_;

  /** \brief The minimum allowed filter value a point will be considered from. */
  double filter_limit_min_;

  /** \brief The maximum allowed filter value a point will be considered from. */
  double filter_limit_max_;

  /** \brief Set to true if we want to return the data outside
    * (\a filter_limit_min_;\a filter_limit_max_). Default: false.
    */
  bool filter_limit_negative_;

  bool filter_keep_organized_;

  //bool use_indices_;
  //int max_queue_size_;

  /** \brief The input TF frame the data should be transformed into,
    * if input.header.frame_id is different.
    */
  std::string tf_input_frame_;

  /** \brief The original data input TF frame. */
  std::string tf_input_orig_frame_;

  /** \brief The output TF frame the data should be transformed into,
    * if input.header.frame_id is different.
    */
  std::string tf_output_frame_;

//  tf2_ros::Buffer tf_buffer_;
//  tf2_ros::TransformListener tf_listener_;
  //rclcpp::Publisher<PointCloud2>::SharedPtr pub_output_;

  /** \brief Internal mutex. */
  std::mutex mutex_;

  std::atomic_bool got_initial_params_;
  rclcpp::TimerBase::SharedPtr timer_;

  /** \brief Child initialization routine.
    * \param nh ROS node handle
    * \param has_service set to true if the child has a Dynamic Reconfigure service
    */
//  virtual bool
//  child_init(bool & has_service)
//  {
//    has_service = false;
//    return true;
//  }

  /** \brief Virtual abstract filter method. To be implemented by every child.
    * \param input the input point cloud dataset.
    * \param indices a pointer to the vector of point indices to use.
    * \param output the resultant filtered PointCloud2
    */
  virtual void
  filter(
      const PointCloud2::ConstSharedPtr & input, const IndicesPtr & indices,
    PointCloud2 & output) = 0;

  /** \brief Lazy transport subscribe routine. */
  void
  subscribe();

  /** \brief Lazy transport unsubscribe routine. */
  void
  unsubscribe();

  /** \brief Nodelet initialization routine. */
  void onInit();

  /** \brief Call the child filter () method, optionally transform the result, and publish it.
    * \param input the input point cloud dataset.
    * \param indices a pointer to the vector of point indices to use.
    */
  void
  computePublish(const PointCloud2::ConstSharedPtr & input, const IndicesPtr & indices);

private:
  /** \brief Synchronized input, and indices.*/
  std::shared_ptr<message_filters::Synchronizer<sync_policies::ExactTime<PointCloud2,
    pcl_msgs::msg::PointIndices>>> sync_input_indices_e_;
  std::shared_ptr<message_filters::Synchronizer<sync_policies::ApproximateTime<PointCloud2,
    pcl_msgs::msg::PointIndices>>> sync_input_indices_a_;

  /** \brief Dynamic reconfigure service callback. */
  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  /** \brief PointCloud2 + Indices data callback. */
  void
  input_indices_callback(const PointCloud2::ConstSharedPtr & cloud,
                         const PointIndicesConstPtr & indices);

  /** \brief PointCloud2 callback. */
  void
  input_cloud_callback(const PointCloud2::ConstSharedPtr cloud);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace pcl_ros

#endif  // PCL_ROS__FILTERS__FILTER_HPP_
