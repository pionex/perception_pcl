/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 * $Id: passthrough.cpp 36194 2011-02-23 07:49:21Z rusu $
 *
 */

#include "pcl_ros/filters/passthrough.hpp"
#include <pluginlib/class_list_macros.hpp>


//////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl_ros::PassThrough::child_init()
{
  //param_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&PassThrough::parametersCallback, this, std::placeholders::_1));
  return true;
}

rcl_interfaces::msg::SetParametersResult pcl_ros::PassThrough::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters)
{
    {
      std::lock_guard<std::mutex> lock(mutex_);

//      filter_field_name_ = impl_.getFilterFieldName();
//      impl_.getFilterLimits(filter_limit_min_, filter_limit_max_);
//      filter_keep_organized_ = impl_.getKeepOrganized();
//      filter_limit_negative_ = impl_.getNegative();
//      tf_input_frame_ = impl_.

      for (const auto &p : parameters)
      {
        if (p.get_name() == "filter_limit_min")
          filter_limit_min_ = p.as_double();
        else if (p.get_name() == "filter_limit_max")
          filter_limit_max_ = p.as_double();
        else if (p.get_name() == "filter_field_name")
          filter_field_name_ = p.as_string();
        else if (p.get_name() == "keep_organized")
          filter_keep_organized_ = p.as_bool();
        else if (p.get_name() == "negative")
          filter_limit_negative_ = p.as_bool();
        else if (p.get_name() == "input_frame")
          tf_input_frame_ = p.as_string();
        else if (p.get_name() == "output_frame")
          tf_output_frame_ = p.as_string();
      }

      impl_.setFilterLimits(filter_limit_min_, filter_limit_max_);
      impl_.setFilterFieldName(filter_field_name_);
      impl_.setKeepOrganized(filter_keep_organized_);
      impl_.setNegative(filter_limit_negative_);

      got_initial_params_ = true;
    }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  // Here update class attributes, do some actions, etc.
  return result;
}

typedef pcl_ros::PassThrough PassThrough;
//PLUGINLIB_EXPORT_CLASS(PassThrough, nodelet::Nodelet);
#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_ros::PassThrough)
