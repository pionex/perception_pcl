//*
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
// * $Id: pcd_io.cpp 35812 2011-02-08 00:05:03Z rusu $
// *
// */

#include <pcl_ros/io/pcd_io.hpp>
#include <thread>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/filesystem.hpp>

namespace pcl_ros
{
  PCDReader::PCDReader(const rclcpp::NodeOptions & options) : Node("PCDReader", options)
  {
    this->declare_parameter<double>("publish_rate", 1);
    this->declare_parameter<std::string>("file_name", "none");
    this->declare_parameter<std::string>("tf_frame", "map");

    cb_handle_ = this->add_on_set_parameters_callback(
        std::bind(&PCDReader::ParametersCallback, this, std::placeholders::_1));

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("output", 10);
    this->get_parameter("publish_rate", publish_rate_);
    this->get_parameter("tf_frame", tf_frame_);
    auto gfn = this->get_parameter("file_name", file_name_);
    RCLCPP_INFO(this->get_logger(), "Got Filename");
    std::flush(std::cout);

    thread_ = std::thread([this](){
      std::string file_name;
      std::string tf;
      double rate = 1.0;
      bool new_file = false;
      auto reader = pcl::PCDReader();

      while(rclcpp::ok())
      {
        //RCLCPP_INFO(this->get_logger(), "In Thread");
        std::flush(std::cout);

        std::lock_guard<std::mutex> lock(mutex_);
        {
          if (file_name != file_name_)
          {
            new_file = true;
            file_name = file_name_;
          }

          tf = tf_frame_;
          rate = publish_rate_;
        }

        if (new_file)
        {
          new_file = false;
          pcl::PCLPointCloud2 cloud;
          if (boost::filesystem::is_regular_file(file_name) && reader.read(file_name, cloud) < 0)
          {
            continue;
          }
          RCLCPP_INFO(this->get_logger(), "Got new_file %s", file_name.c_str());
          std::flush(std::cout);

          output_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
          pcl_conversions::moveFromPCL(cloud, *(output_));
          output_->header.stamp = now();
          output_->header.frame_id = tf;
        }

        if (output_)
            pub_->publish(*output_);

         std::this_thread::sleep_for(std::chrono::seconds(1));
      }
    });
  }

  rcl_interfaces::msg::SetParametersResult PCDReader::ParametersCallback(const std::vector<rclcpp::Parameter> &parameters)
  {
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;
    result.reason = "success";

    RCLCPP_INFO(this->get_logger(), "Got New Parameters");
    std::flush(std::cout);

    std::lock_guard<std::mutex> lock(mutex_);
    {
      for (const auto &p : parameters)
      {
        if (p.get_name() == "publish_rate")
          publish_rate_ = p.as_double();
        else if (p.get_name() == "tf_frame")
          tf_frame_ = p.as_string();
        else if (p.get_name() == "file_name")
          file_name_ = p.as_string();
      }
    }

    return result;
  }

//  PCDWriter::PCDWriter(const rclcpp::NodeOptions &options) : Node("PCDWriter", options)
//  {
//    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("input", 10, std::bind(&PCDWriter::input_callback, this, std::placeholders::_1));
//  }
//
//  void PCDWriter::input_callback(const sensor_msgs::msg::PointCloud2 cloud) const
//  {
//
//  }



} // namespace pcl_ros


