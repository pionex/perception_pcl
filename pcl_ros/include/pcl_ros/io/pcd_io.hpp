/*
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
// * $Id: pcd_io.h 35054 2011-01-03 21:16:49Z rusu $
// *
// */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_ros/visibility_control.h>

namespace pcl_ros
{
 class PCDReader : public rclcpp::Node
 {
  public:
   COMPOSITION_PUBLIC
   explicit PCDReader(const rclcpp::NodeOptions & options);
   rcl_interfaces::msg::SetParametersResult ParametersCallback(const std::vector<rclcpp::Parameter> &parameters);

  protected:
   rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
   std::string tf_frame_;
   double publish_rate_;
   std::string file_name_;
   sensor_msgs::msg::PointCloud2::SharedPtr output_;
   std::mutex mutex_;
   std::thread thread_;
   OnSetParametersCallbackHandle::SharedPtr cb_handle_;
 };

// class PCDWriter : public rclcpp::Node
// {
//  public:
//   typedef sensor_msgs::msg::PointCloud2 PointCloud2;
//   typedef PointCloud2::Ptr PointCloud2Ptr;
//   typedef PointCloud2::ConstPtr PointCloud2ConstPtr;
//
//   COMPOSITION_PUBLIC
//   explicit PCDWriter(const rclcpp::NodeOptions & options);
//   //rcl_interfaces::msg::SetParametersResult ParametersCallback(const std::vector<rclcpp::Parameter> &parameters);
//
//  protected:
//    void input_callback(sensor_msgs::msg::PointCloud2 cloud) const;
//    std::string file_name_;
//    bool binary_mode_;
//    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
// };

}  //namespace pcl_ros

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_ros::PCDReader)


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///** \brief Point Cloud Data (PCD) file format writer.
//  * \author Radu Bogdan Rusu
//  */
//class PCDWriter //: public PCLNodelet
//{
//public:
//  PCDWriter()
//  : file_name_(""), binary_mode_(true) {}
//
//  typedef sensor_msgs::msg::PointCloud2 PointCloud2;
//  typedef PointCloud2::Ptr PointCloud2Ptr;
//  typedef PointCloud2::ConstPtr PointCloud2ConstPtr;
//
//  virtual void onInit();
//  void input_callback(const PointCloud2ConstPtr & cloud);
//
//  /** \brief The input PointCloud subscriber. */
//  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_input_;
//
//protected:
//  /** \brief The name of the file that contains the PointCloud data. */
//  std::string file_name_;
//
//  /** \brief Set to true if the output files should be saved in binary mode (true). */
//  bool binary_mode_;
//
//private:
//  /** \brief The PCL implementation used. */
//  pcl::PCDWriter impl_;
//
//public:
//  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//};
//}  // namespace pcl_ros
//
//#endif  // PCL_ROS__IO__PCD_IO_HPP_
