/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014-2016, JSK Lab, University of Tokyo.
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
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
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
 *********************************************************************/
  
/* Author: Ryohei Ueda, Kentaro Wada <www.kentaro.wada@gmail.com>
 */
  
 #ifndef NODELET_LAZY_H_
 #define NODELET_LAZY_H_
  
 #include <rclcpp/rclcpp.hpp>
 //#include <nodelet/nodelet.h>
 #include <boost/thread.hpp>
 #include <string>
 #include <vector>
  

enum ConnectionStatus
    {
  NOT_INITIALIZED,
  NOT_SUBSCRIBED,
  SUBSCRIBED
    };
  
class NodeletLazy//: public nodelet::Nodelet
    {
//     public:
//      NodeletLazy() {}
//
//     protected:
//      virtual void onInit()
//      {
//        connection_status_ = NOT_SUBSCRIBED;
//        bool use_multithread;
//        ros::param::param<bool>("~use_multithread_callback", use_multithread, true);
//        if (use_multithread)
//        {
//          NODELET_DEBUG("Using multithread callback");
//          nh_.reset (new ros::NodeHandle(getMTNodeHandle()));
//          pnh_.reset (new ros::NodeHandle(getMTPrivateNodeHandle()));
//        }
//        else
//        {
//          NODELET_DEBUG("Using singlethread callback");
//          nh_.reset(new ros::NodeHandle(getNodeHandle()));
//          pnh_.reset(new ros::NodeHandle(getPrivateNodeHandle()));
//        }
//        // option to use lazy transport
//        pnh_->param("lazy", lazy_, true);
//        // option for logging about being subscribed
//        pnh_->param("verbose_connection", verbose_connection_, false);
//        if (!verbose_connection_)
//        {
//          nh_->param("verbose_connection", verbose_connection_, false);
//        }
//        // timer to warn when no connection in the specified seconds
//        ever_subscribed_ = false;
//        double duration_to_warn_no_connection;
//        pnh_->param("duration_to_warn_no_connection",
//                    duration_to_warn_no_connection, 5.0);
//        if (duration_to_warn_no_connection > 0)
//        {
//          timer_ever_subscribed_ = nh_->createWallTimer(
//              ros::WallDuration(duration_to_warn_no_connection),
//              &NodeletLazy::warnNeverSubscribedCallback,
//              this,
//              /*oneshot=*/true);
//        }
//      }
//
//      virtual void onInitPostProcess()
//      {
//        if (!lazy_)
//        {
//          boost::mutex::scoped_lock lock(connection_mutex_);
//          subscribe();
//          ever_subscribed_ = true;
//        }
//      }
//
////      virtual void connectionCallback(const ros::SingleSubscriberPublisher& pub)
////      {
////        if (verbose_connection_)
////        {
////          NODELET_INFO("New connection or disconnection is detected");
////        }
////        if (lazy_)
////        {
////          boost::mutex::scoped_lock lock(connection_mutex_);
////          for (size_t i = 0; i < publishers_.size(); i++)
////          {
////            ros::Publisher pub = publishers_[i];
////            if (pub.getNumSubscribers() > 0)
////            {
////              if (connection_status_ != SUBSCRIBED)
////              {
////                if (verbose_connection_)
////                {
////                  NODELET_INFO("Subscribe input topics");
////                }
////                subscribe();
////                connection_status_ = SUBSCRIBED;
////              }
////              if (!ever_subscribed_)
////              {
////                ever_subscribed_ = true;
////              }
////              return;
////            }
////          }
////          if (connection_status_ == SUBSCRIBED)
////          {
////            if (verbose_connection_)
////            {
////              NODELET_INFO("Unsubscribe input topics");
////            }
////            unsubscribe();
////            connection_status_ = NOT_SUBSCRIBED;
////          }
////        }
////      }
//
////      virtual void warnNeverSubscribedCallback(const ros::WallTimerEvent& event)
////      {
////        if (!ever_subscribed_)
////        {
////          NODELET_WARN("This node/nodelet subscribes topics only when subscribed.");
////        }
////      }
//
//
//      virtual void subscribe() = 0;
//
//      virtual void unsubscribe() = 0;
//
////      template<class T> rclcpp::Publisher<sensor_msgs::msg::PointCloud2>
////          advertise(ros::NodeHandle& nh,
////                    std::string topic, int queue_size, bool latch=false)
////                    {
////            boost::mutex::scoped_lock lock(connection_mutex_);
////            ros::SubscriberStatusCallback connect_cb
////            = boost::bind(&NodeletLazy::connectionCallback, this, _1);
////            ros::SubscriberStatusCallback disconnect_cb
////            = boost::bind(&NodeletLazy::connectionCallback, this, _1);
////            ros::Publisher pub = nh.advertise<T>(topic, queue_size,
////                                                 connect_cb,
////                                                 disconnect_cb,
////                                                 ros::VoidConstPtr(),
////                                                 latch);
////            publishers_.push_back(pub);
////            return pub;
////                    }
//
//      boost::mutex connection_mutex_;
//
//      //boost::shared_ptr<::NodeHandle> nh_;
//
//      //boost::shared_ptr<ros::NodeHandle> pnh_;
//
//      //std::vector<rclcpp::Publisher::> publishers_;
//
//      //ros::WallTimer timer_ever_subscribed_;
//
//      bool ever_subscribed_;
//
//      bool lazy_;
//
//      ConnectionStatus connection_status_;
//
//      bool verbose_connection_;
//
//     private:
    };

  
 #endif  // NODELET_LAZY_H_