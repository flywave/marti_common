// *****************************************************************************
//
// Copyright (c) 2018, Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL SOUTHWEST RESEARCH INSTITUTE BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************
#ifndef SWRI_ROSCPP_TOPIC_SERVICE_CLIENT_H_
#define SWRI_ROSCPP_TOPIC_SERVICE_CLIENT_H_

#include <ros/node_handle.h>
#include <ros/this_node.h>

#include <boost/thread/mutex.hpp>
#include <boost/uuid/random_generator.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <map>
#include <mutex>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

namespace swri
{
template<class MReq, class MRes>
class TopicServiceClientImpl
{
public:
  typedef
  boost::mutex request_lock_;
  ros::Subscriber response_sub_;
  ros::Publisher request_pub_;
  boost::shared_ptr<MRes> response_;

  std::chrono::nanoseconds timeout_;
  std::string name_;
  std::string service_name_;
  bool internal_spin_;

  int sequence_;

  TopicServiceClientImpl() : sequence_(0), timeout_(ros::Duration(4.0))
  {

  }

  void initialize(rclcpp::Node::SharedPtr node,
                const std::string &service,
                const std::string &client_name = "",
                bool internal_spin = true)
  {
    node_ = node;
    internal_spin_ = internal_spin;
    // Generate a quasi-random set of service names if the user did not
    // provide values. std::string::compare() returns 0 if both strings
    // have the same value. This is imperfect, but unless someone creates
    // the same service call at exactly the same time using a multithreaded node,
    // this should be safe
    if (client_name.compare("") == 0)
    {
      auto current_time = std::chrono::time_point_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now()).time_since_epoch().count();
      name_ = node_->get_name() + std::to_string(current_time);
    }
    else
    {
      name_ = client_name;
    }
    service_name_ = service;

    request_pub_ = nh.advertise<MReq>(rservice + "/request", 10);
    response_sub_ = nh.subscribe(rservice + "/response", 10, &TopicServiceClientImpl<MReq, MRes>::response_callback, this);
  }

  bool call(MReq& request, MRes& response)
  {
    std::lock_guard<std::mutex> scoped_lock(request_lock_);

    // block for response
    request.srv_header.stamp = node_->now();
    request.srv_header.sequence = sequence_;
    request.srv_header.sender = name_;

    response_.reset();
    request_pub_->publish(request);

    // Inspired by process in ClientBase::wait_for_service_nanoseconds in client.cpp
    auto start = std::chrono::steady_clock::now();
    std::chrono::nanoseconds time_to_wait = std::chrono::nanoseconds::max();
    if (timeout_ > std::chrono::nanoseconds(0))
    {
      time_to_wait = timeout_ - (std::chrono::steady_clock::now() - start);
    }

    // Wait until we get a response
    do
    {
      if (!rclcpp::ok())
      {
        return false;
      }

      if (response_)
      {
        break;
      }

      if (timeout_ > std::chrono::nanoseconds(0))
      {
        time_to_wait = timeout_ - (std::chrono::steady_clock::now() - start);
        rclcpp::sleep_for(std::chrono::milliseconds(2));
        if (internal_spin_)
        {
          rclcpp::spin_some(node_);
        }
      }
    } while (!response_ && (time_to_wait > std::chrono::nanoseconds(0)));

    sequence_++;
    if (response_)
    {
      response = *response_;
      response_.reset();
      return response.srv_header.result;
    }
    else
    {
      return false;
    }
  }

private:

  void response_callback(const std::shared_ptr<MRes> message)
  {
    RCLCPP_DEBUG(node_->get_logger(), "Got response for %s with sequence %i",
             message->srv_header.sender.c_str(), message->srv_header.sequence);

    if (message->srv_header.sender != name_)
    {
      RCLCPP_DEBUG(node_->get_logger(), "Got response from another client, ignoring..");
      return;
    }
    
    if (message->srv_header.sequence != sequence_)
    {
      RCLCPP_WARN(node_->get_logger(), "Got wrong sequence number, ignoring..");
      RCLCPP_DEBUG(node_->get_logger(), "message seq:%i vs current seq: %i", message->srv_header.sequence, sequence_);
      return;
    }

    response_ = message;
  }
};  // class TopicServiceClientImpl

template<class MReq>
class TopicServiceClient 
{
  boost::shared_ptr<TopicServiceClientImpl<typename MReq:: Request, typename MReq:: Response> > impl_;

public:

  void initialize(ros::NodeHandle &nh,
                const std::string &service,
                const std::string &client_name = "")
  {
    impl_ = boost::shared_ptr<TopicServiceClientImpl<typename MReq:: Request, typename MReq:: Response> >(
      new TopicServiceClientImpl<typename MReq:: Request, typename MReq:: Response>());

    impl_->initialize(nh, service, client_name);
  }

  std::string getService()
  {
    return impl_->service_name_;
  }

  bool exists()
  {
    return impl_->request_pub_.getNumSubscribers() > 0 && impl_->response_sub_.getNumPublishers() > 0;
  }

  bool call(MReq& req)
  {
    return impl_->call(req.request, req.response);
  }
};  // class TopicServiceClient


}  // namespace swri
#endif  // SWRI_ROSCPP_TOPIC_SERVICE_SERVER_H_
