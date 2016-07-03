/* Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Auto-generated by gensrv_cpp from file /Users/Ronan/ros_for_ios/ros/geometry_experimental/tf2_msgs/srv/FrameGraph.srv
 *
 */


#ifndef TF2_MSGS_MESSAGE_FRAMEGRAPH_H
#define TF2_MSGS_MESSAGE_FRAMEGRAPH_H

#include <ros/service_traits.h>


#include <tf2_msgs/FrameGraphRequest.h>
#include <tf2_msgs/FrameGraphResponse.h>


namespace tf2_msgs
{

struct FrameGraph
{

typedef FrameGraphRequest Request;
typedef FrameGraphResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct FrameGraph
} // namespace tf2_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::tf2_msgs::FrameGraph > {
  static const char* value()
  {
    return "437ea58e9463815a0d511c7326b686b0";
  }

  static const char* value(const ::tf2_msgs::FrameGraph&) { return value(); }
};

template<>
struct DataType< ::tf2_msgs::FrameGraph > {
  static const char* value()
  {
    return "tf2_msgs/FrameGraph";
  }

  static const char* value(const ::tf2_msgs::FrameGraph&) { return value(); }
};


// service_traits::MD5Sum< ::tf2_msgs::FrameGraphRequest> should match 
// service_traits::MD5Sum< ::tf2_msgs::FrameGraph > 
template<>
struct MD5Sum< ::tf2_msgs::FrameGraphRequest>
{
  static const char* value()
  {
    return MD5Sum< ::tf2_msgs::FrameGraph >::value();
  }
  static const char* value(const ::tf2_msgs::FrameGraphRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::tf2_msgs::FrameGraphRequest> should match 
// service_traits::DataType< ::tf2_msgs::FrameGraph > 
template<>
struct DataType< ::tf2_msgs::FrameGraphRequest>
{
  static const char* value()
  {
    return DataType< ::tf2_msgs::FrameGraph >::value();
  }
  static const char* value(const ::tf2_msgs::FrameGraphRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::tf2_msgs::FrameGraphResponse> should match 
// service_traits::MD5Sum< ::tf2_msgs::FrameGraph > 
template<>
struct MD5Sum< ::tf2_msgs::FrameGraphResponse>
{
  static const char* value()
  {
    return MD5Sum< ::tf2_msgs::FrameGraph >::value();
  }
  static const char* value(const ::tf2_msgs::FrameGraphResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::tf2_msgs::FrameGraphResponse> should match 
// service_traits::DataType< ::tf2_msgs::FrameGraph > 
template<>
struct DataType< ::tf2_msgs::FrameGraphResponse>
{
  static const char* value()
  {
    return DataType< ::tf2_msgs::FrameGraph >::value();
  }
  static const char* value(const ::tf2_msgs::FrameGraphResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // TF2_MSGS_MESSAGE_FRAMEGRAPH_H
