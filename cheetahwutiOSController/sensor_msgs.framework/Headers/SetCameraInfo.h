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
 * Auto-generated by gensrv_cpp from file /Users/Ronan/ros_for_ios/ros/ros_msgs/common_msgs/sensor_msgs/srv/SetCameraInfo.srv
 *
 */


#ifndef SENSOR_MSGS_MESSAGE_SETCAMERAINFO_H
#define SENSOR_MSGS_MESSAGE_SETCAMERAINFO_H

#include <ros/service_traits.h>


#include <sensor_msgs/SetCameraInfoRequest.h>
#include <sensor_msgs/SetCameraInfoResponse.h>


namespace sensor_msgs
{

struct SetCameraInfo
{

typedef SetCameraInfoRequest Request;
typedef SetCameraInfoResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SetCameraInfo
} // namespace sensor_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::sensor_msgs::SetCameraInfo > {
  static const char* value()
  {
    return "bef1df590ed75ed1f393692395e15482";
  }

  static const char* value(const ::sensor_msgs::SetCameraInfo&) { return value(); }
};

template<>
struct DataType< ::sensor_msgs::SetCameraInfo > {
  static const char* value()
  {
    return "sensor_msgs/SetCameraInfo";
  }

  static const char* value(const ::sensor_msgs::SetCameraInfo&) { return value(); }
};


// service_traits::MD5Sum< ::sensor_msgs::SetCameraInfoRequest> should match 
// service_traits::MD5Sum< ::sensor_msgs::SetCameraInfo > 
template<>
struct MD5Sum< ::sensor_msgs::SetCameraInfoRequest>
{
  static const char* value()
  {
    return MD5Sum< ::sensor_msgs::SetCameraInfo >::value();
  }
  static const char* value(const ::sensor_msgs::SetCameraInfoRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::sensor_msgs::SetCameraInfoRequest> should match 
// service_traits::DataType< ::sensor_msgs::SetCameraInfo > 
template<>
struct DataType< ::sensor_msgs::SetCameraInfoRequest>
{
  static const char* value()
  {
    return DataType< ::sensor_msgs::SetCameraInfo >::value();
  }
  static const char* value(const ::sensor_msgs::SetCameraInfoRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::sensor_msgs::SetCameraInfoResponse> should match 
// service_traits::MD5Sum< ::sensor_msgs::SetCameraInfo > 
template<>
struct MD5Sum< ::sensor_msgs::SetCameraInfoResponse>
{
  static const char* value()
  {
    return MD5Sum< ::sensor_msgs::SetCameraInfo >::value();
  }
  static const char* value(const ::sensor_msgs::SetCameraInfoResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::sensor_msgs::SetCameraInfoResponse> should match 
// service_traits::DataType< ::sensor_msgs::SetCameraInfo > 
template<>
struct DataType< ::sensor_msgs::SetCameraInfoResponse>
{
  static const char* value()
  {
    return DataType< ::sensor_msgs::SetCameraInfo >::value();
  }
  static const char* value(const ::sensor_msgs::SetCameraInfoResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // SENSOR_MSGS_MESSAGE_SETCAMERAINFO_H