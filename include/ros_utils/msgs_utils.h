/*
ros_utils/msgs_utils.h

Copyright (C) 2021 Xuning Yang

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#pragma once

#include <stdexcept>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>


namespace ros_utils {
// converts various data types to ros msg types.
namespace msgs {

template <typename T>
geometry_msgs::Point toPoint(const std::vector<T>& vec)
{
  if (vec.size() != 3) throw std::invalid_argument("[ros_utils::msgs::toPoint] vector needs to be of length 3");

  geometry_msgs::Point point;
  point.x = vec[0];
  point.y = vec[1];
  point.z = vec[2];

  return point;
}

inline geometry_msgs::Point toPoint(const Eigen::Vector3d& vec)
{
  geometry_msgs::Point point;
  point.x = vec(0);
  point.y = vec(1);
  point.z = vec(2);

  return point;
}

inline Eigen::Vector3d fromPointToEigen(const geometry_msgs::Point& point)
{
  Eigen::Vector3d vec(point.x, point.y, point.z);
  return vec;
}

inline Eigen::Vector3d fromVectorToEigen(const geometry_msgs::Vector3& vec3)
{
  Eigen::Vector3d vec(vec3.x, vec3.y, vec3.z);
  return vec;
}

inline std::vector<double> fromPointToVec(const geometry_msgs::Point& point)
{
  std::vector<double> vec;
  vec.push_back(point.x);
  vec.push_back(point.y);
  vec.push_back(point.z);

  return vec;
}

inline Eigen::Quaterniond fromQuatToEigenQuat(const geometry_msgs::Quaternion& quat_msg)
{
  return Eigen::Quaterniond(quat_msg.w, quat_msg.x, quat_msg.y, quat_msg.z);
}

inline Eigen::Vector3d fromQuatToEigenRPY(const geometry_msgs::Quaternion& quat_msg)
{
  Eigen::Quaternion quat_eigen = fromQuatToEigenQuat(quat_msg);
  Eigen::Vector3d rpy = quat_eigen.toRotationMatrix().eulerAngles(0, 1, 2);

  return rpy;
}

} // namespace msgs

} // namespace ros_utils
