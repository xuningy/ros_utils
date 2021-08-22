/*
ros_utils/ParamUtils.h

Copyright (C) 2021 Xuning Yang

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#pragma once

#include <ros/ros.h>

namespace param_utils {

template <typename T>
inline bool get(const std::string& name, T& param, const T& default_val)
{
  std::string node = ros::this_node::getName();

  std::string default_print;
  if (std::is_same<T, bool>::value) {
    default_print = default_val ? std::string("true") : std::string("false");
  }
  else
  {
    default_print = std::to_string(default_val);
  }


  std::string key;
  if (!ros::param::search(name, key)) {
    ROS_WARN("%s: Parameter '%s' not found, using default value: '%s'", node.c_str(), name.c_str(), default_print.c_str() );
    param = default_val;
    return false;
  }

  if (!ros::param::get(key, param))
  {
    ROS_WARN("%s: Parameter '%s' not found, using default value: '%s'", node.c_str(), name.c_str(), default_print.c_str() );
    param = default_val;
    return false;
  }

  return true;
}

template <typename T>
inline bool get(const std::string& name, T& param)
{
  std::string node = ros::this_node::getName();

  std::string key;
  if (!ros::param::search(name, key)) {
    ROS_WARN("%s: Parameter '%s' not found", node.c_str(), name.c_str() );
    return false;
  }

  if (!ros::param::get(key, param))
  {
    ROS_WARN("%s: Parameter '%s' not found", node.c_str(), name.c_str() );
    return false;
  }

  return true;
}

} // namespace parameter_utils
