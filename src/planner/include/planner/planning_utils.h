
/**
 * Utility functions for planning project
 * AUTHOR: Yang Zhang
 */

#pragma once

#include <ros/ros.h>
#include <string>

namespace tamp {
namespace utils {

template <typename T>
bool GetParam(XmlRpc::XmlRpcValue &YamlNode, const std::string &param_name,
              T &data);

template <typename T>
bool GetParam(XmlRpc::XmlRpcValue &YamlNode, const std::string &param_name,
              std::vector<T> &data);

}  // namespace utils
}  // namespace tamp
#include "planning_utils_impl.h"