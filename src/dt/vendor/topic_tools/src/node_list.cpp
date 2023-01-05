// Copyright 2020 Apex.AI GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace topic_tools
{

class NodeList : public rclcpp::Node
{
public:
  explicit NodeList(const rclcpp::NodeOptions & node_options);

private:
};

NodeList::NodeList(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("node_list", node_options)
{
  RCLCPP_INFO(get_logger(), "start searching nodes");
  rclcpp::Rate(1.0).sleep();
  std::vector<std::string> node_names = get_node_names();
  RCLCPP_INFO(
    get_logger(), "found %lu nodes. (%lu and me)", node_names.size(),
    node_names.size() - 1);
  RCLCPP_INFO(get_logger(), "--------------------------------------------");

  for (const auto & n : node_names) {
    RCLCPP_INFO(get_logger(), "%s", n.c_str());
  }

  RCLCPP_INFO(get_logger(), "--------------------------------------------");
  rclcpp::shutdown();
}

}  // namespace topic_tools

RCLCPP_COMPONENTS_REGISTER_NODE(topic_tools::NodeList)
