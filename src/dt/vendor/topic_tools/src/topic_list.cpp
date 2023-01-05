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

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace topic_tools
{

class TopicList : public rclcpp::Node
{
public:
  explicit TopicList(const rclcpp::NodeOptions & node_options);

private:
};

TopicList::TopicList(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("topic_list", node_options)
{
  RCLCPP_INFO(get_logger(), "start searching topics");
  rclcpp::Rate(0.2).sleep();
  std::map<std::string, std::vector<std::string>> topic_names_and_types =
    get_topic_names_and_types();
  RCLCPP_INFO(get_logger(), "found %lu topics.", topic_names_and_types.size());
  RCLCPP_INFO(get_logger(), "--------------------------------------------");

  for (const auto & t : topic_names_and_types) {
    std::stringstream ss;
    ss << t.first.c_str() << " : ";
    for (const auto & type : t.second) {
      ss << "[" << type << "]";
    }
    RCLCPP_INFO(get_logger(), "%s", rclcpp::get_c_string(ss.str()));
  }

  RCLCPP_INFO(get_logger(), "--------------------------------------------");
  rclcpp::shutdown();
}

}  // namespace topic_tools

RCLCPP_COMPONENTS_REGISTER_NODE(topic_tools::TopicList)
