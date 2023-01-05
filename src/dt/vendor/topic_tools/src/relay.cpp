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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <rclcpp/generic_publisher.hpp>
#include <rclcpp/generic_subscription.hpp>

#include <memory>
#include <string>

#include "topic_tools/policy_maps.hpp"

namespace topic_tools
{

class RelayNode : public rclcpp::Node
{
public:
  explicit RelayNode(const rclcpp::NodeOptions & node_options);

private:
  rclcpp::GenericSubscription::SharedPtr sub_;
  rclcpp::GenericPublisher::SharedPtr pub_;

  rmw_qos_reliability_policy_t reliability_policy_;
  rmw_qos_history_policy_t history_policy_;
  rmw_qos_durability_policy_t durability_policy_;
};

RelayNode::RelayNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("relay", node_options)
{
  auto input_topic = declare_parameter<std::string>("input_topic");
  auto output_topic = declare_parameter<std::string>("output_topic");

  // Parse reliability parameter
  {
    rcl_interfaces::msg::ParameterDescriptor reliability_desc;
    reliability_desc.description = "Reliability QoS setting for relay";
    reliability_desc.additional_constraints = "Must be one of: ";
    for (auto entry : name_to_reliability_policy_map) {
      reliability_desc.additional_constraints += entry.first + " ";
    }
    const std::string reliability_param = this->declare_parameter(
      "reliability", "reliable", reliability_desc);
    auto reliability = name_to_reliability_policy_map.find(reliability_param);
    if (reliability == name_to_reliability_policy_map.end()) {
      std::ostringstream oss;
      oss << "Invalid QoS reliability setting '" << reliability_param << "'";
      throw std::runtime_error(oss.str());
    }
    reliability_policy_ = reliability->second;
  }

  // Parse history parameter
  {
    rcl_interfaces::msg::ParameterDescriptor history_desc;
    history_desc.description = "History QoS setting for relay";
    history_desc.additional_constraints = "Must be one of: ";
    for (auto entry : name_to_history_policy_map) {
      history_desc.additional_constraints += entry.first + " ";
    }
    const std::string history_param = this->declare_parameter(
      "history", "keep_last", history_desc);
    auto history = name_to_history_policy_map.find(history_param);
    if (history == name_to_history_policy_map.end()) {
      std::ostringstream oss;
      oss << "Invalid QoS history setting '" << history_param << "'";
      throw std::runtime_error(oss.str());
    }
    history_policy_ = history->second;
  }

  // Parse durability parameter
  {
    rcl_interfaces::msg::ParameterDescriptor durability_desc;
    durability_desc.description = "Durability QoS setting for relay";
    durability_desc.additional_constraints = "Must be one of: ";
    for (auto entry : name_to_durability_policy_map) {
      durability_desc.additional_constraints += entry.first + " ";
    }
    const std::string durability_param = this->declare_parameter(
      "durability", "volatile", durability_desc);
    auto durability = name_to_durability_policy_map.find(durability_param);
    if (durability == name_to_durability_policy_map.end()) {
      std::ostringstream oss;
      oss << "Invalid QoS durability setting '" << durability_param << "'";
      throw std::runtime_error(oss.str());
    }
    durability_policy_ = durability->second;
  }

  auto depth = this->declare_parameter("depth", 1);

  // QoS configuration
  auto qos = rclcpp::QoS(
    rclcpp::QoSInitialization(history_policy_, depth));
  qos.reliability(reliability_policy_);
  qos.durability(durability_policy_);

  auto type = declare_parameter<std::string>("type");
  pub_ = this->create_generic_publisher(output_topic, type, qos);
  sub_ = this->create_generic_subscription(
    input_topic, type, qos,
    [this](std::shared_ptr<rclcpp::SerializedMessage> msg) {
      pub_->publish(*msg);
    });
}

}  // namespace topic_tools

RCLCPP_COMPONENTS_REGISTER_NODE(topic_tools::RelayNode)
