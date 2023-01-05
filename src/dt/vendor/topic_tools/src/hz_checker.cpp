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

#include <memory>
#include <string>
#include <deque>

namespace topic_tools
{

class HzCheckerNode : public rclcpp::Node
{
public:
  explicit HzCheckerNode(const rclcpp::NodeOptions & node_options);

private:
  rclcpp::GenericSubscription::SharedPtr sub_;
  std::deque<rclcpp::Time> time_;
  std::size_t window_;
  double output_rate_;
  constexpr static std::size_t min_window_ = 2;
  void callback(std::shared_ptr<rclcpp::SerializedMessage> msg);
};

HzCheckerNode::HzCheckerNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("hz_checker", node_options)
{
  auto input_topic = declare_parameter<std::string>("input_topic");
  output_rate_ = declare_parameter("output_rate_", 2.0);
  window_ = static_cast<std::size_t>(declare_parameter("window", 10));

  if (window_ < min_window_) {
    RCLCPP_ERROR_STREAM(get_logger(), "minimum window size is " << min_window_);
    window_ = min_window_;
  }

  // get topic type
  std::string topic_type;
  bool found_topic = false;
  while (rclcpp::ok() && !found_topic) {
    const auto topics = get_topic_names_and_types();
    for (const auto & t : topics) {
      if (t.first == input_topic) {
        topic_type = t.second.front();
        found_topic = true;
      }
    }
    RCLCPP_INFO_STREAM_THROTTLE(
      get_logger(),
      *get_clock(), 1000 /* ms */, "waiting for " << input_topic);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  // set qos
  rclcpp::QoS qos{1};
  const auto publishers_info = get_publishers_info_by_topic(input_topic);
  if (!publishers_info.empty()) {
    const auto endpoint_info = publishers_info.front();
    qos = endpoint_info.qos_profile();
    RCLCPP_INFO_STREAM(get_logger(), "Success to get QoS profile of publisher.");
  }

  RCLCPP_INFO_STREAM(get_logger(), "subscribing to " << input_topic << " with type " << topic_type);

  sub_ = this->create_generic_subscription(
    input_topic, topic_type, qos,
    std::bind(&HzCheckerNode::callback, this, std::placeholders::_1));
}

void HzCheckerNode::callback(std::shared_ptr<rclcpp::SerializedMessage> msg)
{
  (void) msg;

  time_.push_back(now());
  while (time_.size() > window_ && rclcpp::ok()) {
    time_.pop_front();
  }

  if (time_.size() < min_window_) {
    return;
  }

  const auto total_duration = (time_.back() - time_.front()).seconds();
  const auto avg_duration = total_duration / (time_.size() - 1);

  double min_duration = total_duration;
  double max_duration = 0.0;
  for (std::size_t i = 1; i < time_.size(); i++) {
    const auto duration = (time_.at(i) - time_.at(i - 1)).seconds();
    if (min_duration > duration) {
      min_duration = duration;
    }
    if (max_duration < duration) {
      max_duration = duration;
    }
  }

  const auto output_period_ms = 1000 / output_rate_;
  RCLCPP_INFO_STREAM_THROTTLE(
    get_logger(), *get_clock(), output_period_ms,
    std::endl <<
      "avg rate: " << 1.0 / avg_duration << " avg sec: " << avg_duration << std::endl <<
      "min: " << min_duration << " max_duration: " << max_duration << " window: " << time_.size());
}

}  // namespace topic_tools

RCLCPP_COMPONENTS_REGISTER_NODE(topic_tools::HzCheckerNode)
