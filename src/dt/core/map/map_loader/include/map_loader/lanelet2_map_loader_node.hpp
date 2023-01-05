// Copyright 2021 Tier IV, Inc.
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

#ifndef MAP_LOADER__LANELET2_MAP_LOADER_NODE_HPP_
#define MAP_LOADER__LANELET2_MAP_LOADER_NODE_HPP_

#include "map_loader/lanelet2_map_visualization_node.hpp"

#include <lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/visualization/visualization.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_lanelet2_msgs/msg/lane2d.hpp>
#include <autoware_lanelet2_msgs/msg/lanes.hpp>
#include <autoware_lanelet2_msgs/msg/map_bin.hpp>
#include <autoware_lanelet2_msgs/msg/point2d.hpp>
#include <autoware_lanelet2_msgs/srv/lanes.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_projection/UTM.h>

#include <memory>
#include <vector>

class Lanelet2MapLoaderNode : public rclcpp::Node
{
public:
  explicit Lanelet2MapLoaderNode(const rclcpp::NodeOptions & options);

private:
  rclcpp::Publisher<autoware_lanelet2_msgs::msg::MapBin>::SharedPtr pub_map_bin_;
  rclcpp::Publisher<autoware_lanelet2_msgs::msg::Lanes>::SharedPtr pub_lanes_;
  rclcpp::Service<autoware_lanelet2_msgs::srv::Lanes>::SharedPtr frontend_map_service_;
  autoware_lanelet2_msgs::msg::Lanes lanes;
  void laneletToMapMsg(
    autoware_lanelet2_msgs::msg::Lanes & lanes, const lanelet::ConstLanelets & lanelets);
  void MapLoaderCallback(
    const std::shared_ptr<autoware_lanelet2_msgs::srv::Lanes::Request>,
    const std::shared_ptr<autoware_lanelet2_msgs::srv::Lanes::Response> response);
};

void Lanelet2MapLoaderNode::laneletToMapMsg(
  autoware_lanelet2_msgs::msg::Lanes & lanes, const lanelet::ConstLanelets & lanelets)
{
  autoware_lanelet2_msgs::msg::Lanes data;
  for (auto lanelet : lanelets) {
    autoware_lanelet2_msgs::msg::Lane2d lane2d;
    autoware_lanelet2_msgs::msg::Lane2d lane;
    for (auto point : lanelet.leftBound2d()) {
      autoware_lanelet2_msgs::msg::Point2d dataL;
      dataL.x = point.x();
      dataL.y = point.y();
      lane.left.push_back(dataL);
    }
    for (auto point : lanelet.rightBound2d()) {
      autoware_lanelet2_msgs::msg::Point2d dataR;
      dataR.x = point.x();
      dataR.y = point.y();
      lane.right.push_back(dataR);
    }
    lanes.lanes.push_back(lane);
  }
  return;
}

#endif  // MAP_LOADER__LANELET2_MAP_LOADER_NODE_HPP_
