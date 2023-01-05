// Copyright 2021 TierIV
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

/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Authors: Simon Thompson, Ryohsuke Mitsudome
 *
 */

#include "map_loader/lanelet2_map_loader_node.hpp"

#include <lanelet2_extension/io/autoware_osm_parser.hpp>
#include <lanelet2_extension/projection/mgrs_projector.hpp>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <rclcpp/rclcpp.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>

#include <string>

Lanelet2MapLoaderNode::Lanelet2MapLoaderNode(const rclcpp::NodeOptions & options)
: Node("lanelet2_map_loader", options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  const auto lanelet2_filename = declare_parameter("lanelet2_map_path", "");
  lanelet::ErrorMessages errors{};
  lanelet::projection::MGRSProjector projector{};
  lanelet::LaneletMapPtr map = lanelet::load(lanelet2_filename, projector, &errors);

  for (const auto & error : errors) {
    RCLCPP_ERROR_STREAM(this->get_logger(), error);
  }
  if (!errors.empty()) {
    return;
  }

  // pub_lanes_ = this->create_publisher<autoware_lanelet2_msgs::msg::Lanes>(
  //   "lanelet_frontend_map", rclcpp::QoS{1}.transient_local());
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(map);
  laneletToMapMsg(this->lanes, all_lanelets);
  frontend_map_service_ = create_service<autoware_lanelet2_msgs::srv::Lanes>(
    "frontend_map_service", std::bind(&Lanelet2MapLoaderNode::MapLoaderCallback, this, _1, _2));
  // pub_lanes_->publish(lanes);

  const auto center_line_resolution = this->declare_parameter("center_line_resolution", 5.0);
  lanelet::utils::overwriteLaneletsCenterline(map, center_line_resolution, false);

  std::string format_version{}, map_version{};
  lanelet::io_handlers::AutowareOsmParser::parseVersions(
    lanelet2_filename, &format_version, &map_version);

  pub_map_bin_ = this->create_publisher<autoware_lanelet2_msgs::msg::MapBin>(
    "output/lanelet2_map", rclcpp::QoS{1}.transient_local());

  autoware_lanelet2_msgs::msg::MapBin map_bin_msg;
  map_bin_msg.header.stamp = this->now();
  map_bin_msg.header.frame_id = "map";
  map_bin_msg.format_version = format_version;
  map_bin_msg.map_version = map_version;
  lanelet::utils::conversion::toBinMsg(map, &map_bin_msg);

  RCLCPP_WARN(get_logger(), "pub map_bin_msg !!!!!!!!!!!!");
  pub_map_bin_->publish(map_bin_msg);
}

void Lanelet2MapLoaderNode::MapLoaderCallback(
  const std::shared_ptr<autoware_lanelet2_msgs::srv::Lanes::Request>,
  const std::shared_ptr<autoware_lanelet2_msgs::srv::Lanes::Response> response)
{
  response->lanes = this->lanes;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(Lanelet2MapLoaderNode)
