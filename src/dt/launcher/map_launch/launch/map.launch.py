# Copyright 2021 Tier IV, Inc. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    lanelet2_map_loader = ComposableNode(
        package="map_loader",
        plugin="Lanelet2MapLoaderNode",
        name="lanelet2_map_loader",
        remappings=[("output/lanelet2_map", "vector_map")],
        parameters=[
            {
                "center_line_resolution": 5.0,
                "lanelet2_map_path": LaunchConfiguration("map_file"),
            }
        ],
        # extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    lanelet2_map_visualization = ComposableNode(
        package="map_loader",
        plugin="Lanelet2MapVisualizationNode",
        name="lanelet2_map_visualization",
        remappings=[
            ("input/lanelet2_map", "vector_map"),
            ("output/lanelet2_map_marker", "vector_map_marker"),
        ],
        # extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    costmap_generator = ComposableNode(
        package="costmap_generator",
        plugin="CostmapGenerator",
        name="costmap_generator",
        remappings=[
            ("~/input/vector_map", "/map/vector_map"),
            ("~/output/occupancy_grid", "costmap_generator/occupancy_grid"),
        ],
        parameters=[  
            {
                "costmap_frame": "map",
                "vehicle_frame": "base_link",
                "map_frame": "map",
                "update_rate": 2.0,
                "use_wayarea": False,
                "use_objects": False,
                "use_points": False,
                "grid_min_value": 0.0,
                "grid_max_value": 1.0,
                "grid_resolution": 0.1,
                "grid_length_x": 50.0,
                "grid_length_y": 50.0,
                "grid_position_x": 0.0,
                "grid_position_y": 0.0,
                "maximum_lidar_height_thres": 0.3,
                "minimum_lidar_height_thres": -2.2,
                "expand_polygon_size": 1.0,
                "size_of_expansion_kernel": 9,
            },
        ],
        # extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    container = ComposableNodeContainer(
        name="map_container",
        namespace="",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=[
            lanelet2_map_loader,
            lanelet2_map_visualization,
            costmap_generator,
        ],
        output="screen",
    )

    def add_launch_arg(name: str, default_value=None, description=None):
        return DeclareLaunchArgument(name, default_value=default_value, description=description)

    return launch.LaunchDescription(
        [
            add_launch_arg(
                name = "map_file",
                description = "path to lanelet2 map file",
            ),
            add_launch_arg(
                "use_intra_process", "false", "use ROS2 component container communication"
            ),
            add_launch_arg("use_multithread", "false", "use multithread"),
            SetLaunchConfiguration(
                "container_executable",
                "component_container",
                condition=UnlessCondition(LaunchConfiguration("use_multithread")),
            ),
            SetLaunchConfiguration(
                "container_executable",
                "component_container_mt",
                condition=IfCondition(LaunchConfiguration("use_multithread")),
            ),
            GroupAction(
                [
                    PushRosNamespace("map"),
                    container,
                ]
            ),
        ]
    )
