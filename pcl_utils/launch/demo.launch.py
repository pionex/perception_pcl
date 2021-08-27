# Copyright 2019 Open Source Robotics Foundation, Inc.
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

"""Launch a talker and a listener in a component container."""

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='PCLDemo',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='pcl_utils',
                    plugin='pcl_utils::NormalsVisualizer',
                    name='visualizer',
                    remappings=[
                        ('cloud_input', 'cloud_output'),
                        ('normals_cloud_input', 'normals_cloud_output')
                    ]),
                ComposableNode(
                    package='pcl_ros',
                    plugin='pcl_ros::PCDReader',
                    name='pcdreader',
                    parameters = [
                        {"file_name": "/home/perception/VagrantProjects/Clouds/IC/ContainerUnload_19-03-20_19-29-10_Gripper.pcd",
                         "tf_frame": "map"
                         }
                    ]),
                ComposableNode(
                    package='pcl_utils',
                    plugin='pcl_utils::NormalsGenerator',
                    name='normalsgenerator',
#                    parameters = [
#                      {
#                        "file_name": "/home/perception/VagrantProjects/Clouds/IC/ContainerUnload_19-03-20_19-29-10_Gripper.pcd",
#                        "tf_frame": "map",
#						 "use_indices":  False
#                      }
#                    ],
                    remappings=[
                          ('raw_cloud', 'output')
                   ]
                  )
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])
