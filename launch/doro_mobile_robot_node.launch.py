#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node

def generate_launch_description():
  doro_mobile_state_parameter = LaunchConfiguration(
    'doro_mobile_state_parameter',
    default=os.path.join(
      get_package_share_directory('doro_mobile'),
      'param',
      'doro_mobile_state.yaml'))
  return LaunchDescription([
    DeclareLaunchArgument(
      'doro_mobile_state_parameter',
      default_value=doro_mobile_state_parameter
    ),
    Node(
      package='doro_mobile',
      executable='doro_mobile_robot_node',
      name='doro_mobile_robot_node',
      output='screen',
      emulate_tty=True,
      parameters=[doro_mobile_state_parameter],
    )
  ])