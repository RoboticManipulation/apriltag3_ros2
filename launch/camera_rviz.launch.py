import launch
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os
import math
from ament_index_python.packages import get_package_share_directory

 # Simulation arguments
use_sim_time = LaunchConfiguration("use_sim_time")

def launch_setup(context, *args, **kwargs):

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("realsense2_camera"),
                    "launch",
                    "l515_launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    rviz_config_dir = os.path.join(get_package_share_directory('apriltag_ros'), 'rviz', 'real_robot_evaluation.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output = 'screen',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
    )


    static_tf_broadcaster = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output = 'screen',
        # Uncallibrated
        #arguments=["-0.232", "0.415", "0.7", "0", "1.57", "0", "world", "camera_link"],
        # Callibration result
        # xyz="-0.223988 0.566215 0.707892" rpy="3.13878 -0.00747704 0.0231476"
        arguments=["-0.223988", "0.566215", "0.707892", "3.13878", str(-0.00747704+(math.pi)), "0.0231476", "world", "camera_color_optical_frame"],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    nodes_to_start = [realsense_launch, rviz_node, static_tf_broadcaster]

    return nodes_to_start

def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
