import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)

# detect all 36h11 tags
cfg_36h11 = {
    "image_transport": "raw",
    "family": "Standard52h13",
    "size": 0.040,
    "max_hamming": 0,
    "z_up": True
}

 # Simulation arguments
use_sim_time = LaunchConfiguration("use_sim_time")

def generate_launch_description():
    composable_node = ComposableNode(
        name='apriltag',
        package='apriltag_ros', plugin='AprilTagNode',
        remappings=[("/apriltag/image", "/camera/image"), ("/apriltag/camera_info", "/camera/camera_info")],
        parameters=[cfg_36h11])
    container = ComposableNodeContainer(
        name='tag_container',
        namespace='apriltag',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[composable_node],
        output='screen',
        parameters=[{"use_sim_time": use_sim_time}],
    )

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time.",
        )
    )

    return launch.LaunchDescription(declared_arguments +[container])
