from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """Generate launch description with multiple components."""

    container = ComposableNodeContainer(
        name="commands_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="odometry",
                plugin="Odometry",
                name="odometry_node",
                parameters=[{
                    "wheel_circumference": 1.158, # in meters
                    "compensation_factor": 0.035,
                    "gear_ratio": 58.0,
                    "encoder_ppr": 1024,
                    "track_width": 0.6,
                    }],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
    )

    return LaunchDescription([container])
