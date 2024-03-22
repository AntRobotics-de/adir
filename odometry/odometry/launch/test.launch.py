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


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer, Node
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
                package="adir_ros_can",
                plugin="can::RosCan",
                name="ros_commands",
                parameters=[
                    {
                        "can_interface_name": "can0",
                    }
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="adir_can_commands",
                plugin="can_commands::VelocityCommands",
                name="velocity_commands",
                parameters=[
                    {
                        "gear_ratio": 58.0,
                        "track_width": 1.2,  # meter
                        "wheel_diameter": 0.35,  # meter
                        "motor_controller_max_rpm": 2500,
                        "max_linear_speed":  5.1, #5.1,  # m/s
                        "max_angular_speed": 3.0,  # rad/s
                    }
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="adir_can_commands",
                plugin="can_commands::EncoderCommands",
                name="encoder_commands",
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="adir_can_commands",
                plugin="can_commands::StatusCommands",
                name="status_commands",
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="adir_can_commands",
                plugin="can_commands::PowerMonitorCommands",
                name="power_monitor_commands",
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="adir_can_commands",
                plugin="can_commands::EncoderRequestCommands",
                name="encoder_request_commands",
                parameters=[{"frequency": 10.0}],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="adir_can_commands",
                plugin="can_commands::StatusRequestCommands",
                name="status_request_commands",
                parameters=[{"frequency": 10.0}],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="adir_can_commands",
                plugin="can_commands::PowerMonitorRequestCommands",
                name="power_monitor_request_commands",
                parameters=[{"frequency": 10.0}],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
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
        output="screen",
    )

    return LaunchDescription([container])
