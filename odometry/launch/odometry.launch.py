from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.event_handlers import OnProcessStart

gear_ratio = 58.0
track_width = 0.6  # in meters
wheel_radius = 0.174  # in meters
max_linear_speed = 5.1  # in meters/second
max_angular_speed = 3.0  # in radians/second
radius_compensation = -0.002
# please refer to: https://ncbi.nlm.nih.gov/pmc/articles/PMC4481911/
track_width_compensation = 0.92

def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
        name="adir_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="antrobotics_ros_can",
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
                package="antrobotics_can_commands",
                plugin="can_commands::VelocityCommands",
                name="velocity_commands",
                parameters=[
                    {
                        "gear_ratio": gear_ratio,
                        "track_width": track_width,  # meter
                        "wheel_diameter": wheel_radius * 2,  # meter
                        "motor_controller_max_rpm": 2500,
                        "max_linear_speed":  max_linear_speed, # m/s
                        "max_angular_speed": max_angular_speed,  # rad/s
                    }
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="antrobotics_can_commands",
                plugin="can_commands::EncoderCommands",
                name="encoder_commands",
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="antrobotics_can_commands",
                plugin="can_commands::StatusCommands",
                name="status_commands",
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="antrobotics_can_commands",
                plugin="can_commands::PowerMonitorCommands",
                name="power_monitor_commands",
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="antrobotics_can_commands",
                plugin="can_commands::ResetEncoderCommands",
                name="reset_encoder_commands",
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="antrobotics_can_commands",
                plugin="can_commands::EncoderRequestCommands",
                name="encoder_request_commands",
                parameters=[{"frequency": 20.0}],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="antrobotics_can_commands",
                plugin="can_commands::StatusRequestCommands",
                name="status_request_commands",
                parameters=[{"frequency": 2.0}],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="antrobotics_can_commands",
                plugin="can_commands::PowerMonitorRequestCommands",
                name="power_monitor_request_commands",
                parameters=[{"frequency": 2.0}],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
    )

    container_odom = ComposableNodeContainer(
        name="adir_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="odometry",
                plugin="OdometryROS",
                name="odometry_node",
                parameters=[{
                    "gear_ratio": gear_ratio,
                    "encoder_ppr": 1024,
                    "wheel_radius": wheel_radius + radius_compensation,  # in meters
                    "wheel_separation": track_width + track_width_compensation,  # in meters
                    "velocity_rolling_window_size": 10,
                    "wheel_per_side": 2,
                    "publish_rate": 30.0,
                    "publish_odom_tf": True,
                    "odom_frame_id": "/adir/odom",
                }],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ]
    )

    delayed_container_odom = RegisterEventHandler(
        OnProcessStart(
            target_action=container,
            on_start=[
                TimerAction(period=1.0, actions=[container_odom])
            ]
        ))

    return LaunchDescription([container, delayed_container_odom])
