import os
import shutil
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_terminal_prefix():
    """Detects available terminal emulator to spawn popup window without cluttering launch logs."""
    if shutil.which("gnome-terminal"):
        return "gnome-terminal --geometry=120x30 --title='Arm Velocity Telemetry Dashboard' --"
    elif shutil.which("terminator"):
        return "terminator -T 'Arm Velocity Telemetry Dashboard' --geometry=960x500 -x"
    elif shutil.which("xterm"):
        return "xterm -geometry 120x30 -title 'Arm Velocity Telemetry Dashboard' -e"
    elif shutil.which("alacritty"):
        return "alacritty --title 'Arm Velocity Telemetry Dashboard' -e"
    elif shutil.which("konsole"):
        return "konsole --title 'Arm Velocity Telemetry Dashboard' -e"
    elif shutil.which("xfce4-terminal"):
        return "xfce4-terminal --title='Arm Velocity Telemetry Dashboard' -x"
    return ""


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_dashboard = LaunchConfiguration("dashboard")
    launch_lights = LaunchConfiguration("lights")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true",
    )
    declare_dashboard = DeclareLaunchArgument(
        "dashboard",
        default_value="false",
        description="Launch live arm velocity telemetry dashboard in a dedicated popup window",
    )
    declare_lights = DeclareLaunchArgument(
        "lights",
        default_value="false",
        description="Launch PS5 controller lighting and mode telemetry node",
    )

    # Node: joy_node (Reads PS5 DualSense controller)
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Node: ps5_mapper (Direct FK velocity mapper -> /arm_cmd [-255, 255])
    mapper_node = Node(
        package="mapper",
        executable="ps5_mapper",
        name="ps5_mapper",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "invert_j0": True,
                "invert_j4": True,
            }
        ],
    )

    # Node: ps5_lights_node (Optional controller lighting)
    lights_node = Node(
        package="mapper",
        executable="ps5_lights_node",
        name="ps5_lights_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(launch_lights),
    )

    # Node: arm_dashboard (Live PWM velocity telemetry dashboard)
    dashboard_prefix = get_terminal_prefix()
    dashboard_node = Node(
        package="arm_controller",
        executable="arm_dashboard.py",
        name="arm_dashboard",
        output="screen",
        prefix=dashboard_prefix if dashboard_prefix else None,
        arguments=["--parent-pid", str(os.getpid())],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(launch_dashboard),
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_dashboard,
            declare_lights,
            joy_node,
            mapper_node,
            lights_node,
            dashboard_node,
        ]
    )
