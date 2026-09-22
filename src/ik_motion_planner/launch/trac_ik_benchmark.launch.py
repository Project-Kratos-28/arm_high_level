import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:
        return None


def generate_launch_description():
    pkg_arm_controller = get_package_share_directory("arm_controller")

    # Launch Arguments
    samples_arg = DeclareLaunchArgument(
        "samples",
        default_value="10000",
        description="Number of IK solve queries to benchmark per test",
    )
    timeout_arg = DeclareLaunchArgument(
        "timeout",
        default_value="0.005",
        description="Max IK solve timeout per query in seconds (for single test mode)",
    )
    sweep_arg = DeclareLaunchArgument(
        "sweep",
        default_value="false",
        description="Enable parameter sweep mode across multiple timeouts",
    )
    sweep_min_arg = DeclareLaunchArgument(
        "sweep_min_ms",
        default_value="0.05",
        description="Minimum timeout in milliseconds for sweep",
    )
    sweep_max_arg = DeclareLaunchArgument(
        "sweep_max_ms",
        default_value="1.50",
        description="Maximum timeout in milliseconds for sweep",
    )
    sweep_step_arg = DeclareLaunchArgument(
        "sweep_step_ms",
        default_value="0.05",
        description="Step size in milliseconds for sweep",
    )
    workspace_map_arg = DeclareLaunchArgument(
        "workspace_map",
        default_value="false",
        description="Enable workspace reachability and failure map mode",
    )
    map_r_steps_arg = DeclareLaunchArgument(
        "map_r_steps",
        default_value="25",
        description="Number of radial reach (r) steps",
    )
    map_z_steps_arg = DeclareLaunchArgument(
        "map_z_steps",
        default_value="30",
        description="Number of elevation (z) steps",
    )
    map_pitch_steps_arg = DeclareLaunchArgument(
        "map_pitch_steps",
        default_value="7",
        description="Number of world_pitch slices",
    )
    map_attempts_arg = DeclareLaunchArgument(
        "map_attempts",
        default_value="5",
        description="Number of IK solve attempts per cell from random seeds",
    )

    samples = LaunchConfiguration("samples")
    timeout = LaunchConfiguration("timeout")
    sweep = LaunchConfiguration("sweep")
    sweep_min_ms = LaunchConfiguration("sweep_min_ms")
    sweep_max_ms = LaunchConfiguration("sweep_max_ms")
    sweep_step_ms = LaunchConfiguration("sweep_step_ms")
    workspace_map = LaunchConfiguration("workspace_map")
    map_r_steps = LaunchConfiguration("map_r_steps")
    map_z_steps = LaunchConfiguration("map_z_steps")
    map_pitch_steps = LaunchConfiguration("map_pitch_steps")
    map_attempts = LaunchConfiguration("map_attempts")

    # 1. Robot Description (URDF from xacro)
    xacro_file = os.path.join(pkg_arm_controller, "urdf", "arm_cad.urdf.xacro")
    robot_description_content = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", xacro_file]),
        value_type=str,
    )
    robot_description = {"robot_description": robot_description_content}

    # 2. Semantic Robot Description (SRDF)
    robot_description_semantic_content = load_file("arm_controller", "config/arm.srdf")
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }

    # 3. Kinematics YAML (TRAC-IK config)
    kinematics_yaml = load_yaml("arm_controller", "config/kinematics.yaml")
    robot_description_kinematics = {
        "robot_description_kinematics": kinematics_yaml
    }

    # 4. Joint Limits YAML
    joint_limits_yaml = load_yaml("arm_controller", "config/joint_limits.yaml")
    robot_description_planning = {
        "robot_description_planning": joint_limits_yaml
    }

    # Node: trac_ik_benchmark
    benchmark_node = Node(
        package="ik_motion_planner",
        executable="trac_ik_benchmark",
        name="trac_ik_benchmark",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            {
                "planning_group": "arm",
                "tip_frame": "tool0",
                "ik_timeout": timeout,
                "samples": samples,
                "sweep": sweep,
                "sweep_min_ms": sweep_min_ms,
                "sweep_max_ms": sweep_max_ms,
                "sweep_step_ms": sweep_step_ms,
                "workspace_map": workspace_map,
                "map_r_steps": map_r_steps,
                "map_z_steps": map_z_steps,
                "map_pitch_steps": map_pitch_steps,
                "map_attempts": map_attempts,
            },
        ],
    )

    return LaunchDescription(
        [
            samples_arg,
            timeout_arg,
            sweep_arg,
            sweep_min_arg,
            sweep_max_arg,
            sweep_step_arg,
            workspace_map_arg,
            map_r_steps_arg,
            map_z_steps_arg,
            map_pitch_steps_arg,
            map_attempts_arg,
            benchmark_node,
        ]
    )
