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

    timeout_arg = DeclareLaunchArgument(
        "timeout",
        default_value="0.005",
        description="Max IK solve timeout per query in seconds",
    )
    output_csv_arg = DeclareLaunchArgument(
        "output_csv",
        default_value="/home/kratos/workspace/fk_ik_reachability_report.csv",
        description="Destination path for output reachability CSV",
    )

    timeout = LaunchConfiguration("timeout")
    output_csv = LaunchConfiguration("output_csv")

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

    # Node: fk_ik_reachability_checker
    checker_node = Node(
        package="ik_motion_planner",
        executable="fk_ik_reachability_checker",
        name="fk_ik_reachability_checker",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            {
                "planning_group": "arm",
                "wrist_planning_group": "arm_wrist",
                "tip_frame": "tool0",
                "wrist_tip_frame": "wrist_center",
                "base_frame": "base_link",
                "ik_timeout": timeout,
                "output_csv": output_csv,
            },
        ],
    )

    return LaunchDescription([
        timeout_arg,
        output_csv_arg,
        checker_node,
    ])
