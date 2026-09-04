import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("rviz")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true",
    )
    declare_rviz = DeclareLaunchArgument(
        "rviz",
        default_value="false",
        description="Launch RViz visualization",
    )

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

    # Node: robot_state_publisher
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # Node: ps5_mapper (teleoperation)
    mapper_node = Node(
        package="mapper",
        executable="ps5_mapper",
        name="ps5_mapper",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Node: ik_solver_node (TRAC-IK MoveIt 2 Solver)
    ik_solver_node = Node(
        package="ik_motion_planner",
        executable="ik_solver_node",
        name="ik_solver_node",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            {
                "planning_group": "arm",
                "base_frame": "base_link",
                "tip_frame": "tool0",
                "ik_timeout": 0.005,
                "use_live_joint_states": False,
                "use_sim_time": use_sim_time,
            },
        ],
    )

    # Node: RViz2
    rviz_config_file = os.path.join(pkg_arm_controller, "config", "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(launch_rviz),
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_rviz,
            rsp_node,
            mapper_node,
            ik_solver_node,
            rviz_node,
        ]
    )
