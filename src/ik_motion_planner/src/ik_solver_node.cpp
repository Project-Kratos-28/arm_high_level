#include <memory>
#include <vector>
#include <string>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>

class IKSolverNode : public rclcpp::Node
{
public:
  IKSolverNode()
  : Node("ik_solver_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    // ----- Parameters -----
    if (!this->has_parameter("planning_group")) {
      this->declare_parameter<std::string>("planning_group", "arm");
    }
    planning_group_ = this->get_parameter("planning_group").as_string();

    if (!this->has_parameter("base_frame")) {
      this->declare_parameter<std::string>("base_frame", "base_link");
    }
    base_frame_ = this->get_parameter("base_frame").as_string();

    if (!this->has_parameter("tip_frame")) {
      this->declare_parameter<std::string>("tip_frame", "tool0");
    }
    tip_frame_ = this->get_parameter("tip_frame").as_string();

    if (!this->has_parameter("ik_timeout")) {
      this->declare_parameter<double>("ik_timeout", 0.001);
    }
    ik_timeout_ = this->get_parameter("ik_timeout").as_double();

    if (!this->has_parameter("use_live_joint_states")) {
      this->declare_parameter<bool>("use_live_joint_states", false);
    }
    use_live_joint_states_ = this->get_parameter("use_live_joint_states").as_bool();

    joint_names_ = {
      "base_yaw_joint",
      "shoulder_joint",
      "elbow_joint",
      "wrist_pitch_joint",
      "wrist_roll_joint"
    };

    // Initialize commanded joints at home (0.0)
    current_arm_joints_.assign(joint_names_.size(), 0.0);
    last_gripper_val_ = 0.0;
    has_valid_solution_ = true;

    // ----- Publishers -----
    // Publishes 6-element array [J0, J1, J2, J3, J4, gripper] to /arm_cmd
    arm_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("arm_cmd", 10);

    // Publishes solved joint angles to /arm_joint_sync for bumpless IK -> FK state mirroring in ps5_mapper
    joint_sync_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("arm_joint_sync", 10);

    // ----- Subscriptions -----
    // Subscribes to PS5 Mapper IK commands [r, theta, z, world_pitch, roll, gripper]
    ik_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "arm_ik_cmd", 10,
      std::bind(&IKSolverNode::ikCmdCallback, this, std::placeholders::_1));

    // Live joint feedback subscription (hook for future closed-loop seeding)
    joint_feedback_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_feedback", 10,
      std::bind(&IKSolverNode::jointFeedbackCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
      "IK Solver Node started (Group: %s, Base: %s, Tip: %s, Timeout: %.3fs).",
      planning_group_.c_str(), base_frame_.c_str(), tip_frame_.c_str(), ik_timeout_);
  }

  void initialize()
  {
    // ----- MoveIt Robot Model Loader -----
    RCLCPP_INFO(this->get_logger(), "Loading robot model from robot_description...");
    robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(
      shared_from_this(), "robot_description");

    kinematic_model_ = robot_model_loader_->getModel();
    if (!kinematic_model_) {
      RCLCPP_FATAL(this->get_logger(), "Failed to load RobotModel from robot_description!");
      throw std::runtime_error("Failed to load kinematic model");
    }

    joint_model_group_ = kinematic_model_->getJointModelGroup(planning_group_);
    if (!joint_model_group_) {
      RCLCPP_FATAL(this->get_logger(), "Planning group '%s' not found in robot model!", planning_group_.c_str());
      throw std::runtime_error("Planning group not found");
    }

    kinematic_state_ = std::make_shared<moveit::core::RobotState>(kinematic_model_);
    kinematic_state_->setToDefaultValues();

    // Verify kinematics solver is loaded (TRAC-IK or KDL)
    if (joint_model_group_->getSolverInstance()) {
      RCLCPP_INFO(this->get_logger(), "Kinematics solver successfully loaded for group '%s'.",
        planning_group_.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "No custom kinematics solver loaded; using default MoveIt IK.");
    }

    const Eigen::Isometry3d & home_pose = kinematic_state_->getGlobalLinkTransform(tip_frame_);
    RCLCPP_INFO(this->get_logger(), "Home '%s' Pose (all joints=0): x=%.3f y=%.3f z=%.3f  r=%.3f  theta=%.3f rad",
      tip_frame_.c_str(),
      home_pose.translation().x(), home_pose.translation().y(), home_pose.translation().z(),
      std::hypot(home_pose.translation().x(), home_pose.translation().y()),
      std::atan2(home_pose.translation().y(), home_pose.translation().x()));

    // ---- FK Workspace Sweep using MoveIt RobotState ----
    // Compute actual bounds by evaluating key joint configurations.
    // No KDL needed — uses the same MoveIt model already loaded for TRAC-IK.
    auto sweep_state = std::make_shared<moveit::core::RobotState>(kinematic_model_);
    double r_min = 1e9, r_max = -1e9, z_min = 1e9, z_max = -1e9;

    // Joint limits (from URDF):
    // shoulder: [-1.57, 1.57], elbow: [-2.50, 2.50], wrist_pitch: [-1.57, 1.57]
    // base_yaw and wrist_roll don't change TCP position (only azimuth/orientation), so fix at 0
    std::vector<double> shoulder_vals = {-1.57, -1.0, -0.5, 0.0, 0.5, 1.0, 1.57};
    std::vector<double> elbow_vals    = {-2.50, -1.5, -0.5, 0.0, 0.5, 1.5, 2.50};
    std::vector<double> wrist_vals    = {-1.57, 0.0, 1.57};

    for (double q2 : shoulder_vals) {
      for (double q3 : elbow_vals) {
        for (double q4 : wrist_vals) {
          sweep_state->setVariablePosition("base_yaw_joint",    0.0);
          sweep_state->setVariablePosition("shoulder_joint",     q2);
          sweep_state->setVariablePosition("elbow_joint",        q3);
          sweep_state->setVariablePosition("wrist_pitch_joint",  q4);
          sweep_state->setVariablePosition("wrist_roll_joint",   0.0);
          sweep_state->update();
          const Eigen::Vector3d & p = sweep_state->getGlobalLinkTransform(tip_frame_).translation();
          double r = std::hypot(p.x(), p.y());
          r_min = std::min(r_min, r);
          r_max = std::max(r_max, r);
          z_min = std::min(z_min, p.z());
          z_max = std::max(z_max, p.z());
        }
      }
    }
    RCLCPP_INFO(this->get_logger(),
      "FK Workspace Bounds (base_yaw=0): Reach r=[%.3f, %.3f] m  |  Elevation z=[%.3f, %.3f] m",
      r_min, r_max, z_min, z_max);
    RCLCPP_INFO(this->get_logger(),
      "Azimuth theta=[-3.14, +3.14] rad (full 360 deg via base_yaw_joint)");
  }

private:
  /**
   * Euler to quaternion conversion (intrinsic Z-Y-X sequence: yaw -> pitch -> roll)
   * Matches ps5_mapper.py exactly.
   */
  void eulerToQuaternion(double yaw, double pitch, double roll,
                         double & qx, double & qy, double & qz, double & qw) const
  {
    double cy = std::cos(yaw * 0.5);
    double sy = std::sin(yaw * 0.5);
    double cp = std::cos(pitch * 0.5);
    double sp = std::sin(pitch * 0.5);
    double cr = std::cos(roll * 0.5);
    double sr = std::sin(roll * 0.5);

    qw = cr * cp * cy + sr * sp * sy;
    qx = sr * cp * cy - cr * sp * sy;
    qy = cr * sp * cy + sr * cp * sy;
    qz = cr * cp * sy - sr * sp * cy;
  }

  void ikCmdCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    // Expected format: [r, theta, z, world_pitch, roll, gripper]
    if (msg->data.size() < 6) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Received arm_ik_cmd with insufficient elements (%zu < 6). Ignoring.", msg->data.size());
      return;
    }

    double r = msg->data[0];
    double theta = msg->data[1];
    double z = msg->data[2];
    double world_pitch = msg->data[3];
    double roll = msg->data[4];
    double gripper_cmd = msg->data[5];
    last_gripper_val_ = gripper_cmd;

    // 1. Reconstruct Cartesian target coordinates
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = r * std::cos(theta);
    target_pose.position.y = r * std::sin(theta);
    target_pose.position.z = z;

    // 2. Reconstruct Auto-Leveling Orientation (matching mapper)
    double qx, qy, qz, qw;
    eulerToQuaternion(theta, world_pitch, roll, qx, qy, qz, qw);
    target_pose.orientation.x = qx;
    target_pose.orientation.y = qy;
    target_pose.orientation.z = qz;
    target_pose.orientation.w = qw;

    // 3. Seed kinematic state with current joint angles (open-loop seed tracking)
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      kinematic_state_->setVariablePosition(joint_names_[i], current_arm_joints_[i]);
    }

    // 4. Solve Inverse Kinematics using MoveIt (TRAC-IK / KDL plugin)
    bool found_ik = kinematic_state_->setFromIK(
      joint_model_group_, target_pose, tip_frame_, ik_timeout_);

    if (found_ik) {
      // Extract solved joint positions in correct hardware joint order
      for (size_t i = 0; i < joint_names_.size(); ++i) {
        current_arm_joints_[i] = kinematic_state_->getVariablePosition(joint_names_[i]);
      }
      has_valid_solution_ = true;
    } else {
      // Failure Handling per agreed specification:
      // Hold last valid joint angles to suppress discontinuous jumps; log throttled warning
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "IK solver could not find a solution for target (r=%.2f, th=%.2f, z=%.2f). Holding position.",
        r, theta, z);
    }

    // 5. Publish unified /arm_cmd [base_yaw, shoulder, elbow, wrist_pitch, wrist_roll, gripper]
    std_msgs::msg::Float64MultiArray cmd_msg;
    cmd_msg.data.reserve(6);
    for (double j_val : current_arm_joints_) {
      cmd_msg.data.push_back(j_val);
    }
    cmd_msg.data.push_back(last_gripper_val_);
    arm_cmd_pub_->publish(cmd_msg);
    joint_sync_pub_->publish(cmd_msg);
  }

  /**
   * NOTE: Hook for future live /joint_states feedback.
   * When hardware encoder feedback is active, enable use_live_joint_states to seed
   * from live physical angles for bumpless transitions.
   */
  void jointFeedbackCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (!use_live_joint_states_) {
      return;
    }

    for (size_t i = 0; i < msg->name.size(); ++i) {
      for (size_t j = 0; j < joint_names_.size(); ++j) {
        if (msg->name[i] == joint_names_[j] && i < msg->position.size()) {
          current_arm_joints_[j] = msg->position[i];
          break;
        }
      }
    }
  }

  // Member variables
  std::string planning_group_;
  std::string base_frame_;
  std::string tip_frame_;
  double ik_timeout_;
  bool use_live_joint_states_;

  std::vector<std::string> joint_names_;
  std::vector<double> current_arm_joints_;
  double last_gripper_val_;
  bool has_valid_solution_;

  std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
  moveit::core::RobotModelPtr kinematic_model_;
  const moveit::core::JointModelGroup* joint_model_group_{nullptr};
  moveit::core::RobotStatePtr kinematic_state_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr arm_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_sync_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ik_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_feedback_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IKSolverNode>();
  node->initialize();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
