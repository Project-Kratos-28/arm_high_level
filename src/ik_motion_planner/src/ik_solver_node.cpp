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

static inline double wrapAngle(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

static inline double analyticalWristPitch(double world_pitch, double q1, double q2)
{
  return -world_pitch - (q1 + q2 - 2.00719 + 0.4363323);
}

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

    if (!this->has_parameter("wrist_planning_group")) {
      this->declare_parameter<std::string>("wrist_planning_group", "arm_wrist");
    }
    wrist_planning_group_ = this->get_parameter("wrist_planning_group").as_string();

    if (!this->has_parameter("wrist_tip_frame")) {
      this->declare_parameter<std::string>("wrist_tip_frame", "wrist_center");
    }
    wrist_tip_frame_ = this->get_parameter("wrist_tip_frame").as_string();

    joint_names_ = {
      "base_yaw_joint",
      "shoulder_joint",
      "elbow_joint",
      "wrist_pitch_joint",
      "wrist_roll_joint"
    };

    // Seed warm-start at home posture so first IK query converges near home
    current_arm_joints_ = {0.0, 0.0, 2.0072, 0.0, 0.0};
    last_gripper_val_ = 0.0;

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

    // Continuous FK state synchronization from ps5_mapper (for bumpless FK -> IK warm seeding)
    fk_sync_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "arm_fk_sync", 10,
      std::bind(&IKSolverNode::fkSyncCallback, this, std::placeholders::_1));

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

    // Load decoupled wrist positioning group (e.g. arm_wrist targeting wrist_center)
    joint_model_group_wrist_ = kinematic_model_->getJointModelGroup(wrist_planning_group_);
    if (joint_model_group_wrist_) {
      RCLCPP_INFO(this->get_logger(), "Decoupled wrist positioning group '%s' (tip: '%s') loaded.",
        wrist_planning_group_.c_str(), wrist_tip_frame_.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "Group '%s' not found; falling back to group '%s'.",
        wrist_planning_group_.c_str(), planning_group_.c_str());
    }

    const Eigen::Isometry3d & home_pose = kinematic_state_->getGlobalLinkTransform(wrist_tip_frame_);
    RCLCPP_INFO(this->get_logger(), "Home '%s' Pose (all joints=0): x=%.3f y=%.3f z=%.3f  r=%.3f  theta=%.3f rad",
      wrist_tip_frame_.c_str(),
      home_pose.translation().x(), home_pose.translation().y(), home_pose.translation().z(),
      std::hypot(home_pose.translation().x(), home_pose.translation().y()),
      std::atan2(home_pose.translation().y(), home_pose.translation().x()));

    // ---- FK Workspace Sweep using MoveIt RobotState for wrist_center ----
    auto sweep_state = std::make_shared<moveit::core::RobotState>(kinematic_model_);
    double r_min = 1e9, r_max = -1e9, z_min = 1e9, z_max = -1e9;

    // Joint limits (from URDF): shoulder: [-1.57, 1.57], elbow: [0.0, 2.50]
    std::vector<double> shoulder_vals = {-1.57, -1.0, -0.5, 0.0, 0.5, 1.0, 1.57};
    std::vector<double> elbow_vals    = {0.0, 0.5, 1.0, 1.5, 2.0, 2.50};

    for (double q2 : shoulder_vals) {
      for (double q3 : elbow_vals) {
        sweep_state->setVariablePosition("base_yaw_joint",    0.0);
        sweep_state->setVariablePosition("shoulder_joint",     q2);
        sweep_state->setVariablePosition("elbow_joint",        q3);
        sweep_state->update();
        const Eigen::Vector3d & p = sweep_state->getGlobalLinkTransform(wrist_tip_frame_).translation();
        double r = std::hypot(p.x(), p.y());
        r_min = std::min(r_min, r);
        r_max = std::max(r_max, r);
        z_min = std::min(z_min, p.z());
        z_max = std::max(z_max, p.z());
      }
    }
    RCLCPP_INFO(this->get_logger(),
      "FK Wrist Workspace Bounds (base_yaw=0): Reach r=[%.3f, %.3f] m  |  Elevation z=[%.3f, %.3f] m",
      r_min, r_max, z_min, z_max);
    RCLCPP_INFO(this->get_logger(),
      "Azimuth theta=[-3.14, +3.14] rad (full 360 deg via base_yaw_joint)");

    // Set RobotState to home posture for accurate startup FK state
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      kinematic_state_->setVariablePosition(joint_names_[i], current_arm_joints_[i]);
    }
    kinematic_state_->update();

    // Publish initial home posture — synchronizes /arm_cmd and ps5_mapper FK state
    std_msgs::msg::Float64MultiArray init_cmd;
    init_cmd.data = current_arm_joints_;
    init_cmd.data.push_back(last_gripper_val_);
    arm_cmd_pub_->publish(init_cmd);
    joint_sync_pub_->publish(init_cmd);
    RCLCPP_INFO(this->get_logger(), "Published initial home posture: elbow=+2.0072 rad.");
  }

private:
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

    // Target wrist_center Cartesian position in base_link frame
    const double X0 = 0.043511;
    const double Y0 = -0.012448;
    const double dx_arm = -0.023;

    double x_w_base = X0 + r * std::sin(theta) + dx_arm * std::cos(theta);
    double y_w_base = Y0 - r * std::cos(theta) + dx_arm * std::sin(theta);
    double z_w_base = z;

    // Update kinematic_state_ with current arm joints so p_cur reflects current physical posture
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      kinematic_state_->setVariablePosition(joint_names_[i], current_arm_joints_[i]);
    }
    kinematic_state_->update();
    Eigen::Isometry3d T_root_base = kinematic_state_->getFrameTransform(base_frame_);
    Eigen::Vector3d pw_in_root = T_root_base * Eigen::Vector3d(x_w_base, y_w_base, z_w_base);

    // If current wrist position is already within 0.5 mm of the target (e.g. at rest or on mode switch),
    // retain current positioning joints to prevent solver chatter / numerical SQP tolerance drift.
    const Eigen::Vector3d & p_cur = kinematic_state_->getGlobalLinkTransform(wrist_tip_frame_).translation();
    double target_dist = (p_cur - pw_in_root).norm();

    if (target_dist < 0.0005) {
      double cand_q3_analytical = analyticalWristPitch(world_pitch, current_arm_joints_[1], current_arm_joints_[2]);
      current_arm_joints_[3] = std::max(-1.57, std::min(1.57, cand_q3_analytical));
      current_arm_joints_[4] = std::max(-3.14, std::min(3.14, roll));
    } else {
      geometry_msgs::msg::Pose wrist_target_pose;
      wrist_target_pose.position.x = pw_in_root.x();
      wrist_target_pose.position.y = pw_in_root.y();
      wrist_target_pose.position.z = pw_in_root.z();
      wrist_target_pose.orientation.w = 1.0; // position_only_ik

      // Seed TRAC-IK with commanded base yaw theta, and current arm joints
      kinematic_state_->setVariablePosition(joint_names_[0], theta);
      kinematic_state_->setVariablePosition(joint_names_[1], current_arm_joints_[1]);
      kinematic_state_->setVariablePosition(joint_names_[2], current_arm_joints_[2]);

      const auto* jmg_to_solve = joint_model_group_wrist_ ? joint_model_group_wrist_ : joint_model_group_;
      const std::string& tip_to_solve = joint_model_group_wrist_ ? wrist_tip_frame_ : tip_frame_;

      bool ik_ok = kinematic_state_->setFromIK(
        jmg_to_solve, wrist_target_pose, tip_to_solve, ik_timeout_);

      if (ik_ok) {
        double cand_q0 = kinematic_state_->getVariablePosition(joint_names_[0]);
        double cand_q1 = kinematic_state_->getVariablePosition(joint_names_[1]);
        double cand_q2 = kinematic_state_->getVariablePosition(joint_names_[2]);

        // Base Yaw Azimuth Guard:
        // Expected base yaw is theta. Solutions that flip 180 degrees backward (|yaw_diff| > 1.0 rad)
        // or jump suddenly are rejected.
        double q0_exp = wrapAngle(theta);
        double yaw_diff = wrapAngle(cand_q0 - q0_exp);
        double yaw_jump = wrapAngle(cand_q0 - current_arm_joints_[0]);

        if (std::abs(yaw_diff) > 1.0 || std::abs(yaw_jump) > 1.0) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "[YAW GUARD] Rejected base yaw flip (cand=%.2f, exp=%.2f, diff=%.2f, jump=%.2f). Holding position.",
            cand_q0, q0_exp, yaw_diff, yaw_jump);
        } else {
          // Analytical wrist pitch strictly enforcing world_pitch relative to horizon:
          double cand_q3_analytical = analyticalWristPitch(world_pitch, cand_q1, cand_q2);
          double cand_q3 = std::max(-1.57, std::min(1.57, cand_q3_analytical));
          double cand_q4 = std::max(-3.14, std::min(3.14, roll));

          current_arm_joints_[0] = cand_q0;
          current_arm_joints_[1] = cand_q1;
          current_arm_joints_[2] = cand_q2;
          current_arm_joints_[3] = cand_q3;
          current_arm_joints_[4] = cand_q4;
        }
      } else {
        // When TRAC-IK fails (e.g. pushed against minimum reach or elevation boundary),
        // the base yaw joint is completely decoupled from the sagittal reach and can still rotate!
        // Update base_yaw to theta within limits, and recompute analytical wrist orientation.
        double cand_q0 = std::max(-3.14, std::min(3.14, theta));
        double yaw_jump = wrapAngle(cand_q0 - current_arm_joints_[0]);

        if (std::abs(yaw_jump) <= 1.0) {
          current_arm_joints_[0] = cand_q0;
          double cand_q3_analytical = analyticalWristPitch(world_pitch, current_arm_joints_[1], current_arm_joints_[2]);
          current_arm_joints_[3] = std::max(-1.57, std::min(1.57, cand_q3_analytical));
          current_arm_joints_[4] = std::max(-3.14, std::min(3.14, roll));
        } else {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "IK solver could not find a solution for wrist target (r=%.2f, th=%.2f, z=%.2f). Holding position.",
            r, theta, z);
        }
      }
    }

    // Publish unified /arm_cmd [base_yaw, shoulder, elbow, wrist_pitch, wrist_roll, gripper]
    std_msgs::msg::Float64MultiArray cmd_msg;
    cmd_msg.data = current_arm_joints_;
    cmd_msg.data.push_back(last_gripper_val_);
    arm_cmd_pub_->publish(cmd_msg);
    joint_sync_pub_->publish(cmd_msg);
  }

  void fkSyncCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() >= 5) {
      for (size_t i = 0; i < 5; ++i) {
        current_arm_joints_[i] = msg->data[i];
      }
    }
    if (msg->data.size() >= 6) {
      last_gripper_val_ = msg->data[5];
    }
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
  std::string wrist_planning_group_;
  std::string wrist_tip_frame_;
  double ik_timeout_;
  bool use_live_joint_states_;

  std::vector<std::string> joint_names_;
  std::vector<double> current_arm_joints_;
  double last_gripper_val_;

  std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
  moveit::core::RobotModelPtr kinematic_model_;
  const moveit::core::JointModelGroup* joint_model_group_{nullptr};
  const moveit::core::JointModelGroup* joint_model_group_wrist_{nullptr};
  moveit::core::RobotStatePtr kinematic_state_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr arm_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_sync_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ik_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr fk_sync_sub_;
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
