/**
 * rth_node.cpp  —  Return-to-Home Action Server
 *
 * Provides the /return_to_home action server (arm_interfaces/ReturnToHome).
 * On goal acceptance:
 *   1. Reads current arm joint state from /arm_joint_sync.
 *   2. Validates interpolated waypoints against URDF joint bounds AND MoveIt
 *      self-collision checking (via PlanningScene + FCL, respecting SRDF
 *      disable_collisions pairs) before any motion begins.
 *   3. Streams a synchronized smooth-step trajectory at 50 Hz to /arm_cmd
 *      and /arm_joint_sync until home is reached or the goal is cancelled.
 * Gripper is held at its current value throughout.
 */

#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/collision_detection/collision_common.hpp>

#include <arm_interfaces/action/return_to_home.hpp>

using RTH = arm_interfaces::action::ReturnToHome;
using GoalHandleRTH = rclcpp_action::ServerGoalHandle<RTH>;

static const std::vector<std::string> JOINT_NAMES = {
  "base_yaw_joint", "shoulder_joint", "elbow_joint",
  "wrist_pitch_joint", "wrist_roll_joint"
};
static const std::vector<double> HOME_JOINTS = {0.0, 0.0, 2.0072, 0.0, 0.0};

class RTHNode : public rclcpp::Node
{
public:
  RTHNode()
  : Node("rth_node",
         rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    // ── Parameters ──────────────────────────────────────────────────
    if (!this->has_parameter("max_joint_speed")) {
      this->declare_parameter<double>("max_joint_speed", 0.15);
    }
    max_joint_speed_ = this->get_parameter("max_joint_speed").as_double();

    if (!this->has_parameter("control_rate")) {
      this->declare_parameter<double>("control_rate", 50.0);
    }
    control_rate_ = this->get_parameter("control_rate").as_double();

    if (!this->has_parameter("collision_check_points")) {
      this->declare_parameter<int>("collision_check_points", 10);
    }
    collision_check_points_ = this->get_parameter("collision_check_points").as_int();

    // ── State ────────────────────────────────────────────────────────
    current_joints_.assign(5, 0.0);
    current_joints_[2] = 2.0072;  // warm-seeded at home
    last_gripper_ = 0.0;
    goal_active_ = false;

    // ── Publishers ───────────────────────────────────────────────────
    arm_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("arm_cmd", 10);
    joint_sync_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("arm_joint_sync", 10);

    // ── Subscriptions ────────────────────────────────────────────────
    // Track current arm joint positions so trajectory starts from the real pose
    joint_sync_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "arm_joint_sync", 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 5) {
          std::lock_guard<std::mutex> lock(joints_mutex_);
          for (size_t i = 0; i < 5; ++i) {
            current_joints_[i] = msg->data[i];
          }
        }
        if (msg->data.size() >= 6) {
          last_gripper_ = msg->data[5];
        }
      });

    // ── Action Server ────────────────────────────────────────────────
    action_server_ = rclcpp_action::create_server<RTH>(
      this, "return_to_home",
      std::bind(&RTHNode::handle_goal,     this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&RTHNode::handle_cancel,   this, std::placeholders::_1),
      std::bind(&RTHNode::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
      "RTH Node started. Max speed=%.3f rad/s, Rate=%.1f Hz.",
      max_joint_speed_, control_rate_);
  }

  // ── Called after construction (requires shared_from_this) ──────────
  void initialize()
  {
    RCLCPP_INFO(this->get_logger(),
      "Loading robot model for joint-bounds and self-collision validation...");
    robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(
      shared_from_this(), "robot_description");

    kinematic_model_ = robot_model_loader_->getModel();
    if (!kinematic_model_) {
      RCLCPP_FATAL(this->get_logger(),
        "Failed to load RobotModel. RTH safety validation disabled.");
      return;
    }

    kinematic_state_ = std::make_shared<moveit::core::RobotState>(kinematic_model_);
    kinematic_state_->setToDefaultValues();

    // PlanningScene enables full MoveIt self-collision detection using FCL
    // geometry and the SRDF disable_collisions pairs (so adjacent links that
    // are always in contact do not generate false positives).
    planning_scene_ = std::make_shared<planning_scene::PlanningScene>(kinematic_model_);

    RCLCPP_INFO(this->get_logger(),
      "Robot model loaded. Joint-bounds + self-collision validation active.");
  }

private:
  // ── Goal Handler ──────────────────────────────────────────────────
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const RTH::Goal>)
  {
    if (goal_active_) {
      RCLCPP_WARN(this->get_logger(), "RTH goal rejected: another goal is already executing.");
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(this->get_logger(), "RTH goal accepted.");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleRTH>)
  {
    RCLCPP_INFO(this->get_logger(), "RTH cancellation requested.");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleRTH> goal_handle)
  {
    // Execute in a detached thread so handle_accepted returns immediately
    std::thread{std::bind(&RTHNode::execute, this, goal_handle)}.detach();
  }

  // ── Core Trajectory Execution ─────────────────────────────────────
  void execute(const std::shared_ptr<GoalHandleRTH> goal_handle)
  {
    goal_active_ = true;
    const double speed_scaling = std::max(0.01, std::min(1.0,
      goal_handle->get_goal()->speed_scaling > 0.0
        ? goal_handle->get_goal()->speed_scaling : 1.0));

    // 1. Capture start state (thread-safe)
    std::vector<double> start(5), delta(5);
    double gripper_val;
    {
      std::lock_guard<std::mutex> lock(joints_mutex_);
      start       = current_joints_;
      gripper_val = last_gripper_;
    }

    // 2. Compute per-joint deltas
    double max_abs_delta = 0.0;
    for (size_t i = 0; i < 5; ++i) {
      delta[i] = HOME_JOINTS[i] - start[i];
      max_abs_delta = std::max(max_abs_delta, std::abs(delta[i]));
    }

    // 3. Check if already at home (within 1 mrad)
    if (max_abs_delta < 0.001) {
      RCLCPP_INFO(this->get_logger(), "Arm already at home. RTH complete immediately.");
      publishJoints(HOME_JOINTS, gripper_val);
      auto result = std::make_shared<RTH::Result>();
      result->success = true;
      result->message = "Already at home position.";
      goal_handle->succeed(result);
      goal_active_ = false;
      return;
    }

    // 4. Compute trajectory duration T
    //    Smooth-step peak velocity = 1.5 * delta / T  =>  T = 1.5 * max_delta / v_max
    const double effective_speed = max_joint_speed_ * speed_scaling;
    const double T = (1.5 * max_abs_delta) / effective_speed;
    RCLCPP_INFO(this->get_logger(),
      "RTH trajectory: T=%.2f s, max_delta=%.3f rad, speed=%.3f rad/s",
      T, max_abs_delta, effective_speed);

    // 5. Pre-flight safety check: joint bounds + self-collision along N waypoints
    if (kinematic_state_ && planning_scene_) {
      if (!validatePathSafety(start, delta, collision_check_points_)) {
        RCLCPP_ERROR(this->get_logger(),
          "RTH aborted: path fails pre-flight safety check (joint bounds or self-collision).");
        auto result = std::make_shared<RTH::Result>();
        result->success = false;
        result->message = "RTH path fails pre-flight safety check (joint bounds or self-collision). Aborting.";
        goal_handle->abort(result);
        goal_active_ = false;
        return;
      }
    }

    // 6. Stream 50 Hz trajectory
    rclcpp::Rate rate(control_rate_);
    const auto t_start = this->now();
    std::vector<double> cmd(5);

    while (rclcpp::ok()) {
      // ── Cancellation check ─────────────────────────────────────
      if (goal_handle->is_canceling()) {
        // Publish the current interpolated position to freeze arm in place
        publishJoints(cmd.empty() ? start : cmd, gripper_val);
        auto result = std::make_shared<RTH::Result>();
        result->success = false;
        result->message = "RTH canceled by operator.";
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "RTH canceled. Arm halted at current position.");
        goal_active_ = false;
        return;
      }

      // ── Compute smooth-step position ───────────────────────────
      const double t = (this->now() - t_start).seconds();
      const double s = std::min(1.0, t / T);
      const double smooth_s = s * s * (3.0 - 2.0 * s);  // f(s) = 3s² - 2s³

      for (size_t i = 0; i < 5; ++i) {
        cmd[i] = start[i] + delta[i] * smooth_s;
      }

      // ── Publish /arm_cmd and /arm_joint_sync ───────────────────
      publishJoints(cmd, gripper_val);

      // ── Publish feedback ───────────────────────────────────────
      auto feedback = std::make_shared<RTH::Feedback>();
      feedback->progress = smooth_s;
      for (size_t i = 0; i < 5; ++i) {
        feedback->current_joints[i] = cmd[i];
      }
      goal_handle->publish_feedback(feedback);

      // ── Check arrival ──────────────────────────────────────────
      if (s >= 1.0) {
        publishJoints(HOME_JOINTS, gripper_val);  // snap to exact home
        auto result = std::make_shared<RTH::Result>();
        result->success = true;
        result->message = "Returned to home position successfully.";
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "RTH complete. Arm is at home.");
        goal_active_ = false;
        return;
      }

      rate.sleep();
    }

    goal_active_ = false;
  }

  // ── Helpers ───────────────────────────────────────────────────────

  /**
   * Publish a 6-element arm command [J0..J4, gripper] to /arm_cmd and /arm_joint_sync.
   */
  void publishJoints(const std::vector<double> & joints, double gripper)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.reserve(6);
    for (auto q : joints) msg.data.push_back(q);
    msg.data.push_back(gripper);
    arm_cmd_pub_->publish(msg);
    joint_sync_pub_->publish(msg);
  }

  /**
   * Validates N+1 evenly-spaced interpolated joint configurations.
   * Performs two checks at every waypoint:
   *   1. URDF joint-bounds check via kinematic_state_->satisfiesBounds().
   *   2. MoveIt self-collision check via planning_scene_->checkSelfCollision(),
   *      which uses FCL geometry and respects SRDF disable_collisions pairs.
   * Returns false immediately if either check fails at any waypoint.
   */
  bool validatePathSafety(
    const std::vector<double> & start,
    const std::vector<double> & delta,
    int n_points)
  {
    collision_detection::CollisionRequest col_req;
    col_req.contacts = false;  // only need a yes/no answer, not contact details
    col_req.verbose  = false;

    for (int k = 0; k <= n_points; ++k) {
      const double s        = static_cast<double>(k) / n_points;
      const double smooth_s = s * s * (3.0 - 2.0 * s);

      // Set waypoint joint positions
      for (size_t i = 0; i < JOINT_NAMES.size(); ++i) {
        kinematic_state_->setVariablePosition(JOINT_NAMES[i], start[i] + delta[i] * smooth_s);
      }
      kinematic_state_->update();

      // Check 1: URDF joint position limits
      if (!kinematic_state_->satisfiesBounds()) {
        RCLCPP_WARN(this->get_logger(),
          "RTH pre-flight: joint bounds violation at waypoint %d/%d. Aborting.",
          k, n_points);
        return false;
      }

      // Check 2: MoveIt self-collision (FCL + SRDF disable_collisions pairs)
      collision_detection::CollisionResult col_res;
      planning_scene_->checkSelfCollision(col_req, col_res, *kinematic_state_);
      if (col_res.collision) {
        RCLCPP_WARN(this->get_logger(),
          "RTH pre-flight: self-collision detected at waypoint %d/%d. Aborting.",
          k, n_points);
        return false;
      }
    }

    RCLCPP_INFO(this->get_logger(),
      "RTH pre-flight: all %d waypoints clear (joint bounds + self-collision). Proceeding.",
      n_points + 1);
    return true;
  }

  // ── Members ───────────────────────────────────────────────────────
  double max_joint_speed_;
  double control_rate_;
  int    collision_check_points_;
  bool   goal_active_;

  std::vector<double> current_joints_;
  double last_gripper_;
  std::mutex joints_mutex_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr arm_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_sync_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_sync_sub_;

  rclcpp_action::Server<RTH>::SharedPtr action_server_;

  std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
  moveit::core::RobotModelPtr kinematic_model_;
  moveit::core::RobotStatePtr kinematic_state_;
  std::shared_ptr<planning_scene::PlanningScene> planning_scene_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RTHNode>();
  node->initialize();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
