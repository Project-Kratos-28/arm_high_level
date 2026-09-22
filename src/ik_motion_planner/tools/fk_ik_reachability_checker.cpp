#include <memory>
#include <vector>
#include <string>
#include <cmath>
#include <chrono>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <map>
#include <set>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>

struct FKPoint {
  double r;
  double z;
  double x_base;
  double y_base;
  double q1;
  double q2;
  double q3;
  double world_pitch;
  double min_pitch;
  double max_pitch;
  int sample_count;
};

class ReachabilityCheckerNode : public rclcpp::Node {
public:
  ReachabilityCheckerNode()
  : Node("fk_ik_reachability_checker", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    if (!this->has_parameter("planning_group")) {
      this->declare_parameter<std::string>("planning_group", "arm");
    }
    planning_group_ = this->get_parameter("planning_group").as_string();

    if (!this->has_parameter("wrist_planning_group")) {
      this->declare_parameter<std::string>("wrist_planning_group", "arm_wrist");
    }
    wrist_planning_group_ = this->get_parameter("wrist_planning_group").as_string();

    if (!this->has_parameter("tip_frame")) {
      this->declare_parameter<std::string>("tip_frame", "tool0");
    }
    tip_frame_ = this->get_parameter("tip_frame").as_string();

    if (!this->has_parameter("wrist_tip_frame")) {
      this->declare_parameter<std::string>("wrist_tip_frame", "wrist_center");
    }
    wrist_tip_frame_ = this->get_parameter("wrist_tip_frame").as_string();

    if (!this->has_parameter("base_frame")) {
      this->declare_parameter<std::string>("base_frame", "base_link");
    }
    base_frame_ = this->get_parameter("base_frame").as_string();

    if (!this->has_parameter("ik_timeout")) {
      this->declare_parameter<double>("ik_timeout", 0.005);
    }
    ik_timeout_ = this->get_parameter("ik_timeout").as_double();

    if (!this->has_parameter("output_csv")) {
      this->declare_parameter<std::string>("output_csv", "fk_ik_reachability_report.csv");
    }
    output_csv_ = this->get_parameter("output_csv").as_string();
  }

  bool run() {
    RCLCPP_INFO(this->get_logger(), "Loading robot model from robot_description...");
    auto robot_model_loader = std::make_shared<robot_model_loader::RobotModelLoader>(shared_from_this(), "robot_description");
    auto kinematic_model = robot_model_loader->getModel();
    if (!kinematic_model) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load kinematic model.");
      return false;
    }

    auto jmg_arm = kinematic_model->getJointModelGroup(planning_group_);
    if (!jmg_arm) {
      RCLCPP_ERROR(this->get_logger(), "Planning group '%s' not found.", planning_group_.c_str());
      return false;
    }

    auto jmg_wrist = kinematic_model->getJointModelGroup(wrist_planning_group_);
    if (!jmg_wrist) {
      RCLCPP_WARN(this->get_logger(), "Decoupled group '%s' not found; using group '%s'.",
        wrist_planning_group_.c_str(), planning_group_.c_str());
      jmg_wrist = jmg_arm;
    }

    auto kinematic_state = std::make_shared<moveit::core::RobotState>(kinematic_model);
    kinematic_state->setToDefaultValues();

    std::vector<std::string> arm_joint_names = {
      "base_yaw_joint", "shoulder_joint", "elbow_joint", "wrist_pitch_joint", "wrist_roll_joint"
    };

    // 1. Densely sample joint space across physical joint limits
    // shoulder [-1.57, 1.57], elbow [-2.50, 2.50], wrist_pitch [-1.57, 1.57]
    const int q1_steps = 41;
    const int q2_steps = 61;
    const int q3_steps = 41;
    size_t total_samples = q1_steps * q2_steps * q3_steps;

    RCLCPP_INFO(this->get_logger(), "Sampling FK workspace across %zu joint configurations...", total_samples);

    double q1_min = -1.57, q1_max = 1.57;
    double q2_min = -2.50, q2_max = 2.50;
    double q3_min = -1.57, q3_max = 1.57;

    double min_r = 1e9, max_r = -1e9;
    double min_z = 1e9, max_z = -1e9;

    const double grid_res = 0.020; // 20 mm grid resolution
    std::map<std::pair<int, int>, FKPoint> grid_cells;

    auto t0_fk = std::chrono::steady_clock::now();

    for (int i = 0; i < q1_steps; ++i) {
      double q1 = q1_min + i * (q1_max - q1_min) / (q1_steps - 1);
      for (int j = 0; j < q2_steps; ++j) {
        double q2 = q2_min + j * (q2_max - q2_min) / (q2_steps - 1);
        for (int k = 0; k < q3_steps; ++k) {
          double q3 = q3_min + k * (q3_max - q3_min) / (q3_steps - 1);

          kinematic_state->setVariablePosition(arm_joint_names[0], 0.0);
          kinematic_state->setVariablePosition(arm_joint_names[1], q1);
          kinematic_state->setVariablePosition(arm_joint_names[2], q2);
          kinematic_state->setVariablePosition(arm_joint_names[3], q3);
          kinematic_state->setVariablePosition(arm_joint_names[4], 0.0);
          kinematic_state->update();

          Eigen::Isometry3d tf_tool0 = kinematic_state->getGlobalLinkTransform(tip_frame_);
          Eigen::Isometry3d tf_base = kinematic_state->getFrameTransform(base_frame_);
          Eigen::Vector3d pos_base = (tf_base.inverse() * tf_tool0).translation();

          double r = std::hypot(pos_base.x(), pos_base.y());
          double z = pos_base.z();
          double world_pitch = -(q1 + q2 - 2.00719 + q3 + 0.4363323);

          min_r = std::min(min_r, r);
          max_r = std::max(max_r, r);
          min_z = std::min(min_z, z);
          max_z = std::max(max_z, z);

          int r_idx = static_cast<int>(std::round(r / grid_res));
          int z_idx = static_cast<int>(std::round(z / grid_res));
          auto key = std::make_pair(r_idx, z_idx);

          auto it = grid_cells.find(key);
          if (it == grid_cells.end()) {
            grid_cells[key] = {r, z, pos_base.x(), pos_base.y(), q1, q2, q3, world_pitch, world_pitch, world_pitch, 1};
          } else {
            it->second.min_pitch = std::min(it->second.min_pitch, world_pitch);
            it->second.max_pitch = std::max(it->second.max_pitch, world_pitch);
            it->second.sample_count++;
            // Keep the configuration closest to horizontal pitch (pitch ~ 0) as representative
            if (std::abs(world_pitch) < std::abs(it->second.world_pitch)) {
              it->second.r = r;
              it->second.z = z;
              it->second.x_base = pos_base.x();
              it->second.y_base = pos_base.y();
              it->second.q1 = q1;
              it->second.q2 = q2;
              it->second.q3 = q3;
              it->second.world_pitch = world_pitch;
            }
          }
        }
      }
    }

    auto t1_fk = std::chrono::steady_clock::now();
    double fk_sec = std::chrono::duration<double>(t1_fk - t0_fk).count();

    size_t total_points = grid_cells.size();
    double workspace_area = total_points * (grid_res * grid_res);

    RCLCPP_INFO(this->get_logger(), "FK Sweep Complete in %.3f s. Sampled %zu states -> %zu unique 20mm reachable Cartesian cells.",
      fk_sec, total_samples, total_points);
    RCLCPP_INFO(this->get_logger(), "Physical Workspace Envelope: Reach r=[%.3f, %.3f] m, Elevation z=[%.3f, %.3f] m, Area=%.3f m^2",
      min_r, max_r, min_z, max_z, workspace_area);

    // 2. Open CSV output
    std::ofstream csv(output_csv_);
    if (!csv.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open output CSV: %s", output_csv_.c_str());
      return false;
    }

    csv << "r,z,x_base,y_base,fk_q1,fk_q2,fk_q3,world_pitch,min_pitch,max_pitch,sample_count,"
        << "in_mapper_limits,in_mapper_sphere,in_mapper_envelope,"
        << "hybrid_ik_success,hybrid_ik_status,hybrid_ik_mode,hybrid_ik_error_m,"
        << "direct_ik_success,direct_ik_status,direct_ik_error_m,"
        << "ik_q0,ik_q1,ik_q2,ik_q3,ik_q4\n";

    // Decoupled wrist offsets from URDF bevel origin
    const double L_par = 0.23700016;
    const double L_perp = -0.00800254;

    // Expanded ps5_mapper envelope parameters (physical max)
    const double op_r_min = 0.05, op_r_max = 1.22;
    const double op_z_min = -0.65, op_z_max = 1.31;
    const double shoulder_pivot_z = 0.10425;
    const double workspace_radius = 1.21;
    const double cartesian_tol_m = 0.010; // 10 mm tolerance

    // Counters
    std::map<std::string, size_t> hybrid_status_counts;
    std::map<std::string, size_t> direct_status_counts;
    size_t hybrid_success_count = 0;
    size_t direct_success_count = 0;
    size_t decoupled_primary_count = 0;
    size_t fallback_engaged_count = 0;

    auto t0_ik = std::chrono::steady_clock::now();
    size_t evaluated = 0;

    RCLCPP_INFO(this->get_logger(), "Evaluating Hybrid IK reachability on all %zu FK reachable points...", total_points);

    for (const auto & pair : grid_cells) {
      const auto & pt = pair.second;
      evaluated++;

      // Check ps5_mapper expanded soft limits
      bool in_rect = (pt.r >= op_r_min && pt.r <= op_r_max && pt.z >= op_z_min && pt.z <= op_z_max);
      double z_rel = pt.z - shoulder_pivot_z;
      bool in_sphere = (pt.r * pt.r + z_rel * z_rel) <= (workspace_radius * workspace_radius + 1e-6);
      bool in_mapper_envelope = in_rect && in_sphere;

      kinematic_state->update();
      Eigen::Isometry3d tf_base = kinematic_state->getFrameTransform(base_frame_);
      Eigen::Vector3d tool0_target_root = tf_base * Eigen::Vector3d(pt.x_base, pt.y_base, pt.z);

      // =============================================================
      // TEST A: HYBRID IK PIPELINE (Decoupled Stage 1 + 5-DOF Fallback Stage 2)
      // =============================================================
      double delta_r = L_par * std::cos(pt.world_pitch) - L_perp * std::sin(pt.world_pitch);
      double delta_z = L_par * std::sin(pt.world_pitch) + L_perp * std::cos(pt.world_pitch);
      
      double xw_base = pt.x_base;
      double yw_base = pt.y_base + delta_r;
      double zw_base = pt.z - delta_z;

      Eigen::Vector3d pw_in_root = tf_base * Eigen::Vector3d(xw_base, yw_base, zw_base);

      geometry_msgs::msg::Pose wrist_target_pose;
      wrist_target_pose.position.x = pw_in_root.x();
      wrist_target_pose.position.y = pw_in_root.y();
      wrist_target_pose.position.z = pw_in_root.z();
      wrist_target_pose.orientation.w = 1.0;

      // Seed with initial home configuration
      kinematic_state->setVariablePosition("base_yaw_joint", 0.0);
      kinematic_state->setVariablePosition("shoulder_joint", 0.0);
      kinematic_state->setVariablePosition("elbow_joint", 2.0072);
      kinematic_state->setVariablePosition("wrist_pitch_joint", 0.0);
      kinematic_state->setVariablePosition("wrist_roll_joint", 0.0);

      bool wrist_ik_ok = kinematic_state->setFromIK(jmg_wrist, wrist_target_pose, wrist_tip_frame_, ik_timeout_);

      bool decoupled_accepted = false;
      std::string hybrid_mode = "DECOUPLED_PRIMARY";
      std::string hybrid_status;
      double hybrid_pos_err = 0.0;
      std::vector<double> solved_q(5, 0.0);

      if (wrist_ik_ok) {
        double cand_q0 = kinematic_state->getVariablePosition(arm_joint_names[0]);
        double cand_q1 = kinematic_state->getVariablePosition(arm_joint_names[1]);
        double cand_q2 = kinematic_state->getVariablePosition(arm_joint_names[2]);

        double q3_analytical = -pt.world_pitch - (cand_q1 + cand_q2 - 2.00719 + 0.4363323);
        bool q3_in_limits = (q3_analytical >= -1.57 && q3_analytical <= 1.57);

        if (q3_in_limits) {
          kinematic_state->setVariablePosition(arm_joint_names[0], cand_q0);
          kinematic_state->setVariablePosition(arm_joint_names[1], cand_q1);
          kinematic_state->setVariablePosition(arm_joint_names[2], cand_q2);
          kinematic_state->setVariablePosition(arm_joint_names[3], q3_analytical);
          kinematic_state->setVariablePosition(arm_joint_names[4], 0.0);
          kinematic_state->update();

          Eigen::Isometry3d tf_tool0_actual = kinematic_state->getGlobalLinkTransform(tip_frame_);
          hybrid_pos_err = (tf_tool0_actual.translation() - tool0_target_root).norm();

          if (hybrid_pos_err <= cartesian_tol_m) {
            solved_q[0] = cand_q0;
            solved_q[1] = cand_q1;
            solved_q[2] = cand_q2;
            solved_q[3] = q3_analytical;
            solved_q[4] = 0.0;
            decoupled_accepted = true;
            decoupled_primary_count++;
          }
        }
      }

      // Stage 2: Fallback to 5-DOF TRAC-IK if decoupled stage rejected
      if (!decoupled_accepted) {
        fallback_engaged_count++;
        hybrid_mode = "5DOF_FALLBACK";

        geometry_msgs::msg::Pose tool0_target_pose;
        tool0_target_pose.position.x = tool0_target_root.x();
        tool0_target_pose.position.y = tool0_target_root.y();
        tool0_target_pose.position.z = tool0_target_root.z();
        tool0_target_pose.orientation.w = 1.0;

        // Re-seed with home configuration
        kinematic_state->setVariablePosition("base_yaw_joint", 0.0);
        kinematic_state->setVariablePosition("shoulder_joint", 0.0);
        kinematic_state->setVariablePosition("elbow_joint", 2.0072);
        kinematic_state->setVariablePosition("wrist_pitch_joint", 0.0);
        kinematic_state->setVariablePosition("wrist_roll_joint", 0.0);

        bool fallback_ok = kinematic_state->setFromIK(jmg_arm, tool0_target_pose, tip_frame_, ik_timeout_);

        if (fallback_ok) {
          for (size_t idx = 0; idx < 5; ++idx) {
            solved_q[idx] = kinematic_state->getVariablePosition(arm_joint_names[idx]);
          }
          kinematic_state->update();
          Eigen::Isometry3d tf_tool0_actual = kinematic_state->getGlobalLinkTransform(tip_frame_);
          hybrid_pos_err = (tf_tool0_actual.translation() - tool0_target_root).norm();

          if (hybrid_pos_err <= cartesian_tol_m) {
            decoupled_accepted = true;
          }
        }
      }

      bool hybrid_success = false;
      if (decoupled_accepted) {
        hybrid_success = true;
        hybrid_success_count++;
        if (!in_mapper_envelope) {
          hybrid_status = "REACHABLE_IK_BUT_CLAMPED_BY_MAPPER_SOFT_LIMITS";
        } else if (hybrid_mode == "DECOUPLED_PRIMARY") {
          hybrid_status = "SUCCESS_DECOUPLED_EXACT_PITCH";
        } else {
          hybrid_status = "SUCCESS_HYBRID_5DOF_FALLBACK";
        }
      } else {
        if (!in_mapper_envelope) {
          hybrid_status = "FAIL_SOLVER_AND_OUTSIDE_MAPPER_SOFT_LIMITS";
        } else {
          hybrid_status = "FAIL_SOLVER_TIMEOUT_OR_DIVERGENCE";
        }
      }
      hybrid_status_counts[hybrid_status]++;

      // =============================================================
      // TEST B: DIRECT 5-DOF TRAC-IK (Benchmark Reference)
      // =============================================================
      geometry_msgs::msg::Pose tool0_target_pose;
      tool0_target_pose.position.x = tool0_target_root.x();
      tool0_target_pose.position.y = tool0_target_root.y();
      tool0_target_pose.position.z = tool0_target_root.z();
      tool0_target_pose.orientation.w = 1.0;

      kinematic_state->setVariablePosition("base_yaw_joint", 0.0);
      kinematic_state->setVariablePosition("shoulder_joint", 0.0);
      kinematic_state->setVariablePosition("elbow_joint", 2.0072);
      kinematic_state->setVariablePosition("wrist_pitch_joint", 0.0);
      kinematic_state->setVariablePosition("wrist_roll_joint", 0.0);

      bool direct_ik_ok = kinematic_state->setFromIK(jmg_arm, tool0_target_pose, tip_frame_, ik_timeout_);

      std::string direct_status;
      double direct_pos_err = 0.0;
      bool direct_success = false;

      if (direct_ik_ok) {
        kinematic_state->update();
        Eigen::Isometry3d actual_tool0_direct = kinematic_state->getGlobalLinkTransform(tip_frame_);
        direct_pos_err = (actual_tool0_direct.translation() - tool0_target_root).norm();

        if (direct_pos_err <= cartesian_tol_m) {
          direct_status = "DIRECT_SUCCESS";
          direct_success = true;
          direct_success_count++;
        } else {
          direct_status = "DIRECT_FAIL_CARTESIAN_TOLERANCE";
        }
      } else {
        direct_status = "DIRECT_FAIL_TIMEOUT_OR_DIVERGENCE";
      }
      direct_status_counts[direct_status]++;

      // Write Row to CSV
      csv << std::fixed << std::setprecision(4)
          << pt.r << "," << pt.z << ","
          << pt.x_base << "," << pt.y_base << ","
          << pt.q1 << "," << pt.q2 << "," << pt.q3 << ","
          << pt.world_pitch << "," << pt.min_pitch << "," << pt.max_pitch << "," << pt.sample_count << ","
          << (in_rect ? 1 : 0) << "," << (in_sphere ? 1 : 0) << "," << (in_mapper_envelope ? 1 : 0) << ","
          << (hybrid_success ? 1 : 0) << "," << hybrid_status << "," << hybrid_mode << "," << std::setprecision(6) << hybrid_pos_err << ","
          << (direct_success ? 1 : 0) << "," << direct_status << "," << std::setprecision(6) << direct_pos_err << ","
          << std::setprecision(4)
          << solved_q[0] << "," << solved_q[1] << "," << solved_q[2] << "," << solved_q[3] << "," << solved_q[4]
          << "\n";

      if (evaluated % 250 == 0 || evaluated == total_points) {
        std::cout << "\r[Hybrid IK Reachability Progress] " << evaluated << "/" << total_points
                  << " (" << std::fixed << std::setprecision(1) << (static_cast<double>(evaluated) / total_points * 100.0) << "%)"
                  << " | Hybrid IK Total: " << (static_cast<double>(hybrid_success_count) / evaluated * 100.0) << "%"
                  << " | Direct TRAC-IK: " << (static_cast<double>(direct_success_count) / evaluated * 100.0) << "%"
                  << std::flush;
      }
    }
    std::cout << std::endl;

    auto t1_ik = std::chrono::steady_clock::now();
    double total_ik_sec = std::chrono::duration<double>(t1_ik - t0_ik).count();
    csv.close();

    // 3. Print Comprehensive Report
    double hybrid_pct = (total_points > 0) ? (static_cast<double>(hybrid_success_count) / total_points * 100.0) : 0.0;
    double direct_pct = (total_points > 0) ? (static_cast<double>(direct_success_count) / total_points * 100.0) : 0.0;

    std::cout << "\n========================================================================================\n";
    std::cout << "             HYBRID IK vs. DIRECT TRAC-IK WORKSPACE REACHABILITY REPORT                 \n";
    std::cout << "========================================================================================\n";
    std::cout << std::left << std::setw(45) << "Metric" << "Value\n";
    std::cout << "----------------------------------------------------------------------------------------\n";
    std::cout << std::left << std::setw(45) << "Total FK Joint Configurations Sampled:" << total_samples << " configurations\n";
    std::cout << std::left << std::setw(45) << "Unique 20mm Reachable Cartesian Points:" << total_points << " unique (r, z) cells\n";
    std::cout << std::left << std::setw(45) << "Total Cross-Sectional Reachable Area:" << std::fixed << std::setprecision(3) << workspace_area << " m^2\n";
    std::cout << std::left << std::setw(45) << "FK Physical Reach Range (r):" << "[" << min_r << " m, " << max_r << " m]\n";
    std::cout << std::left << std::setw(45) << "FK Physical Elevation Range (z):" << "[" << min_z << " m, " << max_z << " m]\n";
    std::cout << std::left << std::setw(45) << "Direct MoveIt TRAC-IK Success Rate:" << direct_success_count << " / " << total_points << " (" << std::fixed << std::setprecision(2) << direct_pct << "%)\n";
    std::cout << std::left << std::setw(45) << "Hybrid IK Total Attainability:" << hybrid_success_count << " / " << total_points << " (" << std::fixed << std::setprecision(2) << hybrid_pct << "%)\n";
    std::cout << std::left << std::setw(45) << "  - Solved via Decoupled (Exact Pitch):" << decoupled_primary_count << " (" << std::fixed << std::setprecision(2) << (static_cast<double>(decoupled_primary_count)/total_points*100.0) << "%)\n";
    std::cout << std::left << std::setw(45) << "  - Solved via 5-DOF Fallback (Position Priority):" << (hybrid_success_count - decoupled_primary_count) << " (" << std::fixed << std::setprecision(2) << (static_cast<double>(hybrid_success_count - decoupled_primary_count)/total_points*100.0) << "%)\n";
    std::cout << std::left << std::setw(45) << "Total IK Evaluation Time:" << std::fixed << std::setprecision(2) << total_ik_sec << " s ("
              << (total_points / total_ik_sec) << " points/sec)\n";
    std::cout << "----------------------------------------------------------------------------------------\n";
    std::cout << "HYBRID IK DETAILED STATUS BREAKDOWN:\n";
    for (const auto & entry : hybrid_status_counts) {
      double pct = static_cast<double>(entry.second) / total_points * 100.0;
      std::cout << "  - " << std::left << std::setw(50) << entry.first
                << ": " << std::setw(6) << entry.second
                << " (" << std::fixed << std::setprecision(2) << pct << "%)\n";
    }
    std::cout << "----------------------------------------------------------------------------------------\n";
    std::cout << "DIRECT MOVEIT TRAC-IK STATUS BREAKDOWN:\n";
    for (const auto & entry : direct_status_counts) {
      double pct = static_cast<double>(entry.second) / total_points * 100.0;
      std::cout << "  - " << std::left << std::setw(50) << entry.first
                << ": " << std::setw(6) << entry.second
                << " (" << std::fixed << std::setprecision(2) << pct << "%)\n";
    }
    std::cout << "----------------------------------------------------------------------------------------\n";
    std::cout << "Detailed point-by-point report saved to: " << output_csv_ << "\n";
    std::cout << "========================================================================================\n\n";

    return true;
  }

private:
  std::string planning_group_;
  std::string wrist_planning_group_;
  std::string tip_frame_;
  std::string wrist_tip_frame_;
  std::string base_frame_;
  double ik_timeout_;
  std::string output_csv_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ReachabilityCheckerNode>();
  bool ok = node->run();
  rclcpp::shutdown();
  return ok ? 0 : 1;
}
