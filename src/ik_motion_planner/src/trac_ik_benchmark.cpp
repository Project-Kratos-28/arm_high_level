#include <memory>
#include <vector>
#include <string>
#include <cmath>
#include <chrono>
#include <numeric>
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <random>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>

struct BenchmarkStats
{
  std::string name;
  size_t total_samples;
  size_t success_count;
  double success_rate_pct;
  double total_time_sec;
  double throughput_hz;
  double min_us;
  double mean_us;
  double stddev_us;
  double median_us;
  double p90_us;
  double p95_us;
  double p99_us;
  double max_us;
  double cpu_load_pct;
  std::vector<double> latencies_us;
};

struct SweepResult
{
  double timeout_ms;
  double warm_sr_pct;
  double cold_sr_pct;
  double avg_sr_pct;
  double warm_cpu_pct;
  double cold_cpu_pct;
  double avg_cpu_pct;
  double score;
};

class TracIKBenchmarkNode : public rclcpp::Node
{
public:
  TracIKBenchmarkNode()
  : Node("trac_ik_benchmark", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    if (!this->has_parameter("planning_group")) {
      this->declare_parameter<std::string>("planning_group", "arm");
    }
    planning_group_ = this->get_parameter("planning_group").as_string();

    if (!this->has_parameter("tip_frame")) {
      this->declare_parameter<std::string>("tip_frame", "tool0");
    }
    tip_frame_ = this->get_parameter("tip_frame").as_string();

    if (!this->has_parameter("ik_timeout")) {
      this->declare_parameter<double>("ik_timeout", 0.005);
    }
    ik_timeout_ = this->get_parameter("ik_timeout").as_double();

    if (!this->has_parameter("samples")) {
      this->declare_parameter<int>("samples", 10000);
    }
    samples_ = this->get_parameter("samples").as_int();

    if (!this->has_parameter("sweep")) {
      this->declare_parameter<bool>("sweep", false);
    }
    sweep_ = this->get_parameter("sweep").as_bool();

    if (!this->has_parameter("sweep_min_ms")) {
      this->declare_parameter<double>("sweep_min_ms", 0.05);
    }
    sweep_min_ms_ = this->get_parameter("sweep_min_ms").as_double();

    if (!this->has_parameter("sweep_max_ms")) {
      this->declare_parameter<double>("sweep_max_ms", 1.50);
    }
    sweep_max_ms_ = this->get_parameter("sweep_max_ms").as_double();

    if (!this->has_parameter("sweep_step_ms")) {
      this->declare_parameter<double>("sweep_step_ms", 0.05);
    }
    sweep_step_ms_ = this->get_parameter("sweep_step_ms").as_double();
  }

  bool initialize()
  {
    RCLCPP_INFO(this->get_logger(), "===============================================================");
    RCLCPP_INFO(this->get_logger(), "             TRAC-IK KINEMATICS COMPUTE BENCHMARK              ");
    RCLCPP_INFO(this->get_logger(), "===============================================================");
    RCLCPP_INFO(this->get_logger(), "Loading robot model from robot_description...");

    robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(
      shared_from_this(), "robot_description");

    kinematic_model_ = robot_model_loader_->getModel();
    if (!kinematic_model_) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load kinematic model from robot_description.");
      return false;
    }

    kinematic_state_ = std::make_shared<moveit::core::RobotState>(kinematic_model_);
    joint_model_group_ = kinematic_model_->getJointModelGroup(planning_group_);
    if (!joint_model_group_) {
      RCLCPP_ERROR(this->get_logger(), "Planning group '%s' not found in robot model.", planning_group_.c_str());
      return false;
    }

    const auto & solver_instance = joint_model_group_->getSolverInstance();
    std::string solver_name = solver_instance ? typeid(*solver_instance).name() : "Default / KDL";

    RCLCPP_INFO(this->get_logger(), "Planning Group:       %s", planning_group_.c_str());
    RCLCPP_INFO(this->get_logger(), "Tip Frame:            %s", tip_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "Active Joints:        %zu", joint_model_group_->getActiveJointModelNames().size());
    RCLCPP_INFO(this->get_logger(), "Kinematics Plugin:    %s", solver_name.c_str());
    RCLCPP_INFO(this->get_logger(), "Sample Size:          %d per test", samples_);
    if (sweep_) {
      RCLCPP_INFO(this->get_logger(), "Mode:                 PARAMETRIC SWEEP (%.2f ms -> %.2f ms, step %.2f ms)",
        sweep_min_ms_, sweep_max_ms_, sweep_step_ms_);
    } else {
      RCLCPP_INFO(this->get_logger(), "Mode:                 SINGLE TIMEOUT (%.4f s / %.2f ms)", ik_timeout_, ik_timeout_ * 1000.0);
    }
    RCLCPP_INFO(this->get_logger(), "===============================================================\n");

    return true;
  }

  void execute()
  {
    if (sweep_) {
      runTimeoutSweep(sweep_min_ms_, sweep_max_ms_, sweep_step_ms_, samples_);
    } else {
      auto warm = runWarmStartBenchmark(ik_timeout_, samples_);
      auto cold = runColdStartBenchmark(ik_timeout_, samples_);
      printSingleReport(warm, cold);
    }
  }

  void cleanup()
  {
    joint_model_group_ = nullptr;
    kinematic_state_.reset();
    kinematic_model_.reset();
    robot_model_loader_.reset();
  }

  BenchmarkStats runWarmStartBenchmark(double timeout_sec, int num_samples)
  {
    kinematic_state_->setToDefaultValues();
    kinematic_state_->update();
    const Eigen::Isometry3d nominal_transform = kinematic_state_->getGlobalLinkTransform(tip_frame_);
    
    double base_x = nominal_transform.translation().x();
    double base_y = nominal_transform.translation().y();
    double base_z = nominal_transform.translation().z();
    Eigen::Quaterniond base_q(nominal_transform.rotation());

    std::vector<double> latencies_us;
    latencies_us.reserve(num_samples);
    size_t success = 0;

    auto t_start_total = std::chrono::steady_clock::now();

    for (int i = 0; i < num_samples; ++i) {
      double t = static_cast<double>(i) / num_samples * 8.0 * M_PI;
      geometry_msgs::msg::Pose target_pose;
      target_pose.position.x = base_x + 0.05 * std::cos(t);
      target_pose.position.y = base_y + 0.05 * std::sin(t);
      target_pose.position.z = base_z + 0.06 * std::sin(t * 0.5);
      target_pose.orientation.x = base_q.x();
      target_pose.orientation.y = base_q.y();
      target_pose.orientation.z = base_q.z();
      target_pose.orientation.w = base_q.w();

      auto t0 = std::chrono::steady_clock::now();
      bool ok = kinematic_state_->setFromIK(joint_model_group_, target_pose, tip_frame_, timeout_sec);
      auto t1 = std::chrono::steady_clock::now();

      double dur_us = std::chrono::duration<double, std::micro>(t1 - t0).count();
      latencies_us.push_back(dur_us);
      if (ok) {
        success++;
      }
    }

    auto t_end_total = std::chrono::steady_clock::now();
    double total_sec = std::chrono::duration<double>(t_end_total - t_start_total).count();

    return computeStats("Scenario A: Warm-Start Teleoperation", latencies_us, success, total_sec);
  }

  BenchmarkStats runColdStartBenchmark(double timeout_sec, int num_samples)
  {
    std::vector<geometry_msgs::msg::Pose> reachable_poses;
    reachable_poses.reserve(num_samples);
    
    for (int i = 0; i < num_samples; ++i) {
      kinematic_state_->setToRandomPositions(joint_model_group_);
      kinematic_state_->enforceBounds(joint_model_group_);
      kinematic_state_->update();

      const Eigen::Isometry3d tf = kinematic_state_->getGlobalLinkTransform(tip_frame_);
      geometry_msgs::msg::Pose p;
      p.position.x = tf.translation().x();
      p.position.y = tf.translation().y();
      p.position.z = tf.translation().z();
      Eigen::Quaterniond q(tf.rotation());
      p.orientation.x = q.x();
      p.orientation.y = q.y();
      p.orientation.z = q.z();
      p.orientation.w = q.w();
      reachable_poses.push_back(p);
    }

    std::vector<double> latencies_us;
    latencies_us.reserve(num_samples);
    size_t success = 0;

    auto t_start_total = std::chrono::steady_clock::now();

    for (int i = 0; i < num_samples; ++i) {
      kinematic_state_->setToDefaultValues();

      auto t0 = std::chrono::steady_clock::now();
      bool ok = kinematic_state_->setFromIK(joint_model_group_, reachable_poses[i], tip_frame_, timeout_sec);
      auto t1 = std::chrono::steady_clock::now();

      double dur_us = std::chrono::duration<double, std::micro>(t1 - t0).count();
      latencies_us.push_back(dur_us);
      if (ok) {
        success++;
      }
    }

    auto t_end_total = std::chrono::steady_clock::now();
    double total_sec = std::chrono::duration<double>(t_end_total - t_start_total).count();

    return computeStats("Scenario B: Cold-Start Random Reachable Poses", latencies_us, success, total_sec);
  }

  void runTimeoutSweep(double min_ms, double max_ms, double step_ms, int num_samples)
  {
    std::cout << "\n========================================================================================\n";
    std::cout << "                 TRAC-IK TIMEOUT PARAMETER SWEEP (" << min_ms << " ms -> " << max_ms << " ms, step " << step_ms << " ms)\n";
    std::cout << "                 Sample Size: " << num_samples << " per scenario | Formula: Score = (Avg_SR)^3 / Avg_CPU\n";
    std::cout << "========================================================================================\n";
    std::cout << std::left
              << std::setw(12) << "Timeout"
              << std::setw(14) << "Warm SR (%)"
              << std::setw(14) << "Cold SR (%)"
              << std::setw(14) << "Avg SR (%)"
              << std::setw(16) << "Avg CPU Load"
              << std::setw(14) << "Score"
              << "\n";
    std::cout << "----------------------------------------------------------------------------------------\n" << std::flush;

    // 1. Pre-generate shared continuous trajectory
    kinematic_state_->setToDefaultValues();
    kinematic_state_->update();
    const Eigen::Isometry3d nominal_transform = kinematic_state_->getGlobalLinkTransform(tip_frame_);
    double base_x = nominal_transform.translation().x();
    double base_y = nominal_transform.translation().y();
    double base_z = nominal_transform.translation().z();
    Eigen::Quaterniond base_q(nominal_transform.rotation());

    std::vector<geometry_msgs::msg::Pose> warm_poses;
    warm_poses.reserve(num_samples);
    for (int i = 0; i < num_samples; ++i) {
      double t = static_cast<double>(i) / num_samples * 8.0 * M_PI;
      geometry_msgs::msg::Pose target_pose;
      target_pose.position.x = base_x + 0.05 * std::cos(t);
      target_pose.position.y = base_y + 0.05 * std::sin(t);
      target_pose.position.z = base_z + 0.06 * std::sin(t * 0.5);
      target_pose.orientation.x = base_q.x();
      target_pose.orientation.y = base_q.y();
      target_pose.orientation.z = base_q.z();
      target_pose.orientation.w = base_q.w();
      warm_poses.push_back(target_pose);
    }

    // 2. Pre-generate shared cold-start random reachable poses
    std::vector<geometry_msgs::msg::Pose> cold_poses;
    cold_poses.reserve(num_samples);
    for (int i = 0; i < num_samples; ++i) {
      kinematic_state_->setToRandomPositions(joint_model_group_);
      kinematic_state_->enforceBounds(joint_model_group_);
      kinematic_state_->update();
      const Eigen::Isometry3d tf = kinematic_state_->getGlobalLinkTransform(tip_frame_);
      geometry_msgs::msg::Pose p;
      p.position.x = tf.translation().x();
      p.position.y = tf.translation().y();
      p.position.z = tf.translation().z();
      Eigen::Quaterniond q(tf.rotation());
      p.orientation.x = q.x();
      p.orientation.y = q.y();
      p.orientation.z = q.z();
      p.orientation.w = q.w();
      cold_poses.push_back(p);
    }

    std::vector<SweepResult> sweep_results;
    SweepResult best_result;
    best_result.score = -1.0;

    int total_steps = static_cast<int>(std::round((max_ms - min_ms) / step_ms)) + 1;
    int step_idx = 0;

    for (double t_ms = min_ms; t_ms <= max_ms + 1e-4; t_ms += step_ms) {
      step_idx++;
      double timeout_sec = t_ms / 1000.0;

      // Warm-start run
      kinematic_state_->setToDefaultValues();
      size_t warm_success = 0;
      double warm_sum_us = 0.0;
      for (int i = 0; i < num_samples; ++i) {
        auto t0 = std::chrono::steady_clock::now();
        bool ok = kinematic_state_->setFromIK(joint_model_group_, warm_poses[i], tip_frame_, timeout_sec);
        auto t1 = std::chrono::steady_clock::now();
        warm_sum_us += std::chrono::duration<double, std::micro>(t1 - t0).count();
        if (ok) warm_success++;
      }
      double warm_sr_pct = (static_cast<double>(warm_success) / num_samples) * 100.0;
      double warm_mean_us = warm_sum_us / num_samples;
      double warm_cpu_pct = 50.0 * (warm_mean_us / 1e6) * 2.0 * 100.0;

      // Cold-start run
      size_t cold_success = 0;
      double cold_sum_us = 0.0;
      for (int i = 0; i < num_samples; ++i) {
        kinematic_state_->setToDefaultValues();
        auto t0 = std::chrono::steady_clock::now();
        bool ok = kinematic_state_->setFromIK(joint_model_group_, cold_poses[i], tip_frame_, timeout_sec);
        auto t1 = std::chrono::steady_clock::now();
        cold_sum_us += std::chrono::duration<double, std::micro>(t1 - t0).count();
        if (ok) cold_success++;
      }
      double cold_sr_pct = (static_cast<double>(cold_success) / num_samples) * 100.0;
      double cold_mean_us = cold_sum_us / num_samples;
      double cold_cpu_pct = 50.0 * (cold_mean_us / 1e6) * 2.0 * 100.0;

      double avg_sr_pct = (warm_sr_pct + cold_sr_pct) / 2.0;
      double avg_cpu_pct = (warm_cpu_pct + cold_cpu_pct) / 2.0;

      // Formula: Score = (Avg_SR / 100)^3 / (Avg_CPU / 100)
      double s_frac = avg_sr_pct / 100.0;
      double c_frac = avg_cpu_pct / 100.0;
      double score = (c_frac > 0.0) ? (std::pow(s_frac, 3.0) / c_frac) : 0.0;

      SweepResult r;
      r.timeout_ms = t_ms;
      r.warm_sr_pct = warm_sr_pct;
      r.cold_sr_pct = cold_sr_pct;
      r.avg_sr_pct = avg_sr_pct;
      r.warm_cpu_pct = warm_cpu_pct;
      r.cold_cpu_pct = cold_cpu_pct;
      r.avg_cpu_pct = avg_cpu_pct;
      r.score = score;
      sweep_results.push_back(r);

      if (score > best_result.score) {
        best_result = r;
      }

      // Print live row
      std::ostringstream ss_timeout, ss_cpu, ss_score;
      ss_timeout << std::fixed << std::setprecision(2) << t_ms << " ms";
      ss_cpu << std::fixed << std::setprecision(2) << avg_cpu_pct << "%";
      ss_score << std::fixed << std::setprecision(2) << score;

      std::cout << std::left
                << std::setw(12) << ss_timeout.str()
                << std::setw(14) << (std::to_string(static_cast<int>(warm_sr_pct * 10.0) / 10.0).substr(0, 5) + "%")
                << std::setw(14) << (std::to_string(static_cast<int>(cold_sr_pct * 10.0) / 10.0).substr(0, 5) + "%")
                << std::setw(14) << (std::to_string(static_cast<int>(avg_sr_pct * 10.0) / 10.0).substr(0, 5) + "%")
                << std::setw(16) << ss_cpu.str()
                << std::setw(14) << ss_score.str()
                << " [" << step_idx << "/" << total_steps << "]"
                << "\n" << std::flush;
    }

    std::cout << "========================================================================================\n";
    std::cout << "                           OPTIMAL TIMEOUT RESULT (WINNER)                              \n";
    std::cout << "========================================================================================\n";
    std::cout << "  Optimal Timeout:       " << std::fixed << std::setprecision(2) << best_result.timeout_ms << " ms (" << best_result.timeout_ms / 1000.0 << " s)\n";
    std::cout << "  Max Efficiency Score:  " << std::fixed << std::setprecision(2) << best_result.score << "\n";
    std::cout << "  Warm-Start Success:    " << std::fixed << std::setprecision(1) << best_result.warm_sr_pct << "%\n";
    std::cout << "  Cold-Start Success:    " << std::fixed << std::setprecision(1) << best_result.cold_sr_pct << "%\n";
    std::cout << "  Average Success Rate:  " << std::fixed << std::setprecision(1) << best_result.avg_sr_pct << "%\n";
    std::cout << "  Estimated CPU Usage:   " << std::fixed << std::setprecision(2) << best_result.avg_cpu_pct << "% of 1 CPU Core\n";
    std::cout << "========================================================================================\n\n";

    // Write CSV file
    std::ofstream csv("trac_ik_sweep_results.csv");
    if (csv.is_open()) {
      csv << "timeout_ms,warm_sr_pct,cold_sr_pct,avg_sr_pct,warm_cpu_pct,cold_cpu_pct,avg_cpu_pct,score\n";
      for (const auto & sr : sweep_results) {
        csv << sr.timeout_ms << ","
            << sr.warm_sr_pct << ","
            << sr.cold_sr_pct << ","
            << sr.avg_sr_pct << ","
            << sr.warm_cpu_pct << ","
            << sr.cold_cpu_pct << ","
            << sr.avg_cpu_pct << ","
            << sr.score << "\n";
      }
      csv.close();
      std::cout << "[INFO] Saved full sweep data to 'trac_ik_sweep_results.csv'\n\n";
    }
  }

  void printSingleReport(const BenchmarkStats & warm, const BenchmarkStats & cold)
  {
    std::cout << "\n========================================================================================\n";
    std::cout << "                             BENCHMARK RESULTS & METRICS                                \n";
    std::cout << "========================================================================================\n";
    std::cout << std::left 
              << std::setw(32) << "Metric" 
              << std::setw(26) << "Warm-Start (Teleop)" 
              << std::setw(26) << "Cold-Start (Random)" 
              << "\n";
    std::cout << "----------------------------------------------------------------------------------------\n";
    
    auto printRow = [](const std::string & label, const std::string & v1, const std::string & v2) {
      std::cout << std::left << std::setw(32) << label 
                << std::setw(26) << v1 
                << std::setw(26) << v2 << "\n";
    };

    auto fmtUs = [](double us) {
      std::ostringstream ss;
      ss << std::fixed << std::setprecision(2) << (us / 1000.0) << " ms (" << std::setprecision(0) << us << " us)";
      return ss.str();
    };

    std::ostringstream ss_succ_w, ss_succ_c;
    ss_succ_w << warm.success_count << "/" << warm.total_samples << " (" << std::fixed << std::setprecision(1) << warm.success_rate_pct << "%)";
    ss_succ_c << cold.success_count << "/" << cold.total_samples << " (" << std::fixed << std::setprecision(1) << cold.success_rate_pct << "%)";
    printRow("Solve Success Rate", ss_succ_w.str(), ss_succ_c.str());

    std::ostringstream ss_thr_w, ss_thr_c;
    ss_thr_w << std::fixed << std::setprecision(0) << warm.throughput_hz << " solves/sec";
    ss_thr_c << std::fixed << std::setprecision(0) << cold.throughput_hz << " solves/sec";
    printRow("Peak Throughput", ss_thr_w.str(), ss_thr_c.str());

    printRow("Min Solve Time", fmtUs(warm.min_us), fmtUs(cold.min_us));
    printRow("Median (p50) Solve Time", fmtUs(warm.median_us), fmtUs(cold.median_us));
    printRow("Mean (Average) Solve Time", fmtUs(warm.mean_us), fmtUs(cold.mean_us));
    printRow("Std-Dev (+/-)", fmtUs(warm.stddev_us), fmtUs(cold.stddev_us));
    printRow("90th Percentile (p90)", fmtUs(warm.p90_us), fmtUs(cold.p90_us));
    printRow("95th Percentile (p95)", fmtUs(warm.p95_us), fmtUs(cold.p95_us));
    printRow("99th Percentile (p99)", fmtUs(warm.p99_us), fmtUs(cold.p99_us));
    printRow("Max Worst-Case Solve Time", fmtUs(warm.max_us), fmtUs(cold.max_us));

    std::cout << "----------------------------------------------------------------------------------------\n";
    
    std::ostringstream ss_cpu_w, ss_cpu_c;
    ss_cpu_w << std::fixed << std::setprecision(2) << warm.cpu_load_pct << "% of 1 CPU Core";
    ss_cpu_c << std::fixed << std::setprecision(2) << cold.cpu_load_pct << "% of 1 CPU Core";
    printRow("Est. CPU Load @ 50 Hz Loop", ss_cpu_w.str(), ss_cpu_c.str());

    double avg_sr = (warm.success_rate_pct + cold.success_rate_pct) / 2.0 / 100.0;
    double avg_cpu = (warm.cpu_load_pct + cold.cpu_load_pct) / 2.0 / 100.0;
    double score = (avg_cpu > 0.0) ? (std::pow(avg_sr, 3.0) / avg_cpu) : 0.0;
    std::ostringstream ss_score;
    ss_score << std::fixed << std::setprecision(2) << score;
    printRow("Efficiency Score (Option 2)", ss_score.str(), ss_score.str());

    std::cout << "========================================================================================\n\n";
  }

private:
  BenchmarkStats computeStats(
    const std::string & name,
    std::vector<double> latencies_us,
    size_t success,
    double total_sec)
  {
    BenchmarkStats stats;
    stats.name = name;
    stats.total_samples = latencies_us.size();
    stats.success_count = success;
    stats.success_rate_pct = (static_cast<double>(success) / stats.total_samples) * 100.0;
    stats.total_time_sec = total_sec;
    stats.throughput_hz = stats.total_samples / total_sec;

    std::sort(latencies_us.begin(), latencies_us.end());
    stats.min_us = latencies_us.front();
    stats.max_us = latencies_us.back();
    stats.median_us = latencies_us[static_cast<size_t>(stats.total_samples * 0.50)];
    stats.p90_us = latencies_us[static_cast<size_t>(stats.total_samples * 0.90)];
    stats.p95_us = latencies_us[static_cast<size_t>(stats.total_samples * 0.95)];
    stats.p99_us = latencies_us[static_cast<size_t>(stats.total_samples * 0.99)];

    double sum = std::accumulate(latencies_us.begin(), latencies_us.end(), 0.0);
    stats.mean_us = sum / stats.total_samples;

    double accum = 0.0;
    for (double x : latencies_us) {
      accum += (x - stats.mean_us) * (x - stats.mean_us);
    }
    stats.stddev_us = std::sqrt(accum / stats.total_samples);
    stats.cpu_load_pct = 50.0 * (stats.mean_us / 1e6) * 2.0 * 100.0;
    stats.latencies_us = std::move(latencies_us);

    return stats;
  }

  std::string planning_group_;
  std::string tip_frame_;
  double ik_timeout_;
  int samples_;
  bool sweep_;
  double sweep_min_ms_;
  double sweep_max_ms_;
  double sweep_step_ms_;

  std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
  moveit::core::RobotModelPtr kinematic_model_;
  moveit::core::RobotStatePtr kinematic_state_;
  const moveit::core::JointModelGroup * joint_model_group_{nullptr};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TracIKBenchmarkNode>();
  if (node->initialize()) {
    node->execute();
  }
  node->cleanup();
  node.reset();
  rclcpp::shutdown();
  return 0;
}
