#pragma once

#include <moveit/kinematics_base/kinematics_base.h>
#include <pinocchio/multibody/model.hpp>

#include <mutex>

namespace qi_robot_pinocchio_ik
{
class PinocchioIKPlugin : public kinematics::KinematicsBase
{
public:
  bool initialize(const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModel& robot_model,
                  const std::string& group_name, const std::string& base_frame,
                  const std::vector<std::string>& tip_frames, double search_discretization) override;
  bool getPositionIK(const geometry_msgs::msg::Pose& pose, const std::vector<double>& seed,
                     std::vector<double>& solution, moveit_msgs::msg::MoveItErrorCodes& error,
                     const kinematics::KinematicsQueryOptions& options) const override;
  bool searchPositionIK(const geometry_msgs::msg::Pose&, const std::vector<double>&, double,
                        std::vector<double>&, moveit_msgs::msg::MoveItErrorCodes&,
                        const kinematics::KinematicsQueryOptions&) const override;
  bool searchPositionIK(const geometry_msgs::msg::Pose&, const std::vector<double>&, double,
                        const std::vector<double>&, std::vector<double>&,
                        moveit_msgs::msg::MoveItErrorCodes&, const kinematics::KinematicsQueryOptions&) const override;
  bool searchPositionIK(const geometry_msgs::msg::Pose&, const std::vector<double>&, double,
                        std::vector<double>&, const IKCallbackFn&, moveit_msgs::msg::MoveItErrorCodes&,
                        const kinematics::KinematicsQueryOptions&) const override;
  bool searchPositionIK(const geometry_msgs::msg::Pose&, const std::vector<double>&, double,
                        const std::vector<double>&, std::vector<double>&, const IKCallbackFn&,
                        moveit_msgs::msg::MoveItErrorCodes&, const kinematics::KinematicsQueryOptions&) const override;
  bool getPositionFK(const std::vector<std::string>&, const std::vector<double>&,
                     std::vector<geometry_msgs::msg::Pose>&) const override;
  const std::vector<std::string>& getJointNames() const override { return joint_names_; }
  const std::vector<std::string>& getLinkNames() const override { return link_names_; }

private:
  bool solve(const geometry_msgs::msg::Pose&, const std::vector<double>&, double,
             const std::vector<double>*, const IKCallbackFn&, std::vector<double>&,
             moveit_msgs::msg::MoveItErrorCodes&) const;
  bool makeConfiguration(const std::vector<double>& group_values, Eigen::VectorXd& q) const;
  bool poseFor(const Eigen::VectorXd& q, const std::string& link, geometry_msgs::msg::Pose& pose) const;

  pinocchio::Model model_;
  std::vector<std::string> joint_names_, link_names_;
  std::vector<int> q_indices_, v_indices_;
  int base_frame_id_{-1}, tip_frame_id_{-1};
  Eigen::VectorXd group_lower_, group_upper_;
  double position_threshold_{0.01}, orientation_threshold_{0.08}, damping_{1e-3}, max_step_{0.15};
  int max_iterations_{250};
  bool initialized_{false};
  mutable std::mutex solve_mutex_;
};
}  // namespace qi_robot_pinocchio_ik
