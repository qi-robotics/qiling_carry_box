#include <qi_robot_pinocchio_ik/pinocchio_ik_plugin.hpp>

#include <pluginlib/class_list_macros.hpp>
#include <moveit/robot_model/robot_model.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace qi_robot_pinocchio_ik
{
namespace
{
geometry_msgs::msg::Pose toPose(const pinocchio::SE3& transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();
  Eigen::Quaterniond q(transform.rotation());
  pose.orientation.x = q.x(); pose.orientation.y = q.y(); pose.orientation.z = q.z(); pose.orientation.w = q.w();
  return pose;
}

bool validPose(const geometry_msgs::msg::Pose& pose)
{
  const double n = pose.orientation.x * pose.orientation.x + pose.orientation.y * pose.orientation.y +
                   pose.orientation.z * pose.orientation.z + pose.orientation.w * pose.orientation.w;
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) && std::isfinite(pose.position.z) &&
         std::isfinite(n) && n > 1e-12;
}
}  // namespace

bool PinocchioIKPlugin::initialize(const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModel& robot_model,
                                   const std::string& group_name, const std::string& base_frame,
                                   const std::vector<std::string>& tip_frames, double search_discretization)
{
  if (tip_frames.size() != 1)
  {
    RCLCPP_ERROR(node->get_logger(), "PinocchioIKPlugin supports one tip frame per solver instance");
    return false;
  }
  node_ = node;
  group_name_ = group_name;
  base_frame_ = base_frame;
  tip_frames_ = tip_frames;
  setSearchDiscretization(search_discretization);
  const auto* group = robot_model.getJointModelGroup(group_name);
  if (!group || !group->isChain())
  {
    RCLCPP_ERROR(node->get_logger(), "PinocchioIKPlugin requires a single-chain group: %s", group_name.c_str());
    return false;
  }
  std::string urdf_xml;
  if (!node->get_parameter("robot_description", urdf_xml) || urdf_xml.empty())
  {
    RCLCPP_ERROR(node->get_logger(), "robot_description is unavailable while initializing Pinocchio IK");
    return false;
  }
  try { pinocchio::urdf::buildModelFromXML(urdf_xml, model_); }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(node->get_logger(), "Pinocchio could not parse robot_description: %s", e.what());
    return false;
  }
  base_frame_id_ = static_cast<int>(model_.getFrameId(base_frame));
  tip_frame_id_ = static_cast<int>(model_.getFrameId(tip_frames.front()));
  if (base_frame_id_ >= static_cast<int>(model_.nframes) || tip_frame_id_ >= static_cast<int>(model_.nframes))
  {
    RCLCPP_ERROR(node->get_logger(), "Pinocchio model lacks base '%s' or tip '%s'", base_frame.c_str(), tip_frames.front().c_str());
    return false;
  }
  joint_names_ = group->getVariableNames();
  link_names_ = group->getLinkModelNames();
  q_indices_.clear();
  v_indices_.clear();
  for (const auto& name : joint_names_)
  {
    const auto jid = model_.getJointId(name);
    if (jid == 0 || model_.joints[jid].nq() != 1 || model_.joints[jid].nv() != 1)
    {
      RCLCPP_ERROR(node->get_logger(), "Joint '%s' is missing or is not a scalar joint in Pinocchio", name.c_str());
      return false;
    }
    q_indices_.push_back(static_cast<int>(model_.joints[jid].idx_q()));
    v_indices_.push_back(static_cast<int>(model_.joints[jid].idx_v()));
  }
  group_lower_.resize(joint_names_.size()); group_upper_.resize(joint_names_.size());
  for (size_t i = 0; i < q_indices_.size(); ++i)
  {
    group_lower_[i] = model_.lowerPositionLimit[q_indices_[i]];
    group_upper_[i] = model_.upperPositionLimit[q_indices_[i]];
  }
  lookupParam(node, "position_threshold", position_threshold_, position_threshold_);
  lookupParam(node, "orientation_threshold", orientation_threshold_, orientation_threshold_);
  lookupParam(node, "damping", damping_, damping_);
  lookupParam(node, "max_step", max_step_, max_step_);
  lookupParam(node, "max_iterations", max_iterations_, max_iterations_);
  initialized_ = true;
  RCLCPP_INFO(node->get_logger(), "Pinocchio IK initialized for %s (%zu variables, %s -> %s)", group_name.c_str(),
              joint_names_.size(), base_frame.c_str(), tip_frames.front().c_str());
  return true;
}

bool PinocchioIKPlugin::makeConfiguration(const std::vector<double>& values, Eigen::VectorXd& q) const
{
  if (values.size() != q_indices_.size()) return false;
  q = pinocchio::neutral(model_);
  for (size_t i = 0; i < values.size(); ++i)
  {
    if (!std::isfinite(values[i])) return false;
    if (values[i] < group_lower_[i] || values[i] > group_upper_[i]) return false;
    q[q_indices_[i]] = values[i];
  }
  return true;
}

bool PinocchioIKPlugin::poseFor(const Eigen::VectorXd& q, const std::string& link, geometry_msgs::msg::Pose& pose) const
{
  const auto frame = model_.getFrameId(link);
  if (frame >= model_.nframes) return false;
  pinocchio::Data data(model_);
  pinocchio::forwardKinematics(model_, data, q);
  pinocchio::updateFramePlacements(model_, data);
  pose = toPose(data.oMf[base_frame_id_].actInv(data.oMf[frame]));
  return true;
}

bool PinocchioIKPlugin::solve(const geometry_msgs::msg::Pose& target_pose, const std::vector<double>& seed, double timeout,
                              const std::vector<double>* consistency, const IKCallbackFn& callback,
                              std::vector<double>& solution, moveit_msgs::msg::MoveItErrorCodes& error) const
{
  error.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
  if (!initialized_ || !validPose(target_pose) || seed.size() != q_indices_.size())
  {
    RCLCPP_WARN(node_->get_logger(),
                "Pinocchio IK rejected request for %s: initialized=%d pose_valid=%d seed_size=%zu expected=%zu",
                group_name_.c_str(), initialized_, validPose(target_pose), seed.size(), q_indices_.size());
    error.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE;
    return false;
  }
  const bool has_consistency_limits = consistency && !consistency->empty();
  if (has_consistency_limits && consistency->size() != seed.size())
  {
    RCLCPP_WARN(node_->get_logger(), "Pinocchio IK consistency size mismatch for %s: got=%zu expected=%zu",
                group_name_.c_str(), consistency->size(), seed.size());
    return false;
  }
  std::lock_guard<std::mutex> lock(solve_mutex_);
  Eigen::VectorXd q;
  if (!makeConfiguration(seed, q))
  {
    RCLCPP_WARN(node_->get_logger(), "Pinocchio IK seed is non-finite or outside limits for %s",
                group_name_.c_str());
    error.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE;
    return false;
  }
  Eigen::Quaterniond target_q(target_pose.orientation.w, target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z);
  target_q.normalize();
  const Eigen::Vector3d target_p(target_pose.position.x, target_pose.position.y, target_pose.position.z);
  const auto started = std::chrono::steady_clock::now();
  const double budget = timeout > 0.0 ? timeout : getDefaultTimeout();
  pinocchio::Data data(model_);
  double final_position_error = std::numeric_limits<double>::infinity();
  double final_orientation_error = std::numeric_limits<double>::infinity();
  int iterations = 0;
  bool timed_out = false;
  for (; iterations < max_iterations_; ++iterations)
  {
    if (std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count() > budget)
    {
      timed_out = true;
      break;
    }
    pinocchio::forwardKinematics(model_, data, q);
    pinocchio::updateFramePlacements(model_, data);
    const pinocchio::SE3 current = data.oMf[base_frame_id_].actInv(data.oMf[tip_frame_id_]);
    const Eigen::Vector3d position_error = target_p - current.translation();
    const Eigen::AngleAxisd angle_axis(target_q * Eigen::Quaterniond(current.rotation()).conjugate());
    const Eigen::Vector3d rotation_error = angle_axis.axis() * angle_axis.angle();
    final_position_error = position_error.norm();
    final_orientation_error = rotation_error.norm();
    if (final_position_error <= position_threshold_ && final_orientation_error <= orientation_threshold_)
    {
      solution.resize(q_indices_.size());
      for (size_t i = 0; i < q_indices_.size(); ++i) solution[i] = q[q_indices_[i]];
      if (callback)
      {
        error.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
        callback(target_pose, solution, error);
        if (error.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
        {
          RCLCPP_WARN(node_->get_logger(), "Pinocchio IK candidate rejected by MoveIt callback for %s: code=%d",
                      group_name_.c_str(), error.val);
          return false;
        }
      }
      error.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
      return true;
    }
    pinocchio::computeJointJacobians(model_, data, q);
    Eigen::Matrix<double, 6, Eigen::Dynamic> full_jacobian(6, model_.nv);
    pinocchio::getFrameJacobian(model_, data, tip_frame_id_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, full_jacobian);
    Eigen::MatrixXd jacobian(6, q_indices_.size());
    const Eigen::Matrix3d world_to_base = data.oMf[base_frame_id_].rotation().transpose();
    for (size_t i = 0; i < q_indices_.size(); ++i)
    {
      // Pinocchio Motion/Jacobian convention is [linear; angular].
      jacobian.block<3, 1>(0, i) = world_to_base * full_jacobian.block<3, 1>(0, v_indices_[i]);
      jacobian.block<3, 1>(3, i) = world_to_base * full_jacobian.block<3, 1>(3, v_indices_[i]);
    }
    Eigen::Matrix<double, 6, 1> residual;
    residual << position_error, rotation_error;
    Eigen::VectorXd delta = jacobian.transpose() * (jacobian * jacobian.transpose() + damping_ * Eigen::Matrix<double, 6, 6>::Identity()).ldlt().solve(residual);
    const double largest = delta.cwiseAbs().maxCoeff();
    if (largest > max_step_) delta *= max_step_ / largest;
    for (size_t i = 0; i < q_indices_.size(); ++i)
    {
      double lower = group_lower_[i], upper = group_upper_[i];
      if (has_consistency_limits)
      {
        lower = std::max(lower, seed[i] - (*consistency)[i]);
        upper = std::min(upper, seed[i] + (*consistency)[i]);
      }
      if (lower > upper)
      {
        RCLCPP_WARN(node_->get_logger(), "Pinocchio IK has empty bounded interval for %s joint %s",
                    group_name_.c_str(), joint_names_[i].c_str());
        return false;
      }
      q[q_indices_[i]] = std::clamp(q[q_indices_[i]] + delta[i], lower, upper);
    }
  }
  RCLCPP_WARN(node_->get_logger(),
              "Pinocchio IK failed for %s: iterations=%d timed_out=%d position_error=%.6f m "
              "orientation_error=%.6f rad thresholds=(%.6f, %.6f) target=(%.6f, %.6f, %.6f)",
              group_name_.c_str(), iterations, timed_out, final_position_error, final_orientation_error,
              position_threshold_, orientation_threshold_, target_p.x(), target_p.y(), target_p.z());
  return false;
}

bool PinocchioIKPlugin::getPositionIK(const geometry_msgs::msg::Pose& p, const std::vector<double>& s, std::vector<double>& q,
                                       moveit_msgs::msg::MoveItErrorCodes& e, const kinematics::KinematicsQueryOptions&) const
{ return solve(p, s, getDefaultTimeout(), nullptr, IKCallbackFn(), q, e); }
bool PinocchioIKPlugin::searchPositionIK(const geometry_msgs::msg::Pose& p, const std::vector<double>& s, double t, std::vector<double>& q, moveit_msgs::msg::MoveItErrorCodes& e, const kinematics::KinematicsQueryOptions&) const
{ return solve(p, s, t, nullptr, IKCallbackFn(), q, e); }
bool PinocchioIKPlugin::searchPositionIK(const geometry_msgs::msg::Pose& p, const std::vector<double>& s, double t, const std::vector<double>& c, std::vector<double>& q, moveit_msgs::msg::MoveItErrorCodes& e, const kinematics::KinematicsQueryOptions&) const
{ return solve(p, s, t, &c, IKCallbackFn(), q, e); }
bool PinocchioIKPlugin::searchPositionIK(const geometry_msgs::msg::Pose& p, const std::vector<double>& s, double t, std::vector<double>& q, const IKCallbackFn& cb, moveit_msgs::msg::MoveItErrorCodes& e, const kinematics::KinematicsQueryOptions&) const
{ return solve(p, s, t, nullptr, cb, q, e); }
bool PinocchioIKPlugin::searchPositionIK(const geometry_msgs::msg::Pose& p, const std::vector<double>& s, double t, const std::vector<double>& c, std::vector<double>& q, const IKCallbackFn& cb, moveit_msgs::msg::MoveItErrorCodes& e, const kinematics::KinematicsQueryOptions&) const
{ return solve(p, s, t, &c, cb, q, e); }
bool PinocchioIKPlugin::getPositionFK(const std::vector<std::string>& links, const std::vector<double>& angles, std::vector<geometry_msgs::msg::Pose>& poses) const
{
  Eigen::VectorXd q; if (!initialized_ || !makeConfiguration(angles, q)) return false;
  poses.clear(); for (const auto& link : links) { geometry_msgs::msg::Pose pose; if (!poseFor(q, link, pose)) return false; poses.push_back(pose); } return true;
}
}  // namespace qi_robot_pinocchio_ik

PLUGINLIB_EXPORT_CLASS(qi_robot_pinocchio_ik::PinocchioIKPlugin, kinematics::KinematicsBase)
