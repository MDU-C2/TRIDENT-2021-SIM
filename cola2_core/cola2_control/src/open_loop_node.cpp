/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <ros/ros.h>
#include <cola2_control/low_level_controllers/only_thruster_allocator.h>
#include <cola2_control/low_level_controllers/poly.h>
#include <cola2_lib_ros/this_node.h>
#include <cola2_lib_ros/param_loader.h>
#include <cola2_msgs/Setpoints.h>
#include <sensor_msgs/Joy.h>
#include <eigen3/Eigen/Dense>

class ThrusterAllocatorWrapper : public OnlyThrusterAllocator
{  // We need a new wrapper class because the methods we want to access are protected
 public:
  ThrusterAllocatorWrapper(unsigned int n_thrusters)
    : OnlyThrusterAllocator(n_thrusters)
  {
  }

  std::vector<double> newWrenchToThrusterForces(const std::vector<double>& input)
  {
    Eigen::VectorXd wrench(input.size());
    for (std::size_t i = 0; i < input.size(); ++i)
      wrench(i) = input[i];
    Eigen::VectorXd thruster_forces = wrenchToThrusterForces(wrench);
    std::vector<double> output(thruster_forces.size());
    for (std::size_t i = 0; i < static_cast<std::size_t>(thruster_forces.size()); ++i)
      output[i] = thruster_forces(i);
    return output;
  }

  std::vector<double> newForceToSetpoint(const std::vector<double>& input)
  {
    Eigen::VectorXd thruster_forces(input.size());
    for (std::size_t i = 0; i < input.size(); ++i)
      thruster_forces(i) = input[i];
    Eigen::VectorXd setpoint = forceToSetpoint(thruster_forces);
    std::vector<double> output(setpoint.size());
    for (std::size_t i = 0; i < static_cast<std::size_t>(setpoint.size()); ++i)
      output[i] = setpoint(i);
    return output;
  }
};

class OpenLoop
{
 protected:
  static constexpr double MAX_SURGE_VEL = 1.0;
  static constexpr double MAX_SWAY_VEL = 0.4;
  static constexpr double MAX_HEAVE_VEL = 0.4;
  static constexpr double MAX_YAW_RATE = 0.3;
  ros::NodeHandle nh_;
  const std::string ns_;
  ros::Timer stop_timer_;
  std::size_t setpoints_sent_;
  ros::Publisher pub_thrusters_;
  ros::Subscriber sub_joy_;
  std::shared_ptr<ThrusterAllocatorWrapper> thruster_allocator_ptr_;
  double max_wrench_surge_, max_wrench_sway_, max_wrench_heave_, max_wrench_yaw_;
  Poly poly_surge_, poly_sway_, poly_heave_, poly_yaw_;

 public:
  OpenLoop();
  void stopCallback(const ros::TimerEvent&);
  void joyCallback(const sensor_msgs::Joy&);
};

OpenLoop::OpenLoop()
  : nh_("~"), ns_(cola2::ros::getNamespace()), setpoints_sent_(0), poly_surge_("poly_surge"), poly_sway_("poly_sway"), poly_heave_("poly_heave"), poly_yaw_("poly_yaw")
{
  // Get config
  bool ok = true;
  int n_thrusters = 0;
  ok &= cola2::ros::getParam(ns_ + "/controller/n_thrusters", n_thrusters);
  std::vector<std::vector<double> > poly_positive_v, poly_negative_v;
  std::vector<double> max_force_thruster_positive_v, max_force_thruster_negative_v;
  for (int i = 0; i < n_thrusters; ++i)
  {
    std::vector<double> thruster_poly_positive, thruster_poly_negative;
    ok &= cola2::ros::getParamVector(ns_ + "/controller/thruster_" + std::to_string(i + 1) + "_poly_positive", thruster_poly_positive);
    ok &= cola2::ros::getParamVector(ns_ + "/controller/thruster_" + std::to_string(i + 1) + "_poly_negative", thruster_poly_negative);
    poly_positive_v.push_back(thruster_poly_positive);
    poly_negative_v.push_back(thruster_poly_negative);

    double max_force_positive, max_force_negative;
    ok &= cola2::ros::getParam(ns_ + "/controller/thruster_" + std::to_string(i + 1) + "_max_force_positive", max_force_positive);
    ok &= cola2::ros::getParam(ns_ + "/controller/thruster_" + std::to_string(i + 1) + "_max_force_negative", max_force_negative);
    max_force_thruster_positive_v.push_back(max_force_positive);
    max_force_thruster_negative_v.push_back(max_force_negative);
  }
  std::vector<double> tcm;
  ok &= cola2::ros::getParamVector(ns_ + "/controller/TCM", tcm);
  ok &= cola2::ros::getParam(ns_ + "/controller/max_wrench_X", max_wrench_surge_);
  ok &= cola2::ros::getParam(ns_ + "/controller/max_wrench_Y", max_wrench_sway_);
  ok &= cola2::ros::getParam(ns_ + "/controller/max_wrench_Z", max_wrench_heave_);
  ok &= cola2::ros::getParam(ns_ + "/controller/max_wrench_Yaw", max_wrench_yaw_);

  double poly_surge_a, poly_surge_b, poly_surge_c;
  ok &= cola2::ros::getParam(ns_ + "/controller/poly_surge_A", poly_surge_a);
  ok &= cola2::ros::getParam(ns_ + "/controller/poly_surge_B", poly_surge_b);
  ok &= cola2::ros::getParam(ns_ + "/controller/poly_surge_C", poly_surge_c);
  std::map<std::string, double> poly_surge_params;
  poly_surge_params.insert(std::pair<std::string, double>("n_dof", 3.0));
  poly_surge_params.insert(std::pair<std::string, double>("0", poly_surge_a));
  poly_surge_params.insert(std::pair<std::string, double>("1", poly_surge_b));
  poly_surge_params.insert(std::pair<std::string, double>("2", poly_surge_c));
  poly_surge_.setParameters(poly_surge_params);

  double poly_sway_a, poly_sway_b, poly_sway_c;
  ok &= cola2::ros::getParam(ns_ + "/controller/poly_sway_A", poly_sway_a);
  ok &= cola2::ros::getParam(ns_ + "/controller/poly_sway_B", poly_sway_b);
  ok &= cola2::ros::getParam(ns_ + "/controller/poly_sway_C", poly_sway_c);
  std::map<std::string, double> poly_sway_params;
  poly_sway_params.insert(std::pair<std::string, double>("n_dof", 3.0));
  poly_sway_params.insert(std::pair<std::string, double>("0", poly_sway_a));
  poly_sway_params.insert(std::pair<std::string, double>("1", poly_sway_b));
  poly_sway_params.insert(std::pair<std::string, double>("2", poly_sway_c));
  poly_sway_.setParameters(poly_sway_params);

  double poly_heave_a, poly_heave_b, poly_heave_c;
  ok &= cola2::ros::getParam(ns_ + "/controller/poly_heave_A", poly_heave_a);
  ok &= cola2::ros::getParam(ns_ + "/controller/poly_heave_B", poly_heave_b);
  ok &= cola2::ros::getParam(ns_ + "/controller/poly_heave_C", poly_heave_c);
  std::map<std::string, double> poly_heave_params;
  poly_heave_params.insert(std::pair<std::string, double>("n_dof", 3.0));
  poly_heave_params.insert(std::pair<std::string, double>("0", poly_heave_a));
  poly_heave_params.insert(std::pair<std::string, double>("1", poly_heave_b));
  poly_heave_params.insert(std::pair<std::string, double>("2", poly_heave_c));
  poly_heave_.setParameters(poly_heave_params);

  double poly_yaw_a, poly_yaw_b, poly_yaw_c;
  ok &= cola2::ros::getParam(ns_ + "/controller/poly_yaw_A", poly_yaw_a);
  ok &= cola2::ros::getParam(ns_ + "/controller/poly_yaw_B", poly_yaw_b);
  ok &= cola2::ros::getParam(ns_ + "/controller/poly_yaw_C", poly_yaw_c);
  std::map<std::string, double> poly_yaw_params;
  poly_yaw_params.insert(std::pair<std::string, double>("n_dof", 3.0));
  poly_yaw_params.insert(std::pair<std::string, double>("0", poly_yaw_a));
  poly_yaw_params.insert(std::pair<std::string, double>("1", poly_yaw_b));
  poly_yaw_params.insert(std::pair<std::string, double>("2", poly_yaw_c));
  poly_yaw_.setParameters(poly_yaw_params);

  // Check whether all parameters have been properly loaded or not
  if (!ok)
  {
    ROS_FATAL("Invalid parametes in ROS param server. Shutting down");
    ros::shutdown();
  }

  // Create thruster allocator
  thruster_allocator_ptr_ = std::make_shared<ThrusterAllocatorWrapper>(n_thrusters);
  thruster_allocator_ptr_->setParams(max_force_thruster_positive_v, max_force_thruster_negative_v, poly_positive_v, poly_negative_v, tcm);

  // Create timer to stop the node after some safety time
  stop_timer_ = nh_.createTimer(ros::Duration(60.0), &OpenLoop::stopCallback, this, true);

  // Create publisher
  pub_thrusters_ = nh_.advertise<cola2_msgs::Setpoints>(ns_ + "/controller/thruster_setpoints", 1);

  // Create subscriber
  sub_joy_ = nh_.subscribe(ns_ + "/input_to_teleoperation/output", 10, &OpenLoop::joyCallback, this);

  ROS_INFO("Initialized");
}

void
OpenLoop::stopCallback(const ros::TimerEvent&)
{
  ROS_INFO("Stopping. Restart if still required");
  ros::shutdown();
  exit(0);
}

void
OpenLoop::joyCallback(const sensor_msgs::Joy& msg)
{
  // Check axes length
  if (msg.axes.size() != 12)
  {
    ROS_ERROR("Invalid msg.axes length");
    return;
  }

  // Compute desired velocities
  const double desired_surge_vel = msg.axes[6] * MAX_SURGE_VEL;
  const double desired_sway_vel  = msg.axes[7] * MAX_SWAY_VEL;
  const double desired_heave_vel = msg.axes[8] * MAX_HEAVE_VEL;
  const double desired_yaw_rate  = msg.axes[11] * MAX_YAW_RATE;

  // Compute desired forces using the open loop poly models
  const double desired_surge_force = poly_surge_.compute(0.0, desired_surge_vel, 0.0);
  const double desired_sway_force  = poly_sway_.compute(0.0, desired_sway_vel, 0.0);
  const double desired_heave_force = poly_heave_.compute(0.0, desired_heave_vel, 0.0);
  const double desired_yaw_torque   = poly_yaw_.compute(0.0, desired_yaw_rate, 0.0);

  // Compute thruster forces
  const std::vector<double> wrench = {desired_surge_force,
                                      desired_sway_force,
                                      desired_heave_force,
                                      0.0,
                                      0.0,
                                      desired_yaw_torque};
  const std::vector<double> thruster_forces = thruster_allocator_ptr_->newWrenchToThrusterForces(wrench);

  // Compute thruster setpoints
  const std::vector<double> setpoints = thruster_allocator_ptr_->newForceToSetpoint(thruster_forces);

  // Publish setpoints
  cola2_msgs::Setpoints msg_setpoints;
  msg_setpoints.header.stamp = ros::Time::now();  // Safer than using joy stamp
  msg_setpoints.header.frame_id = "open_loop";
  msg_setpoints.setpoints = setpoints;
  pub_thrusters_.publish(msg_setpoints);

  // Put also a limit on the amount of setpoints sent (redundancy with the stop timer)
  if (++setpoints_sent_ > 600)
  {
    ros::TimerEvent event;
    stopCallback(event);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "open_loop");
  OpenLoop open_loop;
  ros::spin();
  return 0;
}
