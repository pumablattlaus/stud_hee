#include "stud_hee/my_cartesian_velocity_controller.h"


#include <cmath>
#include <iostream>
#include <franka/exception.h>
#include <franka/robot.h>
#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <pluginlib/class_list_macros.h>

#include "stud_hee/MyVelocity.h"

namespace my_panda_controllers {

template<class T,size_t N>
std::array<T,N> convert( const boost::array<T, N> & v )
{
    //assert(v.size() == N);
    std::array<T,N> r;
    std::copy( v.begin(), v.end(), r.begin() );
    return r;
}

bool CartesianVelocityMyController::init(hardware_interface::RobotHW* robot_hardware,
                                              ros::NodeHandle& node_handle) {
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("CartesianVelocityMyController: Could not get parameter arm_id");
    return false;
  }

  velocity_cartesian_interface_ =
      robot_hardware->get<franka_hw::FrankaVelocityCartesianInterface>();
  if (velocity_cartesian_interface_ == nullptr) {
    ROS_ERROR(
        "CartesianVelocityMyController: Could not get Cartesian velocity interface from "
        "hardware");
    return false;
  }
  try {
    velocity_cartesian_handle_ = std::make_unique<franka_hw::FrankaCartesianVelocityHandle>(
        velocity_cartesian_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianVelocityMyController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("CartesianVelocityMyController: Could not get state interface from hardware");
    return false;
  }

  sub_vel = node_handle.subscribe("my_velocities", 20, &CartesianVelocityMyController::velocity_callback, this, ros::TransportHints().reliable().tcpNoDelay());

  return true;
}

void CartesianVelocityMyController::starting(const ros::Time& /* time */) {
  elapsed_time_ = ros::Duration(0.0);
}

void CartesianVelocityMyController::update(const ros::Time& /* time */,
                                                const ros::Duration& period) {
  
  velocity_cartesian_handle_->setCommand(this->cart_velocities);
}

void CartesianVelocityMyController::velocity_callback(stud_hee::MyVelocity cartesian_velocities)
{
    
    this->cart_velocities = convert(cartesian_velocities.vel); 
    velocity_cartesian_handle_->setCommand(this->cart_velocities);
}

void CartesianVelocityMyController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

}  // namespace my_panda_controllers

PLUGINLIB_EXPORT_CLASS(my_panda_controllers::CartesianVelocityMyController,
                       controller_interface::ControllerBase)

