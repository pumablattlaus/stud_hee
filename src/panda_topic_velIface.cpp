#include <cmath>
#include <iostream>
#include <franka/exception.h>
#include <franka/robot.h>
#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <pluginlib/class_list_macros.h>

namespace my_controller {

class MyVelocityController : public controller_interface::MultiInterfaceController<
                                               franka_hw::FrankaVelocityCartesianInterface,
                                               franka_hw::FrankaStateInterface> {
 public:
  std::array<double, 6> cart_velocities;
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void starting(const ros::Time&) override;
  void stopping(const ros::Time&) override;

  private:
    franka_hw::FrankaVelocityCartesianInterface* velocity_cartesian_interface_;
    std::unique_ptr<franka_hw::FrankaCartesianVelocityHandle> velocity_cartesian_handle_;
    ros::Duration elapsed_time_;

  

  void velocity_callback(std::array<double, 6> &cartesian_velocities)
  {
      this->cart_velocities = cartesian_velocities; 
      velocity_cartesian_handle_->setCommand(cartesian_velocities);
  }

};

bool MyVelocityController::init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle)
  {
    // std::string arm_id;
    // if (!node_handle.getParam("arm_id", arm_id)) {
    //   ROS_ERROR("CartesianVelocityController: Could not get parameter arm_id");
    //   return false;
    // }
    std::string arm_id = "panda";

    velocity_cartesian_interface_ =
        robot_hardware->get<franka_hw::FrankaVelocityCartesianInterface>();
    if (velocity_cartesian_interface_ == nullptr) {
      ROS_ERROR(
          "CartesianVelocityController: Could not get Cartesian velocity interface from "
          "hardware");
      return false;
    }
    try {
      velocity_cartesian_handle_ = std::make_unique<franka_hw::FrankaCartesianVelocityHandle>(
          velocity_cartesian_interface_->getHandle(arm_id + "_robot"));
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "CartesianVelocityController: Exception getting Cartesian handle: " << e.what());
      return false;
    }

    auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr) {
      ROS_ERROR("CartesianVelocityController: Could not get state interface from hardware");
      return false;
    }

    ros::Subscriber sub_vel = node_handle.subscribe("my_velocities", 20, &MyVelocityController::velocity_callback, this);
    return true;
  }

  void MyVelocityController::starting(const ros::Time& /* time */) {
    elapsed_time_ = ros::Duration(0.0);
  }

  void MyVelocityController::update(const ros::Time& /* time */, const ros::Duration& period) {
    // elapsed_time_ += period;

    // double time_max = 4.0;
    // double v_max = 0.05;
    // double angle = M_PI / 4.0;
    // double cycle = std::floor(
    //     pow(-1.0, (elapsed_time_.toSec() - std::fmod(elapsed_time_.toSec(), time_max)) / time_max));
    // double v = cycle * v_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * elapsed_time_.toSec()));
    // double v_x = std::cos(angle) * v;
    // double v_z = -std::sin(angle) * v;
    // std::array<double, 6> command = {{v_x, 0.0, v_z, 0.0, 0.0, 0.0}};
    // velocity_cartesian_handle_->setCommand(command);
    velocity_cartesian_handle_->setCommand(cart_velocities);
  }
}

// namespace my_controller

PLUGINLIB_EXPORT_CLASS(my_controller::MyVelocityController,
                       controller_interface::ControllerBase)