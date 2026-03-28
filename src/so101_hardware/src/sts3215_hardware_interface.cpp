#include "so101_hardware/sts3215_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace so101_hardware
{

// ------------------------------------------------------------------------------------------
// HELPER FUNCTIONS & CALIBRATION
// ------------------------------------------------------------------------------------------
static constexpr double STEPS_PER_REVOLUTION = 4096.0;
static constexpr double STEPS_TO_RAD = (2.0 * M_PI) / STEPS_PER_REVOLUTION;
static constexpr double RAD_TO_STEPS = STEPS_PER_REVOLUTION / (2.0 * M_PI);

// Your custom calibration offsets!
static const int JOINT_OFFSETS[6] = {2046, 2050, 2100, 2074, 2962, 1993};

double raw_position_to_radians(int raw_position, int joint_index) {
  return static_cast<double>(raw_position - JOINT_OFFSETS[joint_index]) * STEPS_TO_RAD;
}

int16_t radians_to_raw_position(double radians, int joint_index) {
  return static_cast<int16_t>((radians * RAD_TO_STEPS) + JOINT_OFFSETS[joint_index]);
}

// ------------------------------------------------------------------------------------------
// LIFECYCLE: ON_INIT
// ------------------------------------------------------------------------------------------
hardware_interface::CallbackReturn STS3215HardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  logger_ = rclcpp::get_logger("STS3215HardwareInterface");
  servo_ = std::make_shared<SMS_STS>();
  servo_->begin(1000000, "/dev/ttyACM0");
  
  size_t num_joints = info_.joints.size();
  hw_state_position_.resize(num_joints, std::numeric_limits<double>::quiet_NaN());
  hw_state_velocity_.resize(num_joints, std::numeric_limits<double>::quiet_NaN());
  hw_cmd_position_.resize(num_joints, std::numeric_limits<double>::quiet_NaN());
  
  motor_ids_.resize(num_joints);

  for (size_t i = 0; i < num_joints; ++i) {
    const auto & joint = info_.joints[i];

    if (joint.parameters.count("motor_id")) {
      motor_ids_[i] = std::stoi(joint.parameters.at("motor_id"));
      servo_sync_ids_.push_back(static_cast<uint8_t>(motor_ids_[i]));
    } else {
      RCLCPP_ERROR(logger_, "Joint '%s' is missing 'motor_id' parameter in URDF.", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  servo_sync_positions_.resize(num_joints);
  servo_sync_speeds_.resize(num_joints);
  servo_sync_accelerations_.resize(num_joints);

  RCLCPP_INFO(logger_, "Successfully initialized %zu STS3215 motors.", num_joints);
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ------------------------------------------------------------------------------------------
// INTERFACE REGISTRATION
// ------------------------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface> STS3215HardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_state_position_[i]));
    
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_state_velocity_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> STS3215HardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_cmd_position_[i]));
  }
  return command_interfaces;
}

// ------------------------------------------------------------------------------------------
// MAIN LOOP: READ
// ------------------------------------------------------------------------------------------
hardware_interface::return_type STS3215HardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (size_t i = 0; i < motor_ids_.size(); i++) {
    // 1. Read the raw position from the motor
    int raw_pos = servo_->ReadPos(motor_ids_[i]);
    // TEMPORARY CALIBRATION PRINT
    // static size_t print_counter = 0;
    // if (print_counter++ % 600 == i) { 
    //     RCLCPP_INFO(logger_, "CALIBRATION -> Motor ID %d is at Raw Pos: %d", motor_ids_[i], raw_pos);
    // }
    // 2. Safety check: Ensure the motor actually responded
    if (servo_->FeedBack(motor_ids_[i]) != 1) {
      RCLCPP_WARN(logger_, "Failed to read from motor ID: %d", motor_ids_[i]);
      continue; 
    }
    
    // 3. Convert the raw position to radians using your specific calibration offsets
    hw_state_position_[i] = raw_position_to_radians(raw_pos, i);
  }

  return hardware_interface::return_type::OK;
}

// ------------------------------------------------------------------------------------------
// MAIN LOOP: WRITE
// ------------------------------------------------------------------------------------------
hardware_interface::return_type STS3215HardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (servo_sync_ids_.empty()) {
    return hardware_interface::return_type::OK;
  }

  for (size_t j = 0; j < servo_sync_ids_.size(); ++j) {
    double target_position_rad = hw_cmd_position_[j]; 

    if (std::isnan(target_position_rad)) {
      target_position_rad = hw_state_position_[j]; 
    }

    // Convert target radians to raw motor steps using your specific calibration offsets
    servo_sync_positions_[j] = radians_to_raw_position(target_position_rad, j);

    servo_sync_speeds_[j] = 2000;         
    servo_sync_accelerations_[j] = 50;    
  }

  servo_->SyncWritePosEx(
    servo_sync_ids_.data(), 
    servo_sync_ids_.size(),
    servo_sync_positions_.data(), 
    servo_sync_speeds_.data(), 
    servo_sync_accelerations_.data()
  );

  return hardware_interface::return_type::OK;
}

}  // namespace so101_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  so101_hardware::STS3215HardwareInterface, hardware_interface::SystemInterface)