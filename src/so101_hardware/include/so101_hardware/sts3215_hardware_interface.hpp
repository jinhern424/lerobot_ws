#ifndef SO101_HARDWARE__STS3215_HARDWARE_INTERFACE_HPP_
#define SO101_HARDWARE__STS3215_HARDWARE_INTERFACE_HPP_
#include "so101_hardware/SCServo.h"
#include "so101_hardware/SMS_STS.h"

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

// IMPORTANT: Include your specific Feetech SDK header here.
// Depending on the SDK version you downloaded, it might be called "SCServo.h", 
// "SMS_STS.h", or something similar. 


namespace so101_hardware
{

class STS3215HardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(STS3215HardwareInterface)

  // -----------------------------------------------------------------------
  // Core ROS 2 Control Lifecycle Methods
  // -----------------------------------------------------------------------
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Logger for console output
  rclcpp::Logger logger_ = rclcpp::get_logger("STS3215HardwareInterface");

  // -----------------------------------------------------------------------
  // Hardware Communication Object
  // -----------------------------------------------------------------------
  // You need an instance of the Feetech serial object here. 
  // "SMS_STS" is a common class name in the Feetech SDK for STS servos. 
  // Adjust this type based on the exact SDK you are using.
  std::shared_ptr<SMS_STS> servo_; 

  // -----------------------------------------------------------------------
  // State and Command Buffers (What MoveIt reads and writes to)
  // -----------------------------------------------------------------------
  std::vector<double> hw_cmd_position_;
  std::vector<double> hw_state_position_;
  std::vector<double> hw_state_velocity_;

  // -----------------------------------------------------------------------
  // Motor Grouping & SyncWrite Buffers
  // -----------------------------------------------------------------------
  std::vector<int> motor_ids_;               // IDs pulled from URDF
  std::vector<uint8_t> servo_sync_ids_;      // IDs formatted for SyncWrite

  // Data arrays formatted for the Feetech SyncWritePos function
  std::vector<int16_t> servo_sync_positions_;       
  std::vector<uint16_t> servo_sync_speeds_;     
  std::vector<uint8_t> servo_sync_accelerations_; 
};

}  // namespace so101_hardware

#endif  // SO101_HARDWARE__STS3215_HARDWARE_INTERFACE_HPP_