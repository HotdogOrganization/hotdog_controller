#ifndef MSG_CONVERSION_HPP_
#define MSG_CONVERSION_HPP_

#include <vector>
#include <algorithm>

#include <sensor_msgs/msg/imu.hpp>
#include "sopu_msgs/msg/motor_command.hpp"
#include "sopu_msgs/msg/motor_status.hpp"
#include "sim_utilities/spine_board.hpp"
#include "sim_utilities/imu_types.hpp"
#include "sim_utilities/visualization_data.hpp"
#include "command_interface/command_interface.hpp"

namespace hotdog_locomotion
{
namespace msg_conversion
{

static inline VectorNavData ConvertToVectorNavData(const float* quat, const float* gyro, const float* accl)
{
  VectorNavData vector_nav_data;
  vector_nav_data.quat[0] = quat[1];
  vector_nav_data.quat[1] = quat[2];
  vector_nav_data.quat[2] = quat[3];
  vector_nav_data.quat[3] = quat[0];

  vector_nav_data.gyro[0] = gyro[0];
  vector_nav_data.gyro[1] = gyro[1];
  vector_nav_data.gyro[2] = gyro[2];

  vector_nav_data.accelerometer[0] = accl[0];
  vector_nav_data.accelerometer[1] = accl[1];
  vector_nav_data.accelerometer[2] = accl[2];

  return vector_nav_data;
}

static inline VectorNavData ConvertToVectorNavData(const sensor_msgs::msg::Imu& imu_msg) {
  VectorNavData vector_nav_data;
  vector_nav_data.quat[0] = imu_msg.orientation.x;
  vector_nav_data.quat[1] = imu_msg.orientation.y;
  vector_nav_data.quat[2] = imu_msg.orientation.z;
  vector_nav_data.quat[3] = imu_msg.orientation.w;

  vector_nav_data.gyro[0] = imu_msg.angular_velocity.x;
  vector_nav_data.gyro[1] = imu_msg.angular_velocity.y;
  vector_nav_data.gyro[2] = imu_msg.angular_velocity.z;

  vector_nav_data.accelerometer[0] = imu_msg.linear_acceleration.x;
  vector_nav_data.accelerometer[1] = imu_msg.linear_acceleration.y;
  vector_nav_data.accelerometer[2] = imu_msg.linear_acceleration.z;

  return vector_nav_data;
}

static inline sopu_msgs::msg::MotorCommand ConvertToMotorCommandMsg(const SpiCommand& spi_command) {
  sopu_msgs::msg::MotorCommand motor_command;
  for (int i = 0; i < 4; ++i) {
    motor_command.q_des_abad[i] = spi_command.q_des_abad[i];
    motor_command.q_des_hip[i] = spi_command.q_des_hip[i];
    motor_command.q_des_knee[i] = spi_command.q_des_knee[i];
    motor_command.kp_abad[i] = spi_command.kp_abad[i];
    motor_command.kp_hip[i] = spi_command.kp_hip[i];
    motor_command.kp_knee[i] = spi_command.kp_knee[i];
    motor_command.kd_abad[i] = spi_command.kd_abad[i];
    motor_command.kd_hip[i] = spi_command.kd_hip[i];
    motor_command.kd_knee[i] = spi_command.kd_knee[i];
    motor_command.tau_abad_ff[i] = spi_command.tau_abad_ff[i];
    motor_command.tau_hip_ff[i] = spi_command.tau_hip_ff[i];
    motor_command.tau_knee_ff[i] = spi_command.tau_knee_ff[i];
  }
  return motor_command;
}

static inline sopu_msgs::msg::MotorStatus ConvertToMotorStatusMsg(const SpiData& spi_data) {
  sopu_msgs::msg::MotorStatus motor_status;
  for (int i = 0; i < 4; ++i) {
    motor_status.q_abad[i] = spi_data.q_abad[i];
    motor_status.q_hip[i] = spi_data.q_hip[i];
    motor_status.q_knee[i] = spi_data.q_knee[i];
    motor_status.qd_abad[i] = spi_data.qd_abad[i];
    motor_status.qd_hip[i] = spi_data.qd_hip[i];
    motor_status.qd_knee[i] = spi_data.qd_knee[i];
    motor_status.tau_abad[i] = spi_data.tau_abad[i];
    motor_status.tau_hip[i] = spi_data.tau_hip[i];
    motor_status.tau_knee[i] = spi_data.tau_knee[i];
  }
  // std::copy(std::begin(spi_data.flags), std::end(spi_data.flags), std::begin(motor_status.flags));
  return motor_status;
}

static inline SpiCommand ConvertToSpiCommand(const sopu_msgs::msg::MotorCommand& motor_command) {
  SpiCommand spi_command;
  for (int i = 0; i < 4; ++i) {
    spi_command.q_des_abad[i] = motor_command.q_des_abad[i];
    spi_command.q_des_hip[i] = motor_command.q_des_hip[i];
    spi_command.q_des_knee[i] = motor_command.q_des_knee[i];
    spi_command.kp_abad[i] = motor_command.kp_abad[i];
    spi_command.kp_hip[i] = motor_command.kp_hip[i];
    spi_command.kp_knee[i] = motor_command.kp_knee[i];
    spi_command.kd_abad[i] = motor_command.kd_abad[i];
    spi_command.kd_hip[i] = motor_command.kd_hip[i];
    spi_command.kd_knee[i] = motor_command.kd_knee[i];
    spi_command.tau_abad_ff[i] = motor_command.tau_abad_ff[i];
    spi_command.tau_hip_ff[i] = motor_command.tau_hip_ff[i];
    spi_command.tau_knee_ff[i] = motor_command.tau_knee_ff[i];
  }
  return spi_command;
}

static inline SpiData ConvertToSpiData(const float* motor_pos, const float* motor_vel, const float* motor_tor)
{
  SpiData spi_data;
  for (int i = 0; i < 4; ++i) {
    spi_data.q_abad[i] = motor_pos[i * 3 + 0];
    spi_data.q_hip[i] = -motor_pos[i * 3 + 1];
    spi_data.q_knee[i] = -motor_pos[i * 3 + 2];
    spi_data.qd_abad[i] = motor_vel[i * 3 + 0];
    spi_data.qd_hip[i] = -motor_vel[i * 3 + 1];
    spi_data.qd_knee[i] = -motor_vel[i * 3 + 2];
    spi_data.tau_abad[i] = motor_tor[i * 3 + 0];
    spi_data.tau_hip[i] = motor_tor[i * 3 + 1];
    spi_data.tau_knee[i] = motor_tor[i * 3 + 2];
  }
  return spi_data;
}

static inline SpiData ConvertToSpiData(const sopu_msgs::msg::MotorStatus& motor_status)
{
  SpiData spi_data;
  for (int i = 0; i < 4; ++i) {
    spi_data.q_abad[i] = motor_status.q_abad[i];
    spi_data.q_hip[i] = motor_status.q_hip[i];
    spi_data.q_knee[i] = motor_status.q_knee[i];
    spi_data.qd_abad[i] = motor_status.qd_abad[i];
    spi_data.qd_hip[i] = motor_status.qd_hip[i];
    spi_data.qd_knee[i] = motor_status.qd_knee[i];
    spi_data.tau_abad[i] = motor_status.tau_abad[i];
    spi_data.tau_hip[i] = motor_status.tau_hip[i];
    spi_data.tau_knee[i] = motor_status.tau_knee[i];
  }

  // TODO: flagso需要赋值

  return spi_data;
}

}  // namespace msg_conversion

}  // namespace hotdog_locomotion

#endif  // MSG_CONVERSION_HPP_
