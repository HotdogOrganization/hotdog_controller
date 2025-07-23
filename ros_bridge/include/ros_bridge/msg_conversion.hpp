#ifndef MSG_CONVERSION_HPP_
#define MSG_CONVERSION_HPP_

#include <vector>
#include <algorithm>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "visualization_msgs/msg/marker_array.hpp"
#include "sopu_msgs/msg/motor_command.hpp"
#include "sopu_msgs/msg/motor_status.hpp"
#include "sopu_msgs/msg/wbc_test_data.hpp"
#include "sim_utilities/spine_board.hpp"
#include "sim_utilities/imu_types.hpp"
#include "sim_utilities/visualization_data.hpp"
#include "sim_utilities/wbc_test_data.hpp"
#include "command_interface/command_interface.hpp"
#include "command_interface/gamepad_command.hpp"
#include "command_interface/keyboard_command.hpp"
#include "command_interface/rc_command.hpp"

namespace hotdog_locomotion
{

struct JoyData
{
  std::vector<float> axes;
  std::vector<int32_t> buttons;
};

namespace msg_conversion
{

static inline KeyboardCommand ConvertToKeyboardCommand(const JoyData& joy_data)
{
  KeyboardCommand keyboard_command;
  keyboard_command.zero();
  // Button 映射
  // 安全判断下标，防止越界
  if (joy_data.buttons.size() > 2) {
    keyboard_command.pureDumpButton      = (joy_data.buttons[0] != 0);
    keyboard_command.recoveryStandButton = (joy_data.buttons[1] != 0);
    keyboard_command.balanceStandButton  = (joy_data.buttons[2] != 0);
  }
  if (joy_data.buttons.size() > 3) {
    keyboard_command.locomotionButton    = (joy_data.buttons[3] != 0);
  }

  // Axes 映射
  if (joy_data.axes.size() > 0) keyboard_command.xVel      = joy_data.axes[0];
  if (joy_data.axes.size() > 1) keyboard_command.yVel      = joy_data.axes[1];
  if (joy_data.axes.size() > 2) keyboard_command.rollPos   = joy_data.axes[2];
  if (joy_data.axes.size() > 3) keyboard_command.pitchPos  = joy_data.axes[3];
  if (joy_data.axes.size() > 4) keyboard_command.yawPos    = joy_data.axes[4];
  if (joy_data.axes.size() > 5) keyboard_command.yawVel    = joy_data.axes[5];
  if (joy_data.axes.size() > 6) keyboard_command.heightPos = joy_data.axes[6];
  // std::cout << "Keyboard Command: " << keyboard_command.ToString() << std::endl;
  return keyboard_command;
}


static inline GamepadCommand ConvertToGamepadCommand(const sensor_msgs::msg::Joy& joy_msg)
{
  GamepadCommand gamepad_command;
  gamepad_command.leftBumper = joy_msg.buttons[6] > 0;  // LB
  gamepad_command.rightBumper = joy_msg.buttons[7] > 0;  // RB
  gamepad_command.leftTriggerButton = joy_msg.buttons[8] > 0;  // LT
  gamepad_command.rightTriggerButton = joy_msg.buttons[9] > 0;  // RT
  gamepad_command.back = joy_msg.buttons[10] > 0;  // Back/Select button
  gamepad_command.start = joy_msg.buttons[11] > 0;  // Start button
  gamepad_command.a = joy_msg.buttons[0] > 0;
  gamepad_command.b = joy_msg.buttons[1] > 0;
  gamepad_command.x = joy_msg.buttons[3] > 0;
  gamepad_command.y = joy_msg.buttons[4] > 0;
  gamepad_command.leftStickButton = joy_msg.buttons[13] > 0;
  gamepad_command.rightStickButton = joy_msg.buttons[14] > 0;
  gamepad_command.leftStickAnalog[0] = joy_msg.axes[0];
  gamepad_command.leftStickAnalog[1] = joy_msg.axes[1];
  gamepad_command.rightStickAnalog[0] = joy_msg.axes[2];
  gamepad_command.rightStickAnalog[1] = joy_msg.axes[3];
  // std::cout << "Gamepad Command: " << gamepad_command.ToString() << std::endl;

  return gamepad_command;
}


static inline RcCommand ConvertToRcCommand(const sensor_msgs::msg::Joy& joy_msg)
{
  RcCommand rc_command;
  rc_command.L1 = joy_msg.buttons[0] > 0;  // L1
  rc_command.L2 = joy_msg.buttons[1] > 0;  // L2
  rc_command.R1 = joy_msg.buttons[2] > 0;  // R1
  rc_command.R2 = joy_msg.buttons[3] > 0;  // R2
  rc_command.B1 = joy_msg.buttons[4] > 0;  // B1
  rc_command.B2 = joy_msg.buttons[5] > 0;  // B2
  rc_command.wheelAnalog = joy_msg.axes[4];  // Wheel analog
  rc_command.leftStickAnalog[0] = joy_msg.axes[3];
  rc_command.leftStickAnalog[1] = joy_msg.axes[2];
  rc_command.rightStickAnalog[0] = joy_msg.axes[0]; // 0是左右
  rc_command.rightStickAnalog[1] = joy_msg.axes[1]; // 1是上下
  // std::cout << "Rc Command: " << rc_command.ToString() << std::endl;

  return rc_command;
}

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

static inline VectorNavData ConvertToVectorNavData(const sensor_msgs::msg::Imu& imu_msg)
{
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

static inline sopu_msgs::msg::WbcTestData ConvertToWbcTestDataMsg(const WbcTestData& wbc_test_data)
{
  sopu_msgs::msg::WbcTestData msg;
  msg.header.stamp = rclcpp::Clock().now();
  msg.header.frame_id = "odom";
  // contact_est
  for (int i = 0; i < 4; ++i) {
    msg.contact_est[i] = wbc_test_data.contact_est[i];
  }

  // fr_des, fr
  for (int i = 0; i < 12; ++i) {
    msg.fr_des[i] = wbc_test_data.Fr_des[i];
    msg.fr[i] = wbc_test_data.Fr[i];
    msg.foot_pos_cmd[i] = wbc_test_data.foot_pos_cmd[i];
    msg.foot_vel_cmd[i] = wbc_test_data.foot_vel_cmd[i];
    msg.foot_acc_cmd[i] = wbc_test_data.foot_acc_cmd[i];
    msg.foot_acc_numeric[i] = wbc_test_data.foot_acc_numeric[i];
    msg.foot_pos[i] = wbc_test_data.foot_pos[i];
    msg.foot_vel[i] = wbc_test_data.foot_vel[i];
    msg.foot_local_pos[i] = wbc_test_data.foot_local_pos[i];
    msg.foot_local_vel[i] = wbc_test_data.foot_local_vel[i];
    msg.jpos_cmd[i] = wbc_test_data.jpos_cmd[i];
    msg.jvel_cmd[i] = wbc_test_data.jvel_cmd[i];
    msg.jacc_cmd[i] = wbc_test_data.jacc_cmd[i];
    msg.jpos[i] = wbc_test_data.jpos[i];
    msg.jvel[i] = wbc_test_data.jvel[i];
  }


  // body_ori_cmd, body_ori
  // for (int i = 0; i < 4; ++i) {
  //   msg.body_ori_cmd[i] = wbc_test_data.body_ori_cmd[i];
  //   msg.body_ori[i] = wbc_test_data.body_ori[i];
  // }
  // body_ori_cmd (quaternion w,x,y,z) -> msg.body_ori_cmd[3] (欧拉角: roll, pitch, yaw)
  // 四元数转欧拉角
  Eigen::Quaternionf quat(wbc_test_data.body_ori_cmd[0], wbc_test_data.body_ori_cmd[1],
                        wbc_test_data.body_ori_cmd[2], wbc_test_data.body_ori_cmd[3]);
  Eigen::Vector3f euler_angles = quat.toRotationMatrix().eulerAngles(2, 1, 0); // roll, pitch, yaw
  msg.body_ori_cmd[0] = euler_angles[2]; // roll
  msg.body_ori_cmd[1] = euler_angles[1]; // pitch
  msg.body_ori_cmd[2] = euler_angles[0]; // yaw
  // std::cout << "WBC Body Ori Cmd: " << msg.body_ori_cmd[0] << ", " << msg.body_ori_cmd[1] << ", " << msg.body_ori_cmd[2] << std::endl;

  Eigen::Quaternionf quat_ori(wbc_test_data.body_ori[0], wbc_test_data.body_ori[1],
                        wbc_test_data.body_ori[2], wbc_test_data.body_ori[3]);
  Eigen::Vector3f euler_angles_ori = quat_ori.toRotationMatrix().eulerAngles(2, 1, 0); // roll, pitch, yaw
  msg.body_ori[0] = euler_angles_ori[2]; // roll
  msg.body_ori[1] = euler_angles_ori[1]; // pitch
  msg.body_ori[2] = euler_angles_ori[0]; // yaw
  // std::cout << "WBC Body Ori: " << msg.body_ori[0] << ", " << msg.body_ori[1] << ", " << msg.body_ori[2] << std::endl;
  // body_pos_cmd, body_vel_cmd, body_ang_vel_cmd, body_pos, body_vel, body_ang_vel, vision_loc
  for (int i = 0; i < 3; ++i) {
    msg.body_pos_cmd[i] = wbc_test_data.body_pos_cmd[i];
    msg.body_vel_cmd[i] = wbc_test_data.body_vel_cmd[i];
    msg.body_ang_vel_cmd[i] = wbc_test_data.body_ang_vel_cmd[i];
    msg.body_pos[i] = wbc_test_data.body_pos[i];
    msg.body_vel[i] = wbc_test_data.body_vel[i];
    msg.body_ang_vel[i] = wbc_test_data.body_ang_vel[i];
    msg.vision_loc[i] = wbc_test_data.vision_loc[i];
  }

  return msg;
}


static inline sopu_msgs::msg::MotorCommand ConvertToMotorCommandMsg(const SpiCommand& spi_command)
{
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

static inline sopu_msgs::msg::MotorStatus ConvertToMotorStatusMsg(const SpiData& spi_data)
{
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

static inline SpiCommand ConvertToSpiCommand(const sopu_msgs::msg::MotorCommand& motor_command)
{
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


static inline geometry_msgs::msg::Point ToPointMsg(const Eigen::Vector3f& v)
{
  geometry_msgs::msg::Point p;
  p.x = v[0];
  p.y = v[1];
  p.z = v[2];
  return p;
}

static inline std_msgs::msg::ColorRGBA ToColorMsg(const Eigen::Vector4f& v)
{
  std_msgs::msg::ColorRGBA c;
  c.r = v[0];
  c.g = v[1];
  c.b = v[2];
  c.a = v[3];
  return c;
}

static inline void ConverToVisualizationMarker(const VisualizationData& vis_data,
                                               visualization_msgs::msg::MarkerArray& marker_array,
                                               const std::string& frame_id, const rclcpp::Time& stamp)
{
  int marker_id = 0;

  // Spheres
  for (int i = 0; i < vis_data.num_spheres && i < 10000; ++i) {
    const auto& sphere = vis_data.spheres[i];
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.ns = "spheres";
    marker.id = marker_id++;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = ToPointMsg(sphere.position);
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = sphere.radius * 2;
    marker.color = ToColorMsg(sphere.color);
    marker_array.markers.push_back(marker);
  }

  // Blocks (CUBE)
  for (int i = 0; i < vis_data.num_blocks && i < 10000; ++i) {
    const auto& block = vis_data.blocks[i];
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.ns = "blocks";
    marker.id = marker_id++;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = ToPointMsg(block.corner_position);
    // 仅支持无旋转展示，如需 support rpy 旋转可以用 tf2::Quaternion 填充 marker.pose.orientation
    marker.pose.orientation.w = 1.0;
    marker.scale.x = block.dimension[0];
    marker.scale.y = block.dimension[1];
    marker.scale.z = block.dimension[2];
    marker.color = ToColorMsg(block.color);
    marker_array.markers.push_back(marker);
  }

  // Arrows
  for (int i = 0; i < vis_data.num_arrows && i < 10000; ++i) {
    const auto& arrow = vis_data.arrows[i];
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.ns = "arrows";
    marker.id = marker_id++;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    geometry_msgs::msg::Point p_base = ToPointMsg(arrow.base_position);
    geometry_msgs::msg::Point p_tip;
    p_tip.x = p_base.x + arrow.direction[0];
    p_tip.y = p_base.y + arrow.direction[1];
    p_tip.z = p_base.z + arrow.direction[2];
    marker.points.push_back(p_base);
    marker.points.push_back(p_tip);

    marker.scale.x = arrow.shaft_width;   // shaft diameter
    marker.scale.y = arrow.head_width;    // head diameter
    marker.scale.z = arrow.head_length;   // head length
    marker.color = ToColorMsg(arrow.color);
    marker_array.markers.push_back(marker);
  }

  // Cones（ROS没有直接Cone，通常用CYLINDER近似）
  for (int i = 0; i < vis_data.num_cones && i < 10000; ++i) {
    const auto& cone = vis_data.cones[i];
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.ns = "cones";
    marker.id = marker_id++;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = ToPointMsg(cone.point_position);
    marker.pose.orientation.w = 1.0;
    marker.scale.x = cone.radius * 2.0;
    marker.scale.y = cone.radius * 2.0;
    marker.scale.z = std::sqrt(cone.direction[0]*cone.direction[0] + cone.direction[1]*cone.direction[1] + cone.direction[2]*cone.direction[2]);
    marker.color = ToColorMsg(cone.color);
    marker_array.markers.push_back(marker);
  }

  // Paths (each path as a line strip)
  for (int i = 0; i < vis_data.num_paths && i < 10; ++i) {
    const auto& path = vis_data.paths[i];
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.ns = "paths";
    marker.id = marker_id++;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    for (int j = 0; j < path.num_points && j < 2000; ++j) {
      geometry_msgs::msg::Point p;
      p.x = path.position[j][0];
      p.y = path.position[j][1];
      p.z = path.position[j][2];
      marker.points.push_back(p);
    }
    marker.scale.x = 0.01; // 可按需要设置线宽
    marker.color = ToColorMsg(path.color);
    marker_array.markers.push_back(marker);
  }
}

}  // namespace msg_conversion

}  // namespace hotdog_locomotion

#endif  // MSG_CONVERSION_HPP_
