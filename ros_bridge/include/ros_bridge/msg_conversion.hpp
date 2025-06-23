#ifndef MSG_CONVERSION_HPP_
#define MSG_CONVERSION_HPP_

#include <vector>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "visualization_msgs/msg/marker_array.hpp"
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
