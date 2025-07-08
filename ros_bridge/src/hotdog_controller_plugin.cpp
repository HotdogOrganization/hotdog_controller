// Copyright (c) 2023 Direct Drive Technology Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "pluginlib/class_list_macros.hpp"

#include "ros_bridge/hotdog_controller_plugin.hpp"
#include "ros_bridge/msg_conversion.hpp"
namespace hotdog_locomotion
{
HotdogControllerPlugin::HotdogControllerPlugin() {}

controller_interface::CallbackReturn HotdogControllerPlugin::on_init()
{
  try {
    sim_mode_ = auto_declare<bool>("sim_mode", false);
    joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
    command_interface_types_ =
      auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
    state_interface_types_ =
      auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);
    sensor_names_ = auto_declare<std::vector<std::string>>("sensors", sensor_names_);
    imu_sensor_ = std::make_unique<semantic_components::IMUSensor>(
      semantic_components::IMUSensor(sensor_names_[0]));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Exception thrown during on_init stage with message: %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  // param list init
  param_listener_ = std::make_shared<hotdog_controller::ParamListener>(get_node());
  params_ = param_listener_->get_params();
  setup_controller();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HotdogControllerPlugin::on_configure(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "The 'joints' parameter is empty");
    return controller_interface::CallbackReturn::ERROR;
  } 

  for (std::string & joint_name : joint_names_) {
    RCLCPP_DEBUG(get_node()->get_logger(), "Get joint name : %s", joint_name.c_str());
    std::shared_ptr<Joint> joint = std::make_shared<Joint>();
    joint->name = joint_name;
    joints_.emplace_back(joint);
  }
  cmd_vel_subscription_ =
    get_node()->create_subscription<geometry_msgs::msg::Twist>(
      tita_topic::manager_twist_command, rclcpp::SensorDataQoS().reliable(),
      std::bind(&HotdogControllerPlugin::cmd_vel_cb, this, std::placeholders::_1));
  posestamped_subscription_ =
    get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
      tita_topic::manager_pose_command, rclcpp::SensorDataQoS().reliable(),
      std::bind(&HotdogControllerPlugin::posestamped_cb, this, std::placeholders::_1));      
  fsm_goal_subscription_ =
    get_node()->create_subscription<std_msgs::msg::String>(
      tita_topic::manager_key_command, rclcpp::SensorDataQoS().reliable(),
      std::bind(&HotdogControllerPlugin::fsm_goal_cb, this, std::placeholders::_1));

  joy_subscription_ =
    get_node()->create_subscription<sensor_msgs::msg::Joy>(
      tita_topic::manager_hotdog_key, rclcpp::SensorDataQoS().reliable(),
      std::bind(&HotdogControllerPlugin::joy_cb, this, std::placeholders::_1));

  imu_subscription_ = get_node()->create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data_raw", rclcpp::SensorDataQoS(),
    std::bind(&HotdogControllerPlugin::imu_cb, this, std::placeholders::_1));

  motor_status_subscription_ = get_node()->create_subscription<sopu_msgs::msg::MotorStatus>(
      "/motor_status", rclcpp::SensorDataQoS(),
      std::bind(&HotdogControllerPlugin::motor_status_cb, this, std::placeholders::_1));

  odom_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  
  // motor_status_publisher_
  //   = get_node()->create_publisher<sopu_msgs::msg::MotorStatus>(
  //     "motor_status", rclcpp::SensorDataQoS().reliable());

  motor_command_publisher_ =
    get_node()->create_publisher<sopu_msgs::msg::MotorCommand>(
      "/motor_command", rclcpp::SensorDataQoS().reliable());

  vis_marker_publisher_ =
    get_node()->create_publisher<visualization_msgs::msg::MarkerArray>(
      "mpc_marker", rclcpp::SensorDataQoS().reliable());

  odom_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(get_node());

  odom_timer_ = get_node()->create_wall_timer(
    std::chrono::milliseconds(10), std::bind(&HotdogControllerPlugin::odom_cb, this));

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration HotdogControllerPlugin::command_interface_configuration() const
{
  std::vector<std::string> conf_names;
  for (std::shared_ptr<Joint> joint : joints_) {
    for (const auto & interface_type : command_interface_types_) {
      conf_names.push_back(joint->name + "/" + interface_type);
      RCLCPP_DEBUG(get_node()->get_logger(), "Get joint cmd : %s", joint->name.c_str());
    }
  }
  return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::InterfaceConfiguration HotdogControllerPlugin::state_interface_configuration() const
{
  std::vector<std::string> conf_names;
  for (std::shared_ptr<Joint> joint : joints_) {
    for (const auto & interface_type : state_interface_types_)
      conf_names.push_back(joint->name + "/" + interface_type);
  }
  for (auto name : imu_sensor_->get_state_interface_names()) conf_names.push_back(name);
  return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::CallbackReturn HotdogControllerPlugin::on_activate(const rclcpp_lifecycle::State &)
{
  for (std::shared_ptr<Joint> joint : joints_) {
    // Position command
    const auto position_command_handle = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(),
      [&joint](const auto & interface)
      {
        return interface.get_prefix_name() == joint->name &&
               interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
      });
    if (position_command_handle == command_interfaces_.end()) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Unable to obtain joint command handle for %s",
        joint->name.c_str());
      return controller_interface::CallbackReturn::FAILURE;
    }
    joint->position_command_handle = std::ref(*position_command_handle);

    // Velocity command
    const auto velocity_command_handle = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(),
      [&joint](const auto & interface)
      {
        return interface.get_prefix_name() == joint->name &&
               interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
      });
    if (velocity_command_handle == command_interfaces_.end()) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Unable to obtain joint command handle for %s",
        joint->name.c_str());
      return controller_interface::CallbackReturn::FAILURE;
    }
    joint->velocity_command_handle = std::ref(*velocity_command_handle);

    // Effort command
    const auto effort_command_handle = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(), [&joint](const auto & interface) {
        return interface.get_prefix_name() == joint->name &&
               interface.get_interface_name() == hardware_interface::HW_IF_EFFORT;
      });
    if (effort_command_handle == command_interfaces_.end()) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Unable to obtain effort command handle for %s",
        joint->name.c_str());
      return controller_interface::CallbackReturn::FAILURE;
    }
    joint->effort_command_handle = std::ref(*effort_command_handle);
    // Position state
    const auto position_handle = std::find_if(
      state_interfaces_.begin(), state_interfaces_.end(), [&joint](const auto & interface) {
        return interface.get_prefix_name() == joint->name &&
               interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
      });
    if (position_handle == state_interfaces_.end()) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Unable to obtain joint state handle for %s",
        joint->name.c_str());
      return controller_interface::CallbackReturn::FAILURE;
    }
    joint->position_handle = std::ref(*position_handle);
    // Velocity state
    const auto velocity_handle = std::find_if(
      state_interfaces_.begin(), state_interfaces_.end(), [&joint](const auto & interface) {
        return interface.get_prefix_name() == joint->name &&
               interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
      });
    if (velocity_handle == state_interfaces_.end()) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Unable to obtain joint state handle for %s",
        joint->name.c_str());
      return controller_interface::CallbackReturn::FAILURE;
    }
    joint->velocity_handle = std::ref(*velocity_handle);
    // Effort state
    const auto effort_handle = std::find_if(
      state_interfaces_.begin(), state_interfaces_.end(), [&joint](const auto & interface) {
        return interface.get_prefix_name() == joint->name &&
               interface.get_interface_name() == hardware_interface::HW_IF_EFFORT;
      });
    if (effort_handle == state_interfaces_.end()) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Unable to obtain joint state handle for %s",
        joint->name.c_str());
      return controller_interface::CallbackReturn::FAILURE;
    }
    joint->effort_handle = std::ref(*effort_handle);
  }
  imu_sensor_->assign_loaned_state_interfaces(state_interfaces_);
  is_running_ = true;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HotdogControllerPlugin::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_node()->get_logger(), "on_deactivate ");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HotdogControllerPlugin::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_node()->get_logger(), "on_cleanup ");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HotdogControllerPlugin::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_node()->get_logger(), "on_error ");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HotdogControllerPlugin::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_node()->get_logger(), "on_shutdown ");
  return controller_interface::CallbackReturn::SUCCESS;
}

HotdogControllerPlugin::~HotdogControllerPlugin() {}


void HotdogControllerPlugin::imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg)
{
//   RCLCPP_INFO(
//       rclcpp::get_logger("HotdogControllerPlugin"),
//       "IMU接收: orientation=(%.3f, %.3f, %.3f, %.3f), "
//       "angular_velocity=(%.3f, %.3f, %.3f), "
//       "linear_acceleration=(%.3f, %.3f, %.3f)",
//       msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w,
//       msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z,
//       msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z
// );

  std::lock_guard<std::mutex> lock(imu_mutex_);
  latest_imu_msg_ = msg;
}


void HotdogControllerPlugin::motor_status_cb(const sopu_msgs::msg::MotorStatus::SharedPtr msg)
{
  // for (int i = 0; i < 4; ++i) {
  //   RCLCPP_INFO(get_node()->get_logger(),
  //       "关节%d: q_abad=%.3f q_hip=%.3f q_knee=%.3f | qd_abad=%.3f qd_hip=%.3f qd_knee=%.3f | tau_abad=%.3f tau_hip=%.3f tau_knee=%.3f | temp_abad=%.3f temp_hip=%.3f temp_knee=%.3f | flag_abad=%d flag_hip=%d flag_knee=%d",
  //       i,
  //       msg->q_abad[i], msg->q_hip[i], msg->q_knee[i],
  //       msg->qd_abad[i], msg->qd_hip[i], msg->qd_knee[i],
  //       msg->tau_abad[i], msg->tau_hip[i], msg->tau_knee[i],
  //       msg->temp_abad[i], msg->temp_hip[i], msg->temp_knee[i],
  //       msg->flag_abad[i], msg->flag_hip[i], msg->flag_knee[i]
  //   );
  // }
  std::lock_guard<std::mutex> lock(motor_mutex_);
  latest_motor_status_msg_ = msg;
}

// TODO:
void HotdogControllerPlugin::cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg){
  // (void)msg;
  // auto cmd = controlData_->state_command->rc_command_;
  // vel_gait_cmd_.x_vel_cmd = msg->linear.x;
  // vel_gait_cmd_.yaw_turn_rate = msg->angular.z;
}

void HotdogControllerPlugin::posestamped_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
  (void)msg;
  // RCLCPP_INFO(get_node()->get_logger(), 
  // "Received PoseStamped: position(x: %f, y: %f, z: %f), orientation(x: %f, y: %f, z: %f, w: %f)",
  // msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
  // msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);

  tf2::Quaternion q(
    msg->pose.orientation.x,
    msg->pose.orientation.y,
    msg->pose.orientation.z,
    msg->pose.orientation.w
  );

  // 将四元数转换为RPY
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  // 打印转换后的RPY值
  // RCLCPP_INFO(get_node()->get_logger(), 
  //   "Converted RPY: roll: %f, pitch: %f, yaw: %f", 
  //   roll, pitch, yaw);
  // auto cmd = controlData_->state_command->rc_command_;
  // cmd->pose_position_[Y] = msg->pose.position.y;
  // cmd->pose_position_[Z] = msg->pose.position.z;
  // cmd->pose_orientation_[QX] = msg->pose.orientation.x;
  // cmd->pose_orientation_[QY] = msg->pose.orientation.y;
  // cmd->pose_orientation_[QZ] = msg->pose.orientation.z;
  // cmd->pose_orientation_[QW] = msg->pose.orientation.w;
}

void HotdogControllerPlugin::fsm_goal_cb(
  const std_msgs::msg::String::SharedPtr msg)
{
  // (void)msg;
  // auto cmd = controlData_->state_command->rc_command_;
  // cmd->fsm_name_ = msg->data;
  if(msg->data == "idle" || msg->data == "transform_down")
    mode_ = 3;
  else if(msg->data == "transform_up")
    mode_ = 1;
  else if(msg->data == "balance_stand")
    mode_ = 4;
  else if(msg->data == "jump")
    mode_ = 5;
  else
    mode_ = 0;
}

void HotdogControllerPlugin::joy_cb(
  const sensor_msgs::msg::Joy::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(joy_data_mutex_);
    joy_data_.axes = msg->axes;
    joy_data_.buttons = msg->buttons;
  }
  is_joy_received_ = true;
}

void HotdogControllerPlugin::odom_cb()
{
  if (simulation_bridge_== nullptr) {
    RCLCPP_ERROR(get_node()->get_logger(), "Simulation bridge is not initialized.");
    return;
  }
  const auto& state_estimator_result = simulation_bridge_->GetStateEstimateData();
  // 发布全局坐标系到机器人坐标系的变换

  std::string frame_prefix_ = auto_declare<std::string>("frame_prefix", "");  // 默认空字符串
  // 发布TransformStamped消息
  geometry_msgs::msg::TransformStamped odom_trans;
  odom_trans.header.stamp = get_node()->now();
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = state_estimator_result.position.x();
  odom_trans.transform.translation.y = state_estimator_result.position.y();
  odom_trans.transform.translation.z = state_estimator_result.position.z();
  // std::cout << "odom  translation: "
  //           << odom_trans.transform.translation.x << ", "
  //           << odom_trans.transform.translation.y << ", "
  //           << odom_trans.transform.translation.z << std::endl;

  Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(state_estimator_result.rpy[0], Eigen::Vector3d::UnitX()));
  Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(state_estimator_result.rpy[1],Eigen::Vector3d::UnitY()));
  Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(state_estimator_result.rpy[2],Eigen::Vector3d::UnitZ()));

  Eigen::Quaterniond quaternion;
  quaternion = yawAngle * pitchAngle * rollAngle;

  tf2::Quaternion q(quaternion.x(), quaternion.y(), quaternion.z(),quaternion.w());
  geometry_msgs::msg::Quaternion geoQuat;
  tf2::convert(q, geoQuat);

  odom_trans.transform.rotation = geoQuat;

  odom_broadcaster_->sendTransform(odom_trans);

  // // 发布Odometry消息
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = get_node()->now();
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_link";
  odom_msg.pose.pose.position.x = state_estimator_result.position.x();
  odom_msg.pose.pose.position.y = state_estimator_result.position.y();
  odom_msg.pose.pose.position.z = state_estimator_result.position.z();
  // 从RPY生成四元数并赋值给odom_msg.pose.pose.orientation
  tf2::Quaternion odom_quat;
  odom_quat.setRPY(state_estimator_result.rpy[0], state_estimator_result.rpy[1], state_estimator_result.rpy[2]);
  odom_msg.pose.pose.orientation = tf2::toMsg(odom_quat);
  odom_msg.twist.twist.linear.x = state_estimator_result.vWorld.x();
  odom_msg.twist.twist.linear.y = state_estimator_result.vWorld.y();
  odom_msg.twist.twist.linear.z = state_estimator_result.vWorld.z();
  odom_msg.twist.twist.angular.x = state_estimator_result.omegaWorld.x();
  odom_msg.twist.twist.angular.y = state_estimator_result.omegaWorld.y();
  odom_msg.twist.twist.angular.z = state_estimator_result.omegaWorld.z();
  odom_msg.twist.covariance = {};
  odom_publisher_->publish(odom_msg);
}

void HotdogControllerPlugin::publish_motor_status(const SpiData & spi_data)
{
  sopu_msgs::msg::MotorStatus motor_status_msg =
              msg_conversion::ConvertToMotorStatusMsg(spi_data);
  motor_status_msg.header.stamp = rclcpp::Clock().now();

  motor_status_publisher_->publish(motor_status_msg);
}

void HotdogControllerPlugin::publish_motor_command(const SpiCommand & spi_command)
{
  sopu_msgs::msg::MotorCommand motor_command_msg =
              msg_conversion::ConvertToMotorCommandMsg(spi_command);
  motor_command_msg.header.stamp = rclcpp::Clock().now();

  motor_command_publisher_->publish(motor_command_msg);
}

controller_interface::return_type HotdogControllerPlugin::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (!is_running_) {
    RCLCPP_ERROR(get_node()->get_logger(), "Controller is not running, please activate it first.");
    return controller_interface::return_type::ERROR;
  }

  (void)time;
  (void)period;
  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();
    update_control_parameters();
  }
  // controlData_->params->dt_ = period.seconds();

  mainLoopThread();
  // TODO:
  // if (!lqr_thread_running_) {
  //   lqr_thread_running_ = true;
  //   lqr_thread_ = std::thread([this]() {
  //     while (lqr_thread_running_) {
  //       lqrLoopThread();
  //     }
  //   });
  // }
  return controller_interface::return_type::OK;
}


void HotdogControllerPlugin::setup_controller()
{
  hotdog_controller_ = std::make_unique<HotdogController>();
  RobotType robot_type = RobotType::CYBERDOG2;
  simulation_bridge_ = std::make_unique<SimulationBridge>(robot_type, hotdog_controller_.get());
}

void HotdogControllerPlugin::mainLoopThread()
{
  RCLCPP_DEBUG(get_node()->get_logger(), "########################################################################");

  // 1. 变量声明提前
  size_t id = 0;
  float abad_effort[4] = {0}, hip_effort[4] = {0}, knee_effort[4] = {0};
  SpiCommand spi_command; // 注意：后面会被赋值

  // 2. 读取电机和IMU数据
  // 仿真数据读取
  std::shared_ptr<SpiData> motor_status = nullptr;
  std::shared_ptr<VectorNavData> vector_nav_data = nullptr;
  if (sim_mode_) {
    for (std::shared_ptr<Joint> joint : joints_) {
      motor_pos_[id] = joint->position_handle->get().get_value();
      motor_vel_[id] = joint->velocity_handle->get().get_value();
      id++;
    }
    motor_status = std::make_shared<SpiData>(
                          msg_conversion::ConvertToSpiData(motor_pos_, motor_vel_, torque_));


    quat_[0] = imu_sensor_->get_orientation()[3];
    for(size_t id = 0; id < 3; id++) {
      quat_[id + 1] = imu_sensor_->get_orientation()[id];
      gyro_[id] = imu_sensor_->get_angular_velocity()[id];
      accl_[id] = imu_sensor_->get_linear_acceleration()[id];
    }
    vector_nav_data = std::make_shared<VectorNavData>(
                            msg_conversion::ConvertToVectorNavData(quat_, gyro_, accl_));
  } else {
    // 实车数据
    if (latest_motor_status_msg_ != nullptr) {
      std::lock_guard<std::mutex> lock(motor_mutex_);
      motor_status = std::make_shared<SpiData>(
                      msg_conversion::ConvertToSpiData(*latest_motor_status_msg_));
      latest_motor_status_msg_.reset();  // 清空最新的电机状态消息
    } else {
      RCLCPP_WARN(get_node()->get_logger(), "No motor status message received.");
      return;  // 如果没有接收到电机状态消息，直接返回
    }

    if (latest_imu_msg_ != nullptr) {
      std::lock_guard<std::mutex> lock(imu_mutex_);
      vector_nav_data = std::make_shared<VectorNavData>(
                              msg_conversion::ConvertToVectorNavData(*latest_imu_msg_));
      latest_imu_msg_.reset();  // 清空最新的IMU消息
    } else {
      RCLCPP_WARN(get_node()->get_logger(), "No IMU message received.");
      return;  // 如果没有接收到IMU消息，直接返回
    }
  }

  if (simulation_bridge_) {
    // 处理手柄数据
    GamepadCommand gamepad_command;
    {

      std::lock_guard<std::mutex> lock(joy_data_mutex_);
      // printf("!!!!!!!!!!!!!!!!!!!!!!!!!!\n\r");

      if (is_joy_received_) {
        // printf("....................................................................\n\r");
        gamepad_command.a = joy_data_.buttons[0];
        gamepad_command.b = joy_data_.buttons[1];
        gamepad_command.x = joy_data_.buttons[2];
        gamepad_command.y = joy_data_.buttons[3];
        gamepad_command.xyz[0] = joy_data_.axes[0];
        gamepad_command.xyz[1] = joy_data_.axes[1];
        gamepad_command.xyz[2] = joy_data_.axes[6];
        gamepad_command.rpy[0] = joy_data_.axes[2];
        gamepad_command.rpy[1] = joy_data_.axes[3];
        gamepad_command.rpy[2] = joy_data_.axes[4];
        gamepad_command.yaw_direction = joy_data_.axes[5];
        simulation_bridge_->SetGamepadCommand(gamepad_command);
      }
    }

    simulation_bridge_->SetSpiData(*motor_status);
    simulation_bridge_->SetVectorNavData(*vector_nav_data);
    // double roll, pitch, yaw;
    // tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    simulation_bridge_->Run();
    
    const auto spi_command = simulation_bridge_->GetSpiCommand();

    static int vis_pub_count = 0;
    if (vis_pub_count % 50 == 0) {
      auto vis_data = simulation_bridge_->GetVisualizationData();
      visualization_msgs::msg::MarkerArray vis_marker_array;
      msg_conversion::ConverToVisualizationMarker(vis_data, vis_marker_array,"odom", get_node()->now());
      vis_marker_publisher_->publish(vis_marker_array);
    }
    vis_pub_count++;

    static int count = 0;   // 静态变量，记得不要在 if/else 内再声明
    // 4. 控制参数赋值
    if (count < 1000) {
      if (sim_mode_) {
        for (int i = 0; i < 4; ++i) {
          abad_effort[i] = 60 * (0 - motor_pos_[i*3+0]) + 1.5 * (0 - motor_vel_[i*3+0]) + spi_command.tau_abad_ff[i];
          hip_effort[i] = 60 * (-1.2 - (-motor_pos_[i*3+1])) + 1.5 * (0 - (-motor_vel_[i*3+1])) + spi_command.tau_hip_ff[i];
          knee_effort[i] = 60 * (2.5 - (-motor_pos_[i*3+2])) + 1.5 * (0 - (-motor_vel_[i*3+2])) + spi_command.tau_knee_ff[i];
        }
      } else {
        for (int i = 0; i < 4; ++i) {
          abad_effort[i] = 0;
          hip_effort[i] = 0;
          knee_effort[i] = 0;
        }
        SpiCommand temp_spi_command;
        // 这里你原本是直接赋 spi_command 的参数，这里建议做成 for 循环，简化代码
        for (int i = 0; i < 4; ++i) {
          temp_spi_command.q_des_abad[i] = 0;
          temp_spi_command.q_des_hip[i] = -1.2;
          temp_spi_command.q_des_knee[i] = 2.5;
          temp_spi_command.qd_des_abad[i] = 0;
          temp_spi_command.qd_des_hip[i] = 0;
          temp_spi_command.qd_des_knee[i] = 0;
          temp_spi_command.kp_abad[i] = 5;
          temp_spi_command.kp_hip[i] = 5;
          temp_spi_command.kp_knee[i] = 5;
          temp_spi_command.kd_abad[i] = 0.1;
          temp_spi_command.kd_hip[i] = 0.1;
          temp_spi_command.kd_knee[i] = 0.1;
          temp_spi_command.tau_abad_ff[i] = 0;
          temp_spi_command.tau_hip_ff[i] = 0;
          temp_spi_command.tau_knee_ff[i] = 0;
        }
      }
      count++;
    } else {
      if (sim_mode_) {
        // 仿真模式下
        for (int i = 0; i < 4; ++i) {
          abad_effort[i] = spi_command.kp_abad[i] * (spi_command.q_des_abad[i] - motor_pos_[i*3+0])
                        + spi_command.kd_abad[i] * (0 - motor_vel_[i*3+0])
                        + spi_command.tau_abad_ff[i];
          hip_effort[i] = spi_command.kp_hip[i] * (spi_command.q_des_hip[i] - (-motor_pos_[i*3+1]))
                        + spi_command.kd_hip[i] * (0 - (-motor_vel_[i*3+1]))
                        + spi_command.tau_hip_ff[i];
          knee_effort[i] = spi_command.kp_knee[i] * (spi_command.q_des_knee[i] - (-motor_pos_[i*3+2]))
                        + spi_command.kd_knee[i] * (0 - (-motor_vel_[i*3+2]))
                        + spi_command.tau_knee_ff[i];
        }
      }
      publish_motor_command(spi_command);
    }

  }


  // 5. torque_与 effort 赋值
  float t_tmp[4] = {0.};
  for(int i = 0; i < 4; i++) {
    t_tmp[i] = torque_[i];
    torque_[i] = torque_[4+i];
    torque_[4+i] = t_tmp[i];
  }
  for (int i = 0; i < 12; i++) {
    if (i % 4 != 3) {
      if (torque_[i] < -65.0)
        torque_[i] = -65.0;
      else if (torque_[i] > 65.0)
        torque_[i] = 65.0;
    } else {
      if (torque_[i] < -10.0)
        torque_[i] = -10.0;
      else if (torque_[i] > 10.0)
        torque_[i] = 10.0;
    }
  }

  for (int i = 0; i < 4; ++i) {
    torque_[i * 3 + 0] = abad_effort[i];
    torque_[i * 3 + 1] = -hip_effort[i];
    torque_[i * 3 + 2] = -knee_effort[i];
  }

  // 更新 effort
  for (uint id = 0; id < joints_.size(); id++) {
    joints_[id]->effort_command_handle->get().set_value(torque_[id]);
  }
}


void HotdogControllerPlugin::setup_control_parameters()
{

}

void HotdogControllerPlugin::update_control_parameters()
{

  RCLCPP_INFO(get_node()->get_logger(), "Parameters were updated");
}


void HotdogControllerPlugin::setup_state_estimate()
{

}

// void CheaterHotdogControllerPlugin::setup_state_estimate()
// {

// }

}  // namespace hotdog_locomotion

#include "class_loader/register_macro.hpp"

PLUGINLIB_EXPORT_CLASS(hotdog_locomotion::HotdogControllerPlugin, controller_interface::ControllerInterface)

// PLUGINLIB_EXPORT_CLASS(
//   tita_locomotion::CheaterHotdogControllerPlugin, controller_interface::ControllerInterface)
