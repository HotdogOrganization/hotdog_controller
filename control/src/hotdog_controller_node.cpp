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

#include "hotdog_controller/hotdog_controller_node.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace hotdog_locomotion
{
NewRobotController::NewRobotController() {}




controller_interface::CallbackReturn NewRobotController::on_init()
{





  try {
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

controller_interface::CallbackReturn NewRobotController::on_configure(
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
      std::bind(&NewRobotController::cmd_vel_cb, this, std::placeholders::_1));
  posestamped_subscription_ =
    get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
      tita_topic::manager_pose_command, rclcpp::SensorDataQoS().reliable(),
      std::bind(&NewRobotController::posestamped_cb, this, std::placeholders::_1));      
  fsm_goal_subscription_ =
    get_node()->create_subscription<std_msgs::msg::String>(
      tita_topic::manager_key_command, rclcpp::SensorDataQoS().reliable(),
      std::bind(&NewRobotController::fsm_goal_cb, this, std::placeholders::_1));

  joy_subscription_ =
    get_node()->create_subscription<sensor_msgs::msg::Joy>(
      tita_topic::manager_hotdog_key, rclcpp::SensorDataQoS().reliable(),
      std::bind(&NewRobotController::joy_cb, this, std::placeholders::_1));

  odom_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  odom_timer_ = get_node()->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&NewRobotController::odom_cb, this));
  odom_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(get_node());    
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration NewRobotController::command_interface_configuration() const
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

controller_interface::InterfaceConfiguration NewRobotController::state_interface_configuration() const
{
  std::vector<std::string> conf_names;
  for (std::shared_ptr<Joint> joint : joints_) {
    for (const auto & interface_type : state_interface_types_)
      conf_names.push_back(joint->name + "/" + interface_type);
  }
  for (auto name : imu_sensor_->get_state_interface_names()) conf_names.push_back(name);
  return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::return_type NewRobotController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
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

controller_interface::CallbackReturn NewRobotController::on_activate(const rclcpp_lifecycle::State &)
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
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn NewRobotController::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_node()->get_logger(), "on_deactivate ");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn NewRobotController::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_node()->get_logger(), "on_cleanup ");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn NewRobotController::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_node()->get_logger(), "on_error ");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn NewRobotController::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_node()->get_logger(), "on_shutdown ");
  return controller_interface::CallbackReturn::SUCCESS;
}

NewRobotController::~NewRobotController() {}

// TODO:
void NewRobotController::cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg){
  // (void)msg;
  // auto cmd = controlData_->state_command->rc_command_;
  // vel_gait_cmd_.x_vel_cmd = msg->linear.x;
  // vel_gait_cmd_.yaw_turn_rate = msg->angular.z;
}

void NewRobotController::posestamped_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
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

void NewRobotController::fsm_goal_cb(
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

void NewRobotController::joy_cb(
  const sensor_msgs::msg::Joy::SharedPtr msg)
{
  joy_data_.axes = msg->axes;
  joy_data_.buttons = msg->buttons;
}

void NewRobotController::odom_cb()
{
  std::string frame_prefix_ = auto_declare<std::string>("frame_prefix", "");  // 默认空字符串
  // 发布TransformStamped消息
  geometry_msgs::msg::TransformStamped odom_trans;
  odom_trans.header.stamp = get_node()->now();
  odom_trans.header.frame_id = "odom";
  // odom_trans.child_frame_id = frame_prefix + controlData_->params->base_name_;
  // odom_trans.transform.translation.x = controlData_->state_estimate->position(X);
  // odom_trans.transform.translation.y = controlData_->state_estimate->position(Y);
  // odom_trans.transform.translation.z = controlData_->state_estimate->position(Z);
  // odom_trans.transform.rotation = tf2::toMsg(tf2::Quaternion(
  //   controlData_->state_estimate->orientation(QX), controlData_->state_estimate->orientation(QY), controlData_->state_estimate->orientation(QZ),
  //   controlData_->state_estimate->orientation(QW)));
  odom_broadcaster_->sendTransform(odom_trans);

  // 发布Odometry消息
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = get_node()->now();
  odom_msg.header.frame_id = "odom";
  // odom_msg.child_frame_id = frame_prefix + controlData_->params->base_name_;
  // odom_msg.pose.pose.position.x = controlData_->state_estimate->position(X);
  // odom_msg.pose.pose.position.y = controlData_->state_estimate->position(Y);
  // odom_msg.pose.pose.position.z = controlData_->state_estimate->position(Z);
  // odom_msg.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(
  //   controlData_->state_estimate->orientation(QX), controlData_->state_estimate->orientation(QY), controlData_->state_estimate->orientation(QZ),
  //   controlData_->state_estimate->orientation(QW)));
  // odom_msg.twist.twist.linear.x = controlData_->state_estimate->vWorld(X);
  // odom_msg.twist.twist.linear.y = controlData_->state_estimate->vWorld(Y);
  // odom_msg.twist.twist.linear.z = controlData_->state_estimate->vWorld(Z);
  // odom_msg.twist.twist.angular.x = controlData_->state_estimate->omegaWorld(X);
  // odom_msg.twist.twist.angular.y = controlData_->state_estimate->omegaWorld(Y);
  // odom_msg.twist.twist.angular.z = controlData_->state_estimate->omegaWorld(Z);
  odom_msg.twist.covariance = {};
  odom_publisher_->publish(odom_msg);
}

void NewRobotController::setup_controller(){
  // robot_task_ = std::make_shared<WheelQuadruped_Task>(torque_);
  // controlData_ = std::make_shared<ControlFSMData>(joint_names_.size());
  setup_control_parameters();
  setup_state_estimate();
  // FSMController_ = std::make_shared<FSM>(controlData_);
}
void NewRobotController::mainLoopThread()
{
  RCLCPP_DEBUG(get_node()->get_logger(), "########################################################################");

  size_t id = 0;
  for (std::shared_ptr<Joint> joint : joints_) {
    motor_pos_[id] = joint->position_handle->get().get_value();
    motor_vel_[id] = joint->velocity_handle->get().get_value();
    id++;
  }

  quat_[0] = imu_sensor_->get_orientation()[3];
  for(size_t id = 0; id < 3; id++) {
    quat_[id + 1] = imu_sensor_->get_orientation()[id];
    gyro_[id] = imu_sensor_->get_angular_velocity()[id];
    accl_[id] = imu_sensor_->get_linear_acceleration()[id];
  }
  if (!init_flag_) {
    init_flag_ = true;
    // robot_task_->Run_Init();
    // vel_gait_cmd_.use_wbc = true;
  }
  // robot_task_->Run_Update(quat_, gyro_, accl_, motor_pos_, motor_vel_);
  // robot_task_->Run_Ctrl(mode_, vel_gait_cmd_);


  // static bool simulation_bridge_initialized_ = false;
//   if (!simulation_bridge_initialized_) {
//     RobotController* ctrl = new HotdogController();
//     MasterConfig gMasterConfig;
//     gMasterConfig.robot = RobotType::CYBERDOG2;
//     SimulationBridgeInterface simulationBridge( gMasterConfig.robot, ctrl );
//     simulationBridge.Run();
//     simulation_bridge_initialized_ = true;
// }
// // 每次循环都可以调用 Run()，或者根据你的需求只调用一次
//   // simulationBridge.Run();





  // 只初始化一次
  if (!simulation_bridge_initialized_) {
    ctrl_ = std::make_unique<HotdogController>();
    MasterConfig gMasterConfig;
    gMasterConfig.robot = RobotType::CYBERDOG2;
    simulation_bridge_ = std::make_unique<SimulationBridgeInterface>(gMasterConfig.robot, ctrl_.get());
    simulation_bridge_initialized_ = true;
  }
  // 后续直接调用
  SpiCommand spi_command_;
  float abad_effort[4],hip_effort[4],knee_effort[4];
  if (simulation_bridge_) {
    simulation_bridge_->Run(motor_pos_, motor_vel_, torque_, quat_, gyro_, accl_, spi_command_,joy_data_);
    static int count = 0; // 在函数外或者类成员变量中声明，确保每次循环时不会被重置

    // 在周期循环函数内
    if (count < 1000)
    {
        for (int i = 0; i < 4; ++i)
        {
          abad_effort[i] = 60 * (0 - motor_pos_[i*3+0]) +
                              1.5 * (0 - motor_vel_[i*3+0]) 
                              + spi_command_.tau_abad_ff[i];
      
          hip_effort[i] = 60 * (-1.5 - (-motor_pos_[i*3+1])) +
                              1.5 * (0 - (-motor_vel_[i*3+1]))
                              + spi_command_.tau_hip_ff[i];
      
          knee_effort[i] = 60 * (2.7 - (-motor_pos_[i*3+2])) +
                              1.5 * (0 - (-motor_vel_[i*3+2]))
                              + spi_command_.tau_knee_ff[i];
        }
        count++;
    }
    else
    {
        for (int i = 0; i < 4; ++i)
        {
            abad_effort[i] = spi_command_.kp_abad[i] * (spi_command_.q_des_abad[i] - motor_pos_[i*3+0]) +
                              spi_command_.kd_abad[i] * (0 - motor_vel_[i*3+0]) +
                              spi_command_.tau_abad_ff[i];

            hip_effort[i] = spi_command_.kp_hip[i] * (spi_command_.q_des_hip[i] - (-motor_pos_[i*3+1])) +
                            spi_command_.kd_hip[i] * (0 - (-motor_vel_[i*3+1])) +
                            spi_command_.tau_hip_ff[i];

            knee_effort[i] = spi_command_.kp_knee[i] * (spi_command_.q_des_knee[i] - (-motor_pos_[i*3+2])) +
                              spi_command_.kd_knee[i] * (0 - (-motor_vel_[i*3+2])) +
                              spi_command_.tau_knee_ff[i];
        }
    }

  }


  
  float t_tmp[4] = {0.};
  for(int i = 0; i < 4; i++) {
    t_tmp[i] = torque_[i];
    torque_[i] = torque_[4+i];
    torque_[4+i] = t_tmp[i];
  }

  for (int i = 0; i < 12; i++) {
    if (i % 4 != 3)
      torque_[i] *= 1.25;
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
  // torque_[12] = 0; 
  // Update torque
  for (uint id = 0; id < joints_.size(); id++) {
    joints_[id]->effort_command_handle->get().set_value(torque_[id]);
  }
  
  // wbcTimer_.endTimer();

  // TODO：debug infomation
  static int count = 0;
  if(count % 400 == 0){
    RCLCPP_DEBUG(get_node()->get_logger(), "########################################################################");
  // RCLCPP_DEBUG(get_node()->get_logger(), "\n### MPC Benchmarking");
  // RCLCPP_DEBUG(get_node()->get_logger(), "\n###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms].");
  // RCLCPP_DEBUG(get_node()->get_logger(), "\n###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms].");
  // RCLCPP_DEBUG(get_node()->get_logger(), "########################################################################");
    RCLCPP_DEBUG(get_node()->get_logger(), "\n### WBC Benchmarking");
  // RCLCPP_DEBUG(get_node()->get_logger(), "\n###   Maximum : %f [ms].", wbcTimer_.getMaxIntervalInMilliseconds());
  // RCLCPP_DEBUG(get_node()->get_logger(), "\n###   Average : %f [ms].", wbcTimer_.getAverageInMilliseconds());
  }
  count++;
}


void NewRobotController::setup_control_parameters()
{

}

void NewRobotController::update_control_parameters()
{

  RCLCPP_INFO(get_node()->get_logger(), "Parameters were updated");
}


void NewRobotController::setup_state_estimate()
{

}

void CheaterNewRobotController::setup_state_estimate()
{

}

}  // namespace tita_locomotion

#include "class_loader/register_macro.hpp"

PLUGINLIB_EXPORT_CLASS(hotdog_locomotion::NewRobotController, controller_interface::ControllerInterface)

// PLUGINLIB_EXPORT_CLASS(
//   tita_locomotion::CheaterNewRobotController, controller_interface::ControllerInterface)
