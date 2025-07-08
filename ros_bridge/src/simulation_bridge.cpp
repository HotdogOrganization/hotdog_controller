/*! @file simulation_bridge.cpp
 *  @brief  The SimulationBridge runs a RobotController and connects it to a
 * Simulator, using shared memory. It is the simulation version of the
 * HardwareBridge.
 */

#include <iostream>

#include <glog/logging.h>

#include "ros_bridge/simulation_bridge.hpp"
#include "utilities/segfault_handler.hpp"

namespace hotdog_locomotion
{

SimulationBridge::SimulationBridge(RobotType robot_type, RobotController* robot_ctrl)
: robot_type_(robot_type), controller_(robot_ctrl)
{
  task_manager_ = std::make_shared<PeriodicTaskManager>();

  robot_params_ = std::make_shared<RobotControlParameters>();

  user_control_parameters_  = robot_ctrl->GetUserControlParameters();
  assert(user_control_parameters_ != nullptr && "User control parameters must not be null");
}

SimulationBridge::~SimulationBridge()
{
  delete user_control_parameters_;
  delete controller_;
}


bool SimulationBridge::LoadControlParametersFromFiles()
{
  printf("[ROS Bridge] Loading parameters from file...\n");

  // TODO: 添加speed param

  printf( "[Hardware Bridge] Loading parameters from file...\n" );

  try {
    robot_params_->InitializeFromYamlFile( THIS_COM "common/config/robot-defaults.yaml" );

    if ( SPEED_PARAMETER_CALIBRATE_CONTROL_DEBUG ) {
      robot_params_->speed_offset_trot_10_4 += speed_param_.speed_offset_trot_10_4;
      robot_params_->speed_offset_trot_follow += speed_param_.speed_offset_trot_follow;
      robot_params_->speed_offset_trot_medium += speed_param_.speed_offset_trot_medium;
      robot_params_->speed_offset_trot_24_16 += speed_param_.speed_offset_trot_24_16;
      robot_params_->speed_offset_trot_slow += speed_param_.speed_offset_trot_slow;
      robot_params_->speed_offset_trot_fast += speed_param_.speed_offset_trot_fast;
      robot_params_->speed_offset_trot_8_3 += speed_param_.speed_offset_trot_8_3;
      robot_params_->speed_offset_ballet += speed_param_.speed_offset_ballet;
      robot_params_->speed_offset_bound += speed_param_.speed_offset_bound;
      robot_params_->speed_offset_pronk += speed_param_.speed_offset_pronk;
      robot_params_->se_ori_cali_offset += speed_param_.se_ori_cali_offset;
      robot_params_->se_ori_cali_gain += speed_param_.se_ori_cali_gain;
    }

    std::cout << "**robot_params_->speed_offset_trot_slow: **** " << robot_params_->speed_offset_trot_slow.transpose() << std::endl;
    std::cout << "**robot_params_->speed_offset_trot_medium: ** " << robot_params_->speed_offset_trot_medium.transpose() << std::endl;
    std::cout << "**robot_params_->speed_offset_trot_fast: **** " << robot_params_->speed_offset_trot_fast.transpose() << std::endl;
    std::cout << "**robot_params_->speed_offset_trot_follow: ** " << robot_params_->speed_offset_trot_follow.transpose() << std::endl;
    std::cout << "**robot_params_->speed_offset_ballet: ******* " << robot_params_->speed_offset_ballet.transpose() << std::endl;
    std::cout << "**robot_params_->speed_offset_bound: ******** " << robot_params_->speed_offset_bound.transpose() << std::endl;
    std::cout << "**robot_params_->speed_offset_pronk: ******** " << robot_params_->speed_offset_pronk.transpose() << std::endl;
    std::cout << "**robot_params_->speed_offset_trot_8_3: ***** " << robot_params_->speed_offset_trot_8_3.transpose() << std::endl;
    std::cout << "**robot_params_->speed_offset_trot_10_4: **** " << robot_params_->speed_offset_trot_10_4.transpose() << std::endl;
    std::cout << "**robot_params_->speed_offset_trot_24_16: *** " << robot_params_->speed_offset_trot_24_16.transpose() << std::endl;
    std::cout << "**robot_params_->se_ori_cali_offset: ******** " << robot_params_->se_ori_cali_offset.transpose() << std::endl;
    std::cout << "**robot_params_->se_ori_cali_gain: ********** " << robot_params_->se_ori_cali_gain.transpose() << std::endl;
  }
  catch ( std::exception& e ) {
    printf( "Failed to initialize robot parameters from yaml file: %s\n", e.what() );
    return false;
  }

  if ( !robot_params_->IsFullyInitialized() ) {
    printf( "Failed to initialize all robot parameters\n" );
    return false;
  }

  printf( "Loaded robot parameters\n" );

  // Step: 加载用户参数
  if (user_control_parameters_) {
    try {
      if (robot_type_ == RobotType::HOTDOG || robot_type_ == RobotType::CYBERDOG2)
      {
        user_control_parameters_->InitializeFromYamlFile(THIS_COM "common/config/hotdog2-ctrl-user-parameters.yaml");
      }
      else {
        printf("Robot type not supported for user control parameters loading\n");
        return false;
      }
    }

    catch (std::exception& e) {
      printf("Failed to initialize user parameters from yaml file: %s\n", e.what());
      return false;
    }

    if (!user_control_parameters_->IsFullyInitialized()) {
      printf("Failed to initialize all user parameters\n");
      return false;
    }

    printf("Loaded user parameters\n");
  }
  else {
    printf("Did not load user parameters because there aren't any\n");
  }

  return true;
}

void SimulationBridge::ClearCommands()
{
  gamepad_command_.reset();
  keyboard_command_.reset();
  rc_command_.reset();
}


bool SimulationBridge::InitRobotRunner()
{
  printf("[SimulationBridge] Initializing RobotRunner...\n");
  robot_runner_ = std::make_shared<RobotRunnerInterface>(controller_, task_manager_.get(), robot_params_->controller_dt, "robot-control");

  if (!robot_runner_) {
    throw std::runtime_error("[SimulationBridge] RobotRunner is not initialized");
    return false;
  }

  robot_runner_->SetRobotType(robot_type_);
  robot_runner_->SetRobotAppearanceType(RobotAppearanceType::CURVED);

  robot_runner_->SetCommandInterface(&cmd_interface_);

  robot_runner_->SetSpiData(&spi_data_ );
  robot_runner_->SetSpiCommand(&spi_command_);
  robot_runner_->SetVectorNavData(&vector_nav_data_);
  robot_runner_->SetRobotControlParameters(robot_params_.get());

  // robot_runner_->SetUserControlParameters(user_control_parameters_);
  robot_runner_->SetVisualizationData( &visualization_data_ );
  robot_runner_->SetWbcTestData(&wbc_test_data_);

  return true;
}


/**
 * @brief Connect to a simulation.
 *
 */
void SimulationBridge::Run()
{
  RunRobotControl();
}


/**
 * @brief Run the robot controller.
 *
 */
void SimulationBridge::RunRobotControl()
{
  if ( first_controller_run_ ) {
    std::cout << first_controller_run_ << std::endl;
    LoadControlParametersFromFiles();
    InitRobotRunner();
    robot_runner_->Init();
    first_controller_run_ = false;
  }

  if (rc_command_) {
    cmd_interface_.ProcessRcCommand(*rc_command_);
  } else if (gamepad_command_) {
    cmd_interface_.ProcessGamepadCommand(*gamepad_command_);
  } else if (keyboard_command_) {
    cmd_interface_.ProcessKeyboardCommand(*keyboard_command_);
  }

  ClearCommands();
  // std::cout << "Gamepad command processed" << std::endl;
  state_estimate_data_ = robot_runner_->GetStateEstimateData();
  robot_runner_->Run();
}

}  // namespace hotdog_locomotion
