#ifndef SIMULATION_BRIDGE_HPP_
#define SIMULATION_BRIDGE_HPP_

#include <lcm/lcm-cpp.hpp>
#include <thread>

#include "parameters/robot_parameters.hpp"
#include "parameters/calibrate_parameters.hpp"
#include "utilities/periodic_task.hpp"
#include "control_flags.hpp"
#include "command_interface/command_interface.hpp"
#include "robot_controller.hpp"
#include "robot_runner_Interface.hpp"

/**
 * @brief The SimulationBridge runs a RobotController and connects it to a
 * Simulator, using shared memory. It is the simulation version of the HardwareBridge.
 * 
 */
class SimulationBridge {
public:
  explicit SimulationBridge(RobotType robot_type, RobotController* robot_ctrl);
  ~SimulationBridge();

  void Run();
  void RunRobotControl();

  SpiData                         spi_data_;
  SpiCommand                      spi_command_;
  VectorNavData                   vector_nav_data_;
  GamepadCommand                  gamepadCommand;
  VisualizationData               visualization_data_;

private:
  bool LoadControlParametersFromFiles();
  bool InitRobotRunner();

private:
  RobotType                  robot_type_;
  std::shared_ptr<RobotControlParameters>     robot_params_ = nullptr;
  ControlParameters*         user_control_parameters_  = nullptr;

  bool                       first_controller_run_ = true;
  std::shared_ptr<PeriodicTaskManager>   task_manager_ = nullptr;
  RobotController*           controller_   = nullptr;
  std::shared_ptr<RobotRunnerInterface>  robot_runner_ = nullptr;

  CommandInterface cmd_interface_;
  SpeedCalibrateParameters speed_param_;

  // ComplementaryFilter imu_filter_;
  // MahonyFilter        imu_mh_filter_;
  // Cyberdog2Visualization          hotdog2_visualization_;
};

#endif  // SIMULATION_BRIDGE_HPP_
