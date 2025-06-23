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
#include "msg_conversion.hpp"
#include "sopu_msgs/msg/motor_command.hpp"
#include "sopu_msgs/msg/motor_status.hpp"

#include "header/lcm_type/localization_lcmt.hpp"

namespace hotdog_locomotion
{

/**
 * @brief The SimulationBridge runs a RobotController and connects it to a
 * Simulator, using shared memory. It is the simulation version of the HardwareBridge.
 * 
 */
class SimulationBridge {
public:
  struct JoyData
  {
    std::vector<float> axes;
    std::vector<int32_t> buttons;
  };

  explicit SimulationBridge(RobotType robot_type, RobotController* robot_ctrl);
  ~SimulationBridge();

  void Run();
  void RunRobotControl();

  void SetSpiData(const SpiData& spi_data) {
    spi_data_ = spi_data;
  }
  void SetSpiCommand(const SpiCommand& spi_command) {
    spi_command_ = spi_command;
  }
  void SetVectorNavData(const VectorNavData& vector_nav_data) {
    vector_nav_data_ = vector_nav_data;
  }
  void SetGamepadCommand(const GamepadCommand& gamepad_command) {
    gamepad_command_ = gamepad_command;
  }

  const SpiData& GetSpiData() const {
    return spi_data_;
  }
  const SpiCommand& GetSpiCommand() const {
    return spi_command_;
  }
  const VectorNavData& GetVectorNavData() const {
    return vector_nav_data_;
  }
  const GamepadCommand& GetGamepadCommand() const {
    return gamepad_command_;
  }
  const VisualizationData& GetVisualizationData() const {
    return visualization_data_;
  }
  const localization_lcmt& GetGlobalToRobotLcm() const {
    return global_to_robot_lcmt_;
  }
  
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

  SpiData                         spi_data_;
  SpiCommand                      spi_command_;
  VectorNavData                   vector_nav_data_;
  GamepadCommand                  gamepad_command_;
  VisualizationData               visualization_data_;
  localization_lcmt               global_to_robot_lcmt_;
  // ComplementaryFilter imu_filter_;
  // MahonyFilter        imu_mh_filter_;
  // Cyberdog2Visualization          hotdog2_visualization_;
};

}  // namespace hotdog_locomotion

#endif  // SIMULATION_BRIDGE_HPP_
