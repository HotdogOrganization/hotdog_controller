#ifndef SIMULATION_BRIDGE_INTERFACE_HPP_
#define SIMULATION_BRIDGE_INTERFACE_HPP_

#include "cpp_types.hpp"
#include "robot_controller.hpp"

class SimulationBridge;

/**
 * @brief The SimulationBridgeInterface runs a RobotController and connects it to a
 * Simulator, using shared memory. It is the simulation version of the HardwareBridge.
 *
 */
class SimulationBridgeInterface {
public:
    explicit SimulationBridgeInterface( RobotType robot, RobotController* robot_ctrl );
    ~SimulationBridgeInterface();
    struct JoyData {
        std::vector<float> axes;
        std::vector<int32_t> buttons;
      };

    void Run(const float* motor_pos, const float* motor_vel, const float* motor_tor, const float* quat, const float* gyro, const float* accl, SpiCommand &spi_command_,JoyData &joydata);
    void HandleControlParameters();
    void RunRobotControl();

private:
    SimulationBridge* bridge_;
};

#endif  // SIMULATION_BRIDGE_INTERFACE_HPP_
