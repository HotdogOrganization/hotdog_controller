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
    void Run(const float* motor_pos, const float* motor_vel, const float* quat, const float* gyro, const float* accl);
    void HandleControlParameters();
    void RunRobotControl();

private:
    SimulationBridge* bridge_;
};

#endif  // SIMULATION_BRIDGE_INTERFACE_HPP_
