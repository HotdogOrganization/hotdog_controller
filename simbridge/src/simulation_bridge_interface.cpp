#include "simulation_bridge_interface.hpp"
#include "simulation_bridge.hpp"

SimulationBridgeInterface::SimulationBridgeInterface( RobotType robot, RobotController* robot_ctrl ) {
    bridge_ = new SimulationBridge( robot, robot_ctrl );
}

SimulationBridgeInterface::~SimulationBridgeInterface() {
    delete bridge_;
}

void SimulationBridgeInterface::Run(const float* motor_pos, const float* motor_vel, const float* quat, const float* gyro, const float* accl)
{
    for(int leg = 0; leg < 4; ++leg) {
        bridge_->spi_data_.q_abad[leg] = motor_pos[leg * 3 + 0];
        bridge_->spi_data_.q_hip[leg]  = motor_pos[leg * 3 + 1];
        bridge_->spi_data_.q_knee[leg] = motor_pos[leg * 3 + 2];
    
        bridge_->spi_data_.qd_abad[leg] = motor_vel[leg * 3 + 0];
        bridge_->spi_data_.qd_hip[leg]  = motor_vel[leg * 3 + 1];
        bridge_->spi_data_.qd_knee[leg] = motor_vel[leg * 3 + 2];
    }
    
    bridge_->Run();
}

void SimulationBridgeInterface::HandleControlParameters() {
    bridge_->HandleControlParameters();
}

void SimulationBridgeInterface::RunRobotControl() {
    bridge_->RunRobotControl();
}