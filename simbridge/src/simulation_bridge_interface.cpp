#include "simulation_bridge_interface.hpp"
#include "simulation_bridge.hpp"

SimulationBridgeInterface::SimulationBridgeInterface( RobotType robot, RobotController* robot_ctrl ) {
    bridge_ = new SimulationBridge( robot, robot_ctrl );
}

SimulationBridgeInterface::~SimulationBridgeInterface() {
    delete bridge_;
}

void SimulationBridgeInterface::Run(const float* motor_pos, const float* motor_vel, const float* motor_tor, const float* quat, const float* gyro, const float* accl, SpiCommand &spi_command_,JoyData &joydata)
{


    bridge_->gamepadCommand.a = joydata.buttons[2];//t
    bridge_->gamepadCommand.b = joydata.buttons[3];//y
    bridge_->gamepadCommand.x = joydata.buttons[0];//e
    bridge_->gamepadCommand.y = joydata.buttons[1];//r

    for(int leg = 0; leg < 4; ++leg) 
    {
        bridge_->spi_data_.q_abad[leg] = motor_pos[leg * 3 + 0];
        bridge_->spi_data_.q_hip[leg]  = -motor_pos[leg * 3 + 1];
        bridge_->spi_data_.q_knee[leg] = -motor_pos[leg * 3 + 2];

        bridge_->spi_data_.qd_abad[leg] = motor_vel[leg * 3 + 0];
        bridge_->spi_data_.qd_hip[leg]  = -motor_vel[leg * 3 + 1];
        bridge_->spi_data_.qd_knee[leg] = -motor_vel[leg * 3 + 2];

        bridge_->spi_data_.tau_abad[leg] = motor_tor[leg * 3 + 0];
        bridge_->spi_data_.tau_hip[leg]  = motor_tor[leg * 3 + 1];
        bridge_->spi_data_.tau_knee[leg] = motor_tor[leg * 3 + 2];




    }

        bridge_->vector_nav_data_.quat[0] = quat[1];
        bridge_->vector_nav_data_.quat[1] = quat[2];
        bridge_->vector_nav_data_.quat[2] = quat[3];
        bridge_->vector_nav_data_.quat[3] = quat[0];
    
    for (size_t i = 0; i < 3; i++)
    {
        bridge_->vector_nav_data_.gyro[i] = gyro[i];

        bridge_->vector_nav_data_.accelerometer[i] = accl[i];

    }
    

    bridge_->Run();

    //To do:send motors message 
    for (int leg = 0; leg < 4; ++leg) {
        spi_command_.q_des_abad[leg]   = bridge_->spi_command_.q_des_abad[leg];
        spi_command_.q_des_hip[leg]    = bridge_->spi_command_.q_des_hip[leg];
        spi_command_.q_des_knee[leg]   = bridge_->spi_command_.q_des_knee[leg];

        spi_command_.qd_des_abad[leg]  = bridge_->spi_command_.qd_des_abad[leg];
        spi_command_.qd_des_hip[leg]   = bridge_->spi_command_.qd_des_hip[leg];
        spi_command_.qd_des_knee[leg]  = bridge_->spi_command_.qd_des_knee[leg];

        spi_command_.kp_abad[leg]      = bridge_->spi_command_.kp_abad[leg];
        spi_command_.kp_hip[leg]       = bridge_->spi_command_.kp_hip[leg];
        spi_command_.kp_knee[leg]      = bridge_->spi_command_.kp_knee[leg];

        spi_command_.kd_abad[leg]      = bridge_->spi_command_.kd_abad[leg];
        spi_command_.kd_hip[leg]       = bridge_->spi_command_.kd_hip[leg];
        spi_command_.kd_knee[leg]      = bridge_->spi_command_.kd_knee[leg];

        spi_command_.tau_abad_ff[leg]  = bridge_->spi_command_.tau_abad_ff[leg];
        spi_command_.tau_hip_ff[leg]   = bridge_->spi_command_.tau_hip_ff[leg];
        spi_command_.tau_knee_ff[leg]  = bridge_->spi_command_.tau_knee_ff[leg];

        spi_command_.flags[leg]        = bridge_->spi_command_.flags[leg];
    }
}

void SimulationBridgeInterface::HandleControlParameters() {
    bridge_->HandleControlParameters();
}

void SimulationBridgeInterface::RunRobotControl() {
    bridge_->RunRobotControl();
}