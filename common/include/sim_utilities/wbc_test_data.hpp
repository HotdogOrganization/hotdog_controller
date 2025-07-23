#ifndef WBC_TEST_DATA_HPP_
#define WBC_TEST_DATA_HPP_

#include <cstring>

#include "c_types.h"

struct WbcTestData
{
    int32_t contact_est[4];

    float Fr_des[12];
    float Fr[12];

    float body_ori_cmd[4];
    float body_pos_cmd[3];
    float body_vel_cmd[3];
    float body_ang_vel_cmd[3];

    float body_pos[3];
    float body_vel[3];

    float body_ori[4];
    float body_ang_vel[3];

    float foot_pos_cmd[12];
    float foot_vel_cmd[12];
    float foot_acc_cmd[12];
    float foot_acc_numeric[12];
    
    float foot_pos[12];
    float foot_vel[12];

    float foot_local_pos[12];
    float foot_local_vel[12];

    float jpos_cmd[12];
    float jvel_cmd[12];
    float jacc_cmd[12];

    float jpos[12];
    float jvel[12];

    float vision_loc[3];

    std::string ToString() const {
        std::ostringstream oss;
        oss << "contact_est: ";
        for (int i = 0; i < 4; ++i) oss << contact_est[i] << " ";
        oss << "\nFr_des: ";
        for (int i = 0; i < 12; ++i) oss << Fr_des[i] << " ";
        oss << "\nFr: ";
        for (int i = 0; i < 12; ++i) oss << Fr[i] << " ";
        oss << "\nbody_ori_cmd: ";
        for (int i = 0; i < 4; ++i) oss << body_ori_cmd[i] << " ";
        oss << "\nbody_pos_cmd: ";
        for (int i = 0; i < 3; ++i) oss << body_pos_cmd[i] << " ";
        oss << "\nbody_vel_cmd: ";
        for (int i = 0; i < 3; ++i) oss << body_vel_cmd[i] << " ";
        oss << "\nbody_ang_vel_cmd: ";
        for (int i = 0; i < 3; ++i) oss << body_ang_vel_cmd[i] << " ";
        oss << "\nbody_pos: ";
        for (int i = 0; i < 3; ++i) oss << body_pos[i] << " ";
        oss << "\nbody_vel: ";
        for (int i = 0; i < 3; ++i) oss << body_vel[i] << " ";
        oss << "\nbody_ori: ";
        for (int i = 0; i < 4; ++i) oss << body_ori[i] << " ";
        oss << "\nbody_ang_vel: ";
        for (int i = 0; i < 3; ++i) oss << body_ang_vel[i] << " ";
        oss << "\nfoot_pos_cmd: ";
        for (int i = 0; i < 12; ++i) oss << foot_pos_cmd[i] << " ";
        oss << "\nfoot_vel_cmd: ";
        for (int i = 0; i < 12; ++i) oss << foot_vel_cmd[i] << " ";
        oss << "\nfoot_acc_cmd: ";
        for (int i = 0; i < 12; ++i) oss << foot_acc_cmd[i] << " ";
        oss << "\nfoot_acc_numeric: ";
        for (int i = 0; i < 12; ++i) oss << foot_acc_numeric[i] << " ";
        oss << "\nfoot_pos: ";
        for (int i = 0; i < 12; ++i) oss << foot_pos[i] << " ";
        oss << "\nfoot_vel: ";
        for (int i = 0; i < 12; ++i) oss << foot_vel[i] << " ";
        oss << "\nfoot_local_pos: ";
        for (int i = 0; i < 12; ++i) oss << foot_local_pos[i] << " ";
        oss << "\nfoot_local_vel: ";
        for (int i = 0; i < 12; ++i) oss << foot_local_vel[i] << " ";
        oss << "\njpos_cmd: ";
        for (int i = 0; i < 12; ++i) oss << jpos_cmd[i] << " ";
        oss << "\njvel_cmd: ";
        for (int i = 0; i < 12; ++i) oss << jvel_cmd[i] << " ";
        oss << "\njacc_cmd: ";
        for (int i = 0; i < 12; ++i) oss << jacc_cmd[i] << " ";
        oss << "\njpos: ";
        for (int i = 0; i < 12; ++i) oss << jpos[i] << " ";
        oss << "\njvel: ";
        for (int i = 0; i < 12; ++i) oss << jvel[i] << " ";
        oss << "\nvision_loc: ";
        for (int i = 0; i < 3; ++i) oss << vision_loc[i] << " ";
        oss << std::endl;
        return oss.str();
    }

};

#endif  // WBC_TEST_DATA_HPP_
