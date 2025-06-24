#ifndef HXT_HPP_
#define HXT_HPP_

struct motor_ctrl_lcmt {
    float q_des[12];
    float qd_des[12];
    float kp_des[12];
    float kd_des[12];
    float tau_des[12];
};

struct wbc_test_data_t 
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
};

struct sim_command_t
{
    int32_t command_number;
    int32_t data_size;
    std::vector<double> data;
};


struct qp_controller_data_t
{
	double exit_flag;	
	double nWSR;
	double cpu_time_microseconds;
	double xOpt[12];
	double p_des[3];
	double p_act[3];
	double v_des[3];
	double v_act[3];
	double O_err[3];
	double omegab_des[3];
	double omegab_act[3];
	double lbA[20];
	double ubA[20];
	double C_times_f[20];
	double b_control[6];
	double b_control_Opt[6];
	double active;	
	double pfeet_des[12];
	double pfeet_act[12];
};



struct motor_ctrl_state_lcmt {
    int16_t err_flag;
    float ctrl_topic_interval;
};

#endif  // HXT_HPP_
