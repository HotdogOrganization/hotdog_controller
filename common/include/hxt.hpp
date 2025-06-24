#ifndef HXT_HPP_
#define HXT_HPP_

struct motor_ctrl_lcmt {
    float q_des[12];
    float qd_des[12];
    float kp_des[12];
    float kd_des[12];
    float tau_des[12];
};

#endif  // HXT_HPP_
