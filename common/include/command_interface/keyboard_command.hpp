#ifndef KEYBOARD_COMMAND_HPP_
#define KEYBOARD_COMMAND_HPP_

#include "utilities/utilities.hpp"
#include "cpp_types.hpp"
// #include "header/lcm_type/gamepad_lcmt.hpp"

/**
 * @brief The state of the gamepad
 * 
 */
struct KeyboardCommand {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Construct a gamepad and set to zero.
   * 
   */
  KeyboardCommand() {
    zero();
  }

  bool recoveryStandButton, balanceStandButton, locomotionButton, pureDumpButton;
  float xVel, yVel, yawVel;  // x,y, yaw velocity command
  float rollPos, pitchPos, yawPos;  // roll pitch yaw position command
  float heightPos;
  /**
   * @brief Set all values to zero
   * 
   */
  void zero() {
    recoveryStandButton   = false;
    balanceStandButton    = false;
    locomotionButton      = false;
    pureDumpButton        = false;

    xVel   = 0.0f;
    yVel   = 0.0f;
    yawVel = 0.0f;

    rollPos  = 0.0f;
    pitchPos = 0.0f;
    yawPos   = 0.0f;

    heightPos = 0.0f;
  }


  /**
   * @brief Represent as human-readable string.
   * @return string representing state
   */
  std::string ToString() const {
    std::ostringstream oss;
    oss << "recoveryStandButton: " << BoolToString(recoveryStandButton) << "\n"
        << "balanceStandButton: " << BoolToString(balanceStandButton) << "\n"
        << "locomotionButton: " << BoolToString(locomotionButton) << "\n"
        << "pureDumpButton: " << BoolToString(pureDumpButton) << "\n"
        << "xVel: " << xVel << "\n"
        << "yVel: " << yVel << "\n"
        << "yawVel: " << yawVel << "\n"
        << "rollPos: " << rollPos << "\n"
        << "pitchPos: " << pitchPos << "\n"
        << "yawPos: " << yawPos << "\n"
        << "heightPos: " << heightPos << "\n";
    return oss.str();
  }
};

#endif  // KEYBOARD_COMMAND_HPP_
