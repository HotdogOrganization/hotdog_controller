#ifndef RC_COMMAND_HPP_
#define RC_COMMAND_HPP_

#include "utilities/utilities.hpp"
#include "cpp_types.hpp"

/**
 * @brief The state of the gamepad
 * 
 */
struct RcCommand {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief Construct a gamepad and set to zero.
     * 
     */
    RcCommand() {
        zero();
    }

    Vec2< float > leftStickAnalog, rightStickAnalog;
    float wheelAnalog;
    bool L1, L2, R1, R2, B1, B2;


    /**
     * @brief Set all values to zero
     * 
     */
    void zero() {
        leftStickAnalog.setZero();
        rightStickAnalog.setZero();
        wheelAnalog = 0.0f;
        L1 = L2 = R1 = R2 = B1 = B2 = false;
    }

    /**
     * @brief Represent as human-readable string.
     * @return string representing state
     */
    std::string ToString() {
        std::string result = "RcCommand State:\n";
        result += "leftStickAnalog: " + EigenToString(leftStickAnalog) + "\n";
        result += "rightStickAnalog: " + EigenToString(rightStickAnalog) + "\n";
        result += "wheelAnalog: " + std::to_string(wheelAnalog) + "\n";
        result += "L1: " + BoolToString(L1) + "\n";
        result += "L2: " + BoolToString(L2) + "\n";
        result += "R1: " + BoolToString(R1) + "\n";
        result += "R2: " + BoolToString(R2) + "\n";
        result += "B1: " + BoolToString(B1) + "\n";
        result += "B2: " + BoolToString(B2) + "\n";
        return result;
    }
};

#endif  // RC_COMMAND_HPP_
