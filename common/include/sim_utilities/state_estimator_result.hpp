#ifndef STATE_ESTIMATOR_RESULT_HPP_
#define STATE_ESTIMATOR_RESULT_HPP_

#include "cpp_types.hpp"

struct StateEstimateData
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3f position;
  Eigen::Vector3f vWorld;
  Eigen::Vector3f vBody;
  Eigen::Vector3f p_abs;
  Eigen::Vector3f vWorld_abs;
  Eigen::Vector3f vBody_abs;
  Eigen::Vector3f vRemoter;
  Eigen::Vector3f rpy;
  Eigen::Vector3f omegaBody;
  Eigen::Vector3f omegaWorld;
  Eigen::Quaternionf quat;
  Eigen::Vector3f aBody;
  Eigen::Vector3f aWorld;
  Eigen::Vector4f contactEstimate;

  int64_t timestamp;
};

/**
 * @brief Result of state estimation
 *
 */
template < typename T > struct StateEstimatorResult
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Vec3< T >   position;
  Quat< T >   orientation;
  Vec3< T >   rpy;
  Vec3< T >   velocity_in_body_frame;
  Vec3< T >   angular_velocity_in_body_frame;
  Vec3< T >   acceleration_in_body_frame;
  RotMat< T > world2body_rotation_matrix;
  Vec4< T >   contact;
  Vec4< T >   footforce_contact;

  Vec3< T > velocity_in_world_frame;
  Vec3< T > angular_velocity_in_world_frame;
  Vec3< T > acceleration_in_world_frame;
  Vec3< T > remoter_velocity;
  Vec3< T > terrain_coefficient;
  Mat3< T > terrain_rotation_matrix;

  bool   is_battery_low;
  bool   is_charging;
  int8_t battery_soc;

  T         height;
  Vec3< T > absolute_position;
  Vec3< T > absolute_velocity_in_body_frame;
  Vec3< T > absolute_velocity_in_world_frame;

  explicit StateEstimatorResult() {
    memset( ( void* )this, 0, sizeof( StateEstimatorResult< T > ) );
  }

  void ToStateEstimateData(StateEstimateData& state_data) const {
    state_data.position = position.template cast<float>();
    state_data.vWorld = velocity_in_world_frame.template cast<float>();
    state_data.vBody = velocity_in_body_frame.template cast<float>();
    state_data.p_abs = absolute_position.template cast<float>();
    state_data.vWorld_abs = absolute_velocity_in_world_frame.template cast<float>();
    state_data.vBody_abs = absolute_velocity_in_body_frame.template cast<float>();
    state_data.rpy = rpy.template cast<float>();
    state_data.omegaBody = angular_velocity_in_body_frame.template cast<float>();
    state_data.omegaWorld = angular_velocity_in_world_frame.template cast<float>();
    state_data.vRemoter = remoter_velocity.template cast<float>();
    state_data.aBody = acceleration_in_body_frame.template cast<float>();
    state_data.aWorld = acceleration_in_world_frame.template cast<float>();
    state_data.quat = Eigen::Quaternionf(
        static_cast<float>(orientation.w()),
        static_cast<float>(orientation.x()),
        static_cast<float>(orientation.y()),
        static_cast<float>(orientation.z()));
    state_data.contactEstimate = contact.template cast<float>();
    // Optionally set timestamp if available
    // state_data.timestamp = ...;
  }
};

using StateEstimatorResultf = StateEstimatorResult<float>;
using StateEstimatorResultd = StateEstimatorResult<double>;

#endif  // STATE_ESTIMATOR_RESULT_HPP_
