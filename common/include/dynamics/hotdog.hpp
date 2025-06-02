#ifndef CYBERDOG_HPP_
#define CYBERDOG_HPP_

#include "Configuration.h"
#include "dynamics/floating_base_model.hpp"
#include "dynamics/quadruped.hpp"
#include "parameters/model_parameters.hpp"

/**
 * @brief Generate a quadruped model of Hotdog
 *
 * @param robot_type Robot type
 * @return A quadruped model of Hotdog
 */
template < typename T > Quadruped< T > BuildHotdog( const RobotType& robot_type, const RobotAppearanceType& appearance_type = RobotAppearanceType::CURVED ) {
    Quadruped< T >  hotdog;
    ModelParameters model_params;
    if ( robot_type == RobotType::CYBERDOG2 ) {
        if ( appearance_type == RobotAppearanceType::CURVED )
        {
            model_params.InitializeFromYamlFile( THIS_COM "common/config/hotdog2-description.yaml" );
            printf( "laod ************************************************************************hotdog2-description.yaml\n" );
        }
        else
        {
            model_params.InitializeFromYamlFile( THIS_COM "common/config/hotdog2-angular-description.yaml" );
            printf( "laod ************************************************************************hotdog2-angular-description.yaml\n" );
        }
    }
    else
        model_params.InitializeFromYamlFile( THIS_COM "common/config/hotdog-description.yaml" );
    if ( !model_params.IsFullyInitialized() ) {
        printf( "Failed to initialize all model parameters\n" );
        exit( 1 );
    }
    hotdog.robot_type_ = robot_type;

    hotdog.robot_appearance_type_ = appearance_type;

    if ( robot_type == RobotType::CYBERDOG2 ) {
        hotdog.abad_n1_ = 11.5192, hotdog.abad_n2_ = 30.9970, hotdog.abad_n3_ = 60.0;
        hotdog.hip_n1_ = 11.5192, hotdog.hip_n2_ = 30.9970, hotdog.hip_n3_ = 60.0;
        hotdog.knee_n1_ = 11.5192, hotdog.knee_n2_ = 30.9970, hotdog.knee_n3_ = 60.0;
    }
    else {
        hotdog.abad_n1_ = 11.4145, hotdog.abad_n2_ = 20.9136, hotdog.abad_n3_ = 60.0;
        hotdog.hip_n1_ = 11.4145, hotdog.hip_n2_ = 20.9136, hotdog.hip_n3_ = 60.0;
        hotdog.knee_n1_ = 11.4145, hotdog.knee_n2_ = 20.9136, hotdog.knee_n3_ = 60.0;
    }

    hotdog.body_mass_   = model_params.body_mass;        // 3.3
    hotdog.body_length_ = model_params.body_length * 2;  // 0.23536*2;//0.19 * 2;
    hotdog.body_width_  = model_params.body_width * 2;   // 0.05*2;// 0.049 * 2;//

    hotdog.body_height_    = 0.05 * 2;
    hotdog.abad_gear_ratio_ = model_params.abad_gear_ratio;
    hotdog.hip_gear_ratio_  = model_params.hip_gear_ratio;
    hotdog.knee_gear_ratio_    = model_params.knee_gear_ratio;
    hotdog.abad_link_length_   = model_params.abad_link_length;                            // 0.10715;//0.062;//
    hotdog.hip_link_length_    = model_params.hip_link_length;                             // 0.200;//0.211;
    hotdog.knee_link_y_offset_ = 0;                                                      // 0.004;
    hotdog.knee_link_length_   = model_params.knee_link_length + model_params.foot_radius;  // 0.217;//0.20;
    hotdog.max_leg_length_     = ( model_params.hip_link_length + model_params.knee_link_length );

    hotdog.abad_motor_tau_max_  = model_params.abad_torque_max / hotdog.abad_gear_ratio_;
    hotdog.hip_motor_tau_max_   = model_params.hip_torque_max / hotdog.hip_gear_ratio_;
    hotdog.knee_motor_tau_max_  = model_params.knee_torque_max / hotdog.knee_gear_ratio_;
    hotdog.battery_V_         = 24;
    hotdog.motor_KT_          = .05;  // this is flux linkage * pole pairs
    hotdog.motor_R_           = 0.173;
    hotdog.joint_damping_     = .01;
    hotdog.joint_dry_friction_ = .246;

    hotdog.abad_lower_bound_     = model_params.abad_lower_bound;
    hotdog.abad_upper_bound_     = model_params.abad_upper_bound;
    hotdog.front_hip_lower_bound_ = model_params.front_hip_lower_bound;
    hotdog.front_hip_upper_bound_ = model_params.front_hip_upper_bound;
    hotdog.rear_hip_lower_bound_  = model_params.rear_hip_lower_bound;
    hotdog.rear_hip_upper_bound_  = model_params.rear_hip_upper_bound;
    hotdog.knee_lower_bound_     = model_params.knee_lower_bound;
    hotdog.knee_upper_bound_     = model_params.knee_upper_bound;

    // rotor inertia if the rotor is oriented so it spins around the z-axis
    Mat3< T > rotor_inertia_z;
    rotor_inertia_z << model_params.rotor_inertia_z[ 0 ], model_params.rotor_inertia_z[ 1 ], model_params.rotor_inertia_z[ 2 ],
        model_params.rotor_inertia_z[ 3 ], model_params.rotor_inertia_z[ 4 ], model_params.rotor_inertia_z[ 5 ], model_params.rotor_inertia_z[ 6 ],
        model_params.rotor_inertia_z[ 7 ], model_params.rotor_inertia_z[ 8 ];
    Mat3< T > RY                      = CoordinateRotation< T >( CoordinateAxis::Y, M_PI / 2 );
    Mat3< T > RX                      = CoordinateRotation< T >( CoordinateAxis::X, M_PI / 2 );
    Mat3< T > rotorRotationalInertiaX = RY * rotor_inertia_z * RY.transpose();
    Mat3< T > rotorRotationalInertiaY = RX * rotor_inertia_z * RX.transpose();

    // spatial inertias
    Mat3< T > abad_inertia;
    abad_inertia << model_params.abad_inertia[ 0 ], model_params.abad_inertia[ 1 ], model_params.abad_inertia[ 2 ], model_params.abad_inertia[ 3 ],
        model_params.abad_inertia[ 4 ], model_params.abad_inertia[ 5 ], model_params.abad_inertia[ 6 ], model_params.abad_inertia[ 7 ],
        model_params.abad_inertia[ 8 ];
    Vec3< T > abad_com( model_params.abad_com[ 0 ], model_params.abad_com[ 1 ], model_params.abad_com[ 2 ] );
    SpatialInertia< T > abadInertia( model_params.abad_mass, abad_com, abad_inertia );

    Mat3< T > hip_inertia;
    hip_inertia << model_params.hip_inertia[ 0 ], model_params.hip_inertia[ 1 ], model_params.hip_inertia[ 2 ], model_params.hip_inertia[ 3 ],
        model_params.hip_inertia[ 4 ], model_params.hip_inertia[ 5 ], model_params.hip_inertia[ 6 ], model_params.hip_inertia[ 7 ],
        model_params.hip_inertia[ 8 ];
    Vec3< T >           hip_com( model_params.hip_com[ 0 ], model_params.hip_com[ 1 ], model_params.hip_com[ 2 ] );
    SpatialInertia< T > hipInertia( model_params.hip_mass, hip_com, hip_inertia );

    Mat3< T > knee_inertia, kneeRotationalInertiaRotated;
    kneeRotationalInertiaRotated << model_params.knee_inertia[ 0 ], model_params.knee_inertia[ 1 ], model_params.knee_inertia[ 2 ], model_params.knee_inertia[ 3 ],
        model_params.knee_inertia[ 4 ], model_params.knee_inertia[ 5 ], model_params.knee_inertia[ 6 ], model_params.knee_inertia[ 7 ],
        model_params.knee_inertia[ 8 ];
    knee_inertia = RY * kneeRotationalInertiaRotated * RY.transpose();
    Vec3< T >           knee_com( model_params.knee_com[ 0 ], model_params.knee_com[ 1 ], model_params.knee_com[ 2 ] );
    SpatialInertia< T > kneeInertia( model_params.knee_mass, knee_com, knee_inertia );

    Vec3< T >           rotorCOM( 0, 0, 0 );
    SpatialInertia< T > rotorInertiaX( model_params.rotor_mass, rotorCOM, rotorRotationalInertiaX );
    SpatialInertia< T > rotorInertiaY( model_params.rotor_mass, rotorCOM, rotorRotationalInertiaY );

    Mat3< T > body_inertia;
    body_inertia << model_params.body_inertia[ 0 ], model_params.body_inertia[ 1 ], model_params.body_inertia[ 2 ], model_params.body_inertia[ 3 ],
        model_params.body_inertia[ 4 ], model_params.body_inertia[ 5 ], model_params.body_inertia[ 6 ], model_params.body_inertia[ 7 ],
        model_params.body_inertia[ 8 ];
    Vec3< T >           body_com( model_params.body_com[ 0 ], model_params.body_com[ 1 ], model_params.body_com[ 2 ] );
    SpatialInertia< T > bodyInertia( hotdog.body_mass_, body_com, body_inertia );

    hotdog.abad_inertia_      = abadInertia;
    hotdog.hip_inertia_       = hipInertia;
    hotdog.knee_inertia_      = kneeInertia;
    hotdog.abad_rotor_inertia_ = rotorInertiaX;
    hotdog.hip_rotor_inertia_  = rotorInertiaY;
    hotdog.knee_rotor_inertia_ = rotorInertiaY;
    hotdog.body_inertia_      = bodyInertia;

    hotdog.abad_rotor_location_ = Vec3< T >( hotdog.body_length_, hotdog.body_width_, 0 ) * 0.5 + Vec3< T >( -model_params.abad_rotor_location, 0, 0 );
    ;
    hotdog.abad_location_      = Vec3< T >( hotdog.body_length_, hotdog.body_width_, 0 ) * 0.5;
    hotdog.hip_location_       = Vec3< T >( 0, hotdog.abad_link_length_, 0 );
    hotdog.hip_rotor_location_  = Vec3< T >( model_params.abad_rotor_location, hotdog.abad_link_length_ - model_params.hip_rotor_location, 0 );
    hotdog.knee_location_      = Vec3< T >( 0, 0, -hotdog.hip_link_length_ );
    hotdog.knee_rotor_location_ = Vec3< T >( 0, model_params.hip_rotor_location - model_params.knee_rotor_location, 0 );

    // contact points only for c++ code
    hotdog.knee_rubber_       = model_params.knee_rubber;
    hotdog.head_nose_location_ = Vec3< T >( model_params.nose_location[ 0 ], model_params.nose_location[ 1 ], model_params.nose_location[ 2 ] );
    hotdog.head_ear_location_  = Vec3< T >( model_params.ear_location[ 0 ], model_params.ear_location[ 1 ], model_params.ear_location[ 2 ] );
    hotdog.hip_cover_location_ = model_params.hip_cover_location;

    return hotdog;
}

#endif  // CYBERDOG_HPP_
