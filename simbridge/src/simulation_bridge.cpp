/*! @file simulation_bridge.cpp
 *  @brief  The SimulationBridge runs a RobotController and connects it to a
 * Simulator, using shared memory. It is the simulation version of the
 * HardwareBridge.
 */

#include <iostream>

#include "simulation_bridge.hpp"
#include "utilities/segfault_handler.hpp"

/**
 * @brief Connect to a simulation.
 *
 */
void SimulationBridge::Run() {
    iterations_++;
    RunRobotControl();
}

/**
 * @brief This function handles a control parameter message from the simulator.
 *
 */
void SimulationBridge::HandleControlParameters() {
    ControlParameterRequest&  request  = shared_memory_().simToRobot.controlParameterRequest;
    ControlParameterResponse& response = shared_memory_().robotToSim.controlParameterResponse;
    if ( request.requestNumber <= response.requestNumber ) {
        // nothing to do!
        printf( "[SimulationBridge] Warning: the simulator has run a ControlParameter "
                "iteration, but there is no new request!\n" );
        return;
    }

    // sanity check
    u64 num_requests = request.requestNumber - response.requestNumber;
    assert( num_requests == 1 );

    response.nParameters = robot_params_->collection_.map_.size();  // todo don't do this every single time?

    switch ( request.requestKind ) {
    case ControlParameterRequestKind::kSET_ROBOT_PARAM_BY_NAME: {
        std::string       name( request.name );
        ControlParameter& param = robot_params_->collection_.LookUp( name );

        // type check
        if ( param.kind_ != request.parameterKind ) {
            throw std::runtime_error( "type mismatch for parameter " + name + ", robot thinks it is " + ControlParameterValueKindToString( param.kind_ ) + " but received a command to set it to "
                                      + ControlParameterValueKindToString( request.parameterKind ) );
        }

        // do the actual set
        param.Set( request.value, request.parameterKind );

        // respond:
        response.requestNumber = request.requestNumber;  // acknowledge that the set has happened
        response.parameterKind = request.parameterKind;  // just for debugging print statements
        response.value         = request.value;          // just for debugging print statements
        strcpy( response.name,
                name.c_str() );  // just for debugging print statements
        response.requestKind = request.requestKind;

        // printf( "%s\n", response.ToString().c_str() );
        // printf( ".....................................................................................................\n" );


    } break;

    case ControlParameterRequestKind::kSET_USER_PARAM_BY_NAME: {
        std::string name( request.name );
        if ( !user_params_ ) {
            printf( "[Simulation Bridge] Warning: tried to set user parameter, but the robot does not have any!\n" );
        }
        else {
            ControlParameter& param = user_params_->collection_.LookUp( name );

            // type check
            if ( param.kind_ != request.parameterKind ) {
                throw std::runtime_error( "type mismatch for parameter " + name + ", robot thinks it is " + ControlParameterValueKindToString( param.kind_ ) + " but received a command to set it to "
                                          + ControlParameterValueKindToString( request.parameterKind ) );
            }

            // do the actual set
            param.Set( request.value, request.parameterKind );
        }

        // respond:
        response.requestNumber = request.requestNumber;  // acknowledge that the set has happened
        response.parameterKind = request.parameterKind;  // just for debugging print statements
        response.value         = request.value;          // just for debugging print statements
        strcpy( response.name,
                name.c_str() );  // just for debugging print statements
        response.requestKind = request.requestKind;

        // printf( "%s\n", response.ToString().c_str() );
        // printf( "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n" );


    } break;

    case ControlParameterRequestKind::kGET_ROBOT_PARAM_BY_NAME: {
        std::string       name( request.name );
        ControlParameter& param = robot_params_->collection_.LookUp( name );

        // type check
        if ( param.kind_ != request.parameterKind ) {
            throw std::runtime_error( "type mismatch for parameter " + name + ", robot thinks it is " + ControlParameterValueKindToString( param.kind_ ) + " but received a command to set it to "
                                      + ControlParameterValueKindToString( request.parameterKind ) );
        }

        // respond
        response.value         = param.Get( request.parameterKind );
        response.requestNumber = request.requestNumber;  // acknowledge
        response.parameterKind = request.parameterKind;  // just for debugging print statements
        strcpy( response.name,
                name.c_str() );                      // just for debugging print statements
        response.requestKind = request.requestKind;  // just for debugging print statements

        printf( "%s\n", response.ToString().c_str() );
    } break;
    default:
        throw std::runtime_error( "unhandled get/set" );
    }
}

/**
 * @brief Run the robot controller.
 *
 */
void SimulationBridge::RunRobotControl() {
    if ( first_controller_run_ ) {
        printf( "[Simulator Driver] First run of robot controller...\n" );
        robot_params_->InitializeFromYamlFile( THIS_COM "common/config/robot-defaults.yaml" );
        if ( robot_params_->IsFullyInitialized() ) {
            printf( "\tAll %ld control parameters are initialized\n", robot_params_->collection_.map_.size() );
        }
        else {
            printf( "\tbut not all control parameters were initialized. Missing:\n%s\n", robot_params_->GenerateUnitializedList().c_str() );
            throw std::runtime_error( "not all parameters initialized when going into kRunContorller" );
        }
        user_params_->InitializeFromYamlFile( THIS_COM "common/config/hotdog2-ctrl-user-parameters.yaml" );
        
        if ( !user_params_->IsFullyInitialized() ) {
            printf( "\tAll %ld user parameters are initialized\n", user_params_->collection_.map_.size() );
        }
        printf( "888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888\n" );

        // auto* userControlParameters = robot_runner_->robot_ctrl_->GetUserControlParameters();
        auto* userControlParameters = robot_runner_->GetUserControlParameters();
        if ( userControlParameters ) {
            if ( userControlParameters->IsFullyInitialized() ) {
                printf( "\tAll %ld user parameters are initialized\n", userControlParameters->collection_.map_.size() );
                sim_mode_ = SimulatorMode::kRunContorller;
            }
            else {
                printf( "\tbut not all control parameters were initialized. Missing:\n%s\n", userControlParameters->GenerateUnitializedList().c_str() );
                throw std::runtime_error( "not all parameters initialized when going into kRunContorller" );
            }
        }
        else {
            sim_mode_ = SimulatorMode::kRunContorller;
        }

        interface_lcm_thread_cmd_          = std::thread( &SimulationBridge::HandleInterfaceLCM_cmd, this );
        interface_lcm_thread_hotdog_cmd_ = std::thread( &SimulationBridge::HandleInterfaceLCM_hotdog_cmd, this );
        interface_lcm_thread_motion_list_  = std::thread( &SimulationBridge::HandleInterfaceLCM_motion_list, this );
        interface_lcm_thread_usergait_     = std::thread( &SimulationBridge::HandleInterfaceLCM_usergait_file, this );
        interface_motor_control_thread_    = std::thread( &SimulationBridge::HandleMotorCtrlLcmThread, this );

        robot_runner_->SetCommandInterface( &cmd_interface_ );
        robot_runner_->SetSpiData( &spi_data_  );
        robot_runner_->SetSpiCommand( &spi_command_);
        robot_runner_->SetRobotType( robot_ );
        robot_runner_->SetRobotAppearanceType( RobotAppearanceType::CURVED );
        robot_runner_->SetVectorNavData( &vector_nav_data_ );
        // robot_runner_->SetCheaterState( &shared_memory_().simToRobot.cheater_state );
        robot_runner_->SetRobotControlParameters( robot_params_ );
        // robot_runner_->SetVisualizationData( &visualization_data_);
        // robot_runner_->SetHotdog2Visualization( &hotdog2_visualization_ );

        robot_runner_->Init();
        first_controller_run_ = false;
    }
    cmd_interface_.ProcessGamepadCommand( gamepadCommand);

    robot_runner_->Run();
    // robot_runner_->LCMPublishByThread();
}

void SimulationBridge::HotdogLcmCmdCallback( const lcm::ReceiveBuffer* buf, const std::string& channel, const motion_control_request_lcmt* msg ) {
    ( void )buf;
    ( void )channel;
    ( void )msg;
    cmd_interface_.ProcessHotdogLcmCommand( msg );
}

void SimulationBridge::UserGaitFileCallback( const lcm::ReceiveBuffer* buf, const std::string& channel, const file_send_lcmt* msg ) {
    ( void )buf;
    ( void )channel;
    file_recv_lcmt receive_msg_;
    receive_msg_.result = cmd_interface_.ProcessUserGaitFile( msg );
    user_gait_file_responce_lcm_.publish( "user_gait_result", &receive_msg_ );
}

void SimulationBridge::LcmCmdCallback( const lcm::ReceiveBuffer* buf, const std::string& channel, const robot_control_cmd_lcmt* msg ) {
    ( void )buf;
    ( void )channel;
    ( void )msg;
    cmd_interface_.ProcessLcmCommand( msg );
}

void SimulationBridge::LcmMotionCallback( const lcm::ReceiveBuffer* buf, const std::string& channel, const trajectory_command_lcmt* msg ) {
    ( void )buf;
    ( void )channel;
    cmd_interface_.ProcessLcmMotionCommand( msg );
}

void SimulationBridge::LcmMotorCtrlCallback( const lcm::ReceiveBuffer* rbuf, const std::string& chan, const motor_ctrl_lcmt* msg ) {
    ( void )rbuf;
    ( void )chan;
    cmd_interface_.ProcessLcmMotorCtrlCommand( msg );
}

void SimulationBridge::HandleInterfaceLCM_cmd() {
    while ( !interface_lcm_quit_ ) {
        lcm_.handle();
    }
}

void SimulationBridge::HandleInterfaceLCM_hotdog_cmd() {
    while ( !interface_lcm_quit_ ) {
        hotdog_lcm_.handle();
    }
}

void SimulationBridge::HandleInterfaceLCM_motion_list() {
    while ( !interface_lcm_quit_ ) {
        motion_list_lcm_.handle();
    }
}

void SimulationBridge::HandleInterfaceLCM_usergait_file() {
    while ( !interface_lcm_quit_ ) {
        user_gait_file_lcm_.handle();
    }
}
void SimulationBridge::HandleMotorCtrlLcmThread() {
    while ( !interface_lcm_quit_ ) {
        motor_control_lcm_.handle();
    }
}