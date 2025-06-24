#ifndef ROBOT_RUNNER_INTERFACE_HPP_
#define ROBOT_RUNNER_INTERFACE_HPP_

#include "robot_controller.hpp"
#include "robot_runner.hpp"
// #include "header/lcm_type/localization_lcmt.hpp"

class RobotRunner;
class PeriodicTaskManager;

class RobotRunnerInterface {
public:
    RobotRunnerInterface( RobotController* controller, PeriodicTaskManager* taskManager, float period, std::string name );
    ~RobotRunnerInterface();

    void Init();
    void Start();
    void Run();
    void Cleanup();

    void InitializeStateEstimator( bool cheaterMode = false );

    ControlParameters* GetUserControlParameters();
    bool*              GetLowPowerEnable();

    // void LCMPublishByThread();

    void SetCommandInterface( CommandInterface* value );
    void SetSpiData( SpiData* value );
    void SetSpiCommand( SpiCommand* value );
    void SetRobotType( RobotType value );
    void SetRobotAppearanceType( RobotAppearanceType value );
    void SetVectorNavData( VectorNavData* value );
    void SetRobotControlParameters( RobotControlParameters* value );
    void SetVisualizationData( VisualizationData* value );
    void SetHotdog2Visualization( Hotdog2Visualization* value );
    void SetCheaterState( CheaterState< double >* value );
    void SetBmsStatus( int8_t* value );
    void SetBattSoc( int8_t* value );

    // const localization_lcmt& GetGlobalToRobotLcm() const {
    //     if (!robot_runner_) {
    //         throw std::runtime_error("RobotRunner is not initialized.");
    //     }
    //     return robot_runner_->GetGlobalToRobotLcm();
    // }

private:
    RobotRunner* robot_runner_;
};

#endif  // ROBOT_RUNNER_INTERFACE_HPP_
