#ifndef HEXAPODCONTROL_H
#define HEXAPODCONTROL_H

#ifdef USE_SIMULATOR
    #include "../modules/custom/raisimSimulatorFull/raisimSimulator.h"
    #include <raisim/Path.hpp>  // Include Path from raisim only if simulator is enabled
#endif

#include "../modules/custom/arduinoConnection/arduinoController.h"
#include "../modules/custom/rsTimedLoop/rsTimedLoop.h"
#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>


using namespace std;

class HexapodControl
{
public:
    #ifdef USE_SIMULATOR
        HexapodControl(unsigned int id, unique_ptr<ArduinoController> arduino, RSTimedLoop& rsLoop, bool arduinoConnected, bool raisimSimulator, float rsStep, Path binaryPath);
    #else
        HexapodControl(unsigned int id, unique_ptr<ArduinoController> arduino, RSTimedLoop& rsLoop, bool arduinoConnected, bool raisimSimulator, float rsStep);
    #endif
    ~HexapodControl();

    Eigen::MatrixXd getJacobian(int legNum) const;

    void updatePos();
    Eigen::VectorXd doFK(Eigen::VectorXd angs);
    void printPos() const;

    bool active;
    int operationDuration = 0; 

    // Instant set position
    void setAngs(const Eigen::VectorXd& angs);

    // Velocity set position
    void moveLegToPos(const Eigen::Vector3d& desiredPos, const int legNum);
    void moveLegsToPos(const Eigen::VectorXd& desiredPos, float dur);


    // Instant positions
    void jumpToZero();
    void jumpToBasic();
    void jumpToOff();
    void jumpToCurled();

    // Velocity positions
    void stand();
    void moveToStand(float dur);
    void moveToOff();

    // Velocity Path
    void jacobianTest(const int &style);
    void walk(double, double);

    bool directed = 0;

    int id;
    Eigen::VectorXd pos;
    Eigen::VectorXd currentAngles;
    Eigen::VectorXd currentAngularVelocities;
    Eigen::VectorXd desiredAngles;

    double moveVectorMag;
    RSTimedLoop& rsLoop;
    
    #ifdef USE_SIMULATOR
        unique_ptr<RaisimSimulator> simulator;  // Simulator object
    #endif

    // Walk Params
    double minStepDuration = 8000;
    double jacDuration = 5000;
    double standDuration = 1500;

private:
    float rsStep;
    void sendAngs();
    
    // Arduino object params
    unique_ptr<ArduinoController> arduino;
    bool arduinoConnected;

    /*** Stand Params */
    double height = -0.14; // Control height off of the ground
    double stepOff = 0.2; // controls foot offset from coxa rotation
    double legAng = 90; // Control control angle of leg relative to world frame
    double flush = 0; // controle

    /*** Step Params */
    double stepRadius = 0.08;
    double stepHeight = 0.2;

    double standPos[18];
    double standIntermediatePos[18];

    // Leg Dims
    constexpr static float coxaX = 0.044924;
    constexpr static float coxaZ = 0.010656;
    constexpr static float femurX = 0.118313;
    constexpr static float tibiaX = 0.221170;

    // Leg Information
    const vector<int> angleInits = {90, 45, 17, 90, 45, 17, 90, 45, 17, 90, 45, 17, 90, 45, 17, 90, 45, 17};
    const vector<double> bodyLegStandAngs = {legAng*M_PI/180, 0*M_PI/180, -legAng*M_PI/180, (-180+legAng)*M_PI/180, 180*M_PI/180, (180-legAng)*M_PI/180};
    const vector<float> bodyLegOffsets = {0.180, 0.130, 0.180, 0.180, 0.130, 0.180};
    const vector<float> femurRotation= {M_PI_2, M_PI_2, M_PI_2, -M_PI_2, -M_PI_2, -M_PI_2};
    const vector<float> bodyLegAngles = {57*M_PI/180, 0*M_PI/180, -57*M_PI/180, -123*M_PI/180, 180*M_PI/180, 123*M_PI/180};

    // Walking sequence controls
    double lastAngle = 0;
    vector<bool> standing = {0,1,0,1,0,1};
};

#endif