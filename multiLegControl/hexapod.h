#ifndef HEXAPODLEG_H
#define HEXAPODLEG_H

#include "../modules/custom/arduinoConnection/arduinoController.h"
#include "../modules/custom/rsTimedLoop/rsTimedLoop.h"
#include "../modules/custom/raisimSimulatorLeg/raisimSimulator.h"
#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>


using namespace std;

class Hexapod
{
public:
    Hexapod(unsigned int id, unique_ptr<ArduinoController> arduino, RSTimedLoop& rsLoop, bool arduinoConnected, bool raisimSimulator, float rsStep, Path binaryPath);
    ~Hexapod();

    Eigen::MatrixXd getJacobian(int legNum) const;
    Eigen::Vector3d doLegIK(float x, float y, float z) const;
    Eigen::Vector3d doBodyIK(float x, float y, float z) const;
    void updatePos();
    void printPos() const;

    void setAngs(float coxa, float femur, float tibia);
    void setAngs(const Eigen::VectorXd& angs);

    void moveLegToPos(const Eigen::Vector3d& desiredPos, const int legNum);
    void moveLegsToPos(const Eigen::VectorXd& desiredPos);


    void jumpToZero();
    void jumpToBasic();
    void jumpToOff();
    void jumpToCurled();

    void stand();

    void doJacobianTest(const int &style);
    // void doLegIKTest();
    // void doBodyIKTest();

    int id;
    Eigen::VectorXd pos;
    Eigen::VectorXd currentAngles;
    Eigen::Vector3d currentAngularVelocities;

private:
    float rsStep;
    void sendAngs();
    void sendPos(float x, float y, float z);
    
    unique_ptr<ArduinoController> arduino;
    RSTimedLoop& rsLoop;

    bool arduinoConnected;

    unique_ptr<RaisimSimulator> simulator;

    constexpr static float coxaX = 0.044925;
    constexpr static float coxaZ = 0.01065;
    constexpr static float femurX = 0.118314;
    constexpr static float tibiaX = 0.221426;

    constexpr static int coxaAngleOffset = 6;
    constexpr static int femurAngleOffset = 0;
    constexpr static int tibiaAngleOffset = -73;

    constexpr static int coxaAngleInit = 96;
    constexpr static int femurAngleInit = 94;
    constexpr static int tibiaAngleInit = 17;

    const vector<int> angleInits = {96, 94, 17};

    const vector<float> bodyLegOffsets = {0.180, 0.130, 0.180, 0.180, 0.130, 0.180};
    const vector<float> femurRotation= {M_PI_2, M_PI_2, M_PI_2, -M_PI_2, -M_PI_2, -M_PI_2};
    const vector<float> bodyLegAngles = {57*M_PI/180, 0*M_PI/180, -57*M_PI/180, -123*M_PI/180, 180*M_PI/180, 123*M_PI/180};

    const vector<bool> legStatus = {0,1,0,1,0,1};

    constexpr static int centreLegOffset = 130;
    constexpr static int diagLegOffset = 180;
    constexpr static int diagLegOffsetAngle = 57;
};

#endif