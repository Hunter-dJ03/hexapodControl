#include "hexapod.h"

#include <iostream>
#include <fmt/core.h>
#include <math.h>
#include <chrono>
#include <thread>
#include <cmath>
#include <bitset>

using namespace std;
using namespace this_thread; // sleep_for, sleep_until
using chrono::system_clock;

#ifdef USE_SIMULATOR
// Hexapod Class constructor
HexapodControl::HexapodControl(unsigned int id, std::unique_ptr<ArduinoController> arduino, RSTimedLoop &rsLoop, bool arduinoConnected, bool raisimSimulator, float rsStep, Path binaryPath)
    : id(id), arduino(move(arduino)), rsLoop(rsLoop), arduinoConnected(arduinoConnected), rsStep(rsStep)
{
    // If simulator is chosen then create a raisim simulator
    simulator = make_unique<RaisimSimulator>(rsStep, binaryPath, "hexapodCustom.urdf");

    // Initialise VectorXd sizes for angles and positions
    currentAngles = Eigen::VectorXd(18);
    pos = Eigen::VectorXd(18);
    desiredAngles = Eigen::VectorXd(18);
    currentAngularVelocities = Eigen::VectorXd(18);

    currentAngularVelocities.setZero();
    
    // Desired angles array for each leg
    float baseAngs[3] = {0, 2.35619, 3.52557};

    // Move angle arrays to eigen vector
    for (int i = 0; i < 18; ++i)
    {
        currentAngles[i] = baseAngs[i % 3]; // Repeat the set of 3 angles
    }

    desiredAngles = currentAngles;

    // Set hexapod active level (deployed or not)
    active = false;

    // Jump to starting curled position
    // jumpToCurled();

    simulator->setSimAngle(currentAngles);
}
#else

HexapodControl::HexapodControl(unsigned int id, std::unique_ptr<ArduinoController> arduino, RSTimedLoop &rsLoop, bool arduinoConnected, bool raisimSimulator, float rsStep)
    : id(id), arduino(move(arduino)), rsLoop(rsLoop), arduinoConnected(arduinoConnected), rsStep(rsStep)
{
    // Initialise VectorXd sizes for angles and positions
    currentAngles = Eigen::VectorXd(18);
    pos = Eigen::VectorXd(18);
    desiredAngles = Eigen::VectorXd(18);
    currentAngularVelocities = Eigen::VectorXd(18);

    currentAngularVelocities.setZero();
    
    // Desired angles array for each leg
    float baseAngs[3] = {0, 2.35619, 3.52557};

    // Move angle arrays to eigen vector
    for (int i = 0; i < 18; ++i)
    {
        currentAngles[i] = baseAngs[i % 3]; // Repeat the set of 3 angles
    }

    desiredAngles = currentAngles;

    // Set hexapod active level (deployed or not)
    active = false;
}
#endif

// Class destructor
HexapodControl::~HexapodControl()
{
    // Reset arduino unique pointer
    if (arduino)
    {
        arduino.reset();
    }
    // Reset simulator unique pointer
    #ifdef USE_SIMULATOR
        simulator.reset();
    #endif
}

// Perform jacobian mathematics based on current angle and input legnum
Eigen::MatrixXd HexapodControl::getJacobian(int legNum) const
{
    // Blank mmatric of desired size
    Eigen::MatrixXd Jac(3, 3);

    // Elemetnal operations
    Jac(0, 0) = -sin(bodyLegAngles[legNum] + currentAngles[legNum * 3 + 0]) * (coxaX + tibiaX * cos(currentAngles[legNum * 3 + 1] + currentAngles[legNum * 3 + 2]) + femurX * cos(currentAngles[legNum * 3 + 1]));
    Jac(0, 1) = -cos(bodyLegAngles[legNum] + currentAngles[legNum * 3 + 0]) * (tibiaX * sin(currentAngles[legNum * 3 + 1] + currentAngles[legNum * 3 + 2]) + femurX * sin(currentAngles[legNum * 3 + 1]));
    Jac(0, 2) = -tibiaX * cos(bodyLegAngles[legNum] + currentAngles[legNum * 3 + 0]) * sin(currentAngles[legNum * 3 + 1] + currentAngles[legNum * 3 + 2]);
    Jac(1, 0) = cos(bodyLegAngles[legNum] + currentAngles[legNum * 3 + 0]) * (coxaX + tibiaX * cos(currentAngles[legNum * 3 + 1] + currentAngles[legNum * 3 + 2]) + femurX * cos(currentAngles[legNum * 3 + 1]));
    Jac(1, 1) = -sin(bodyLegAngles[legNum] + currentAngles[legNum * 3 + 0]) * (tibiaX * sin(currentAngles[legNum * 3 + 1] + currentAngles[legNum * 3 + 2]) + femurX * sin(currentAngles[legNum * 3 + 1]));
    Jac(1, 2) = -tibiaX * sin(bodyLegAngles[legNum] + currentAngles[legNum * 3 + 0]) * sin(currentAngles[legNum * 3 + 1] + currentAngles[legNum * 3 + 2]);
    Jac(2, 0) = 0;
    Jac(2, 1) = tibiaX * cos(currentAngles[legNum * 3 + 1] + currentAngles[legNum * 3 + 2]) + femurX * cos(currentAngles[legNum * 3 + 1]);
    Jac(2, 2) = tibiaX * cos(currentAngles[legNum * 3 + 1] + currentAngles[legNum * 3 + 2]);

    return Jac;
}

// Perform forward kinematics and update the class position variable
void HexapodControl::updatePos()
{
    pos = doFK(currentAngles);
}

// Perform forward kinematics of the Hexapod to get end affector positions for set angles
Eigen::VectorXd HexapodControl::doFK(Eigen::VectorXd angs)
{
    // Blank position element for outpur
    Eigen::VectorXd position(18);

    // Perform forward kinematics per leg and save to position
    for (int legNum = 0; legNum < 6; legNum++)
    {

        position(legNum * 3) = coxaX * cos(bodyLegAngles[legNum] + angs[legNum * 3 + 0]) + bodyLegOffsets[legNum] * cos(bodyLegAngles[legNum]) + tibiaX * cos(bodyLegAngles[legNum] + angs[legNum * 3 + 0]) * cos(angs[legNum * 3 + 1] + angs[legNum * 3 + 2]) + femurX * cos(bodyLegAngles[legNum] + angs[legNum * 3 + 0]) * cos(angs[legNum * 3 + 1]);
        position(legNum * 3 + 1) = coxaX * sin(bodyLegAngles[legNum] + angs[legNum * 3 + 0]) + bodyLegOffsets[legNum] * sin(bodyLegAngles[legNum]) + tibiaX * cos(angs[legNum * 3 + 1] + angs[legNum * 3 + 2]) * sin(bodyLegAngles[legNum] + angs[legNum * 3 + 0]) + femurX * sin(bodyLegAngles[legNum] + angs[legNum * 3 + 0]) * cos(angs[legNum * 3 + 1]);
        position(legNum * 3 + 2) = coxaZ + tibiaX * sin(angs[legNum * 3 + 1] + angs[legNum * 3 + 2]) + femurX * sin(angs[legNum * 3 + 1]);
    }

    return position;
}

// Print End Affector Positions
void HexapodControl::printPos() const
{
    for (int legNum = 0; legNum < 6; legNum++)
    {
        cout << "Leg " << legNum + 1 << " Position:  (" << pos(legNum * 3) << ", " << pos(legNum * 3 + 1) << ", " << pos(legNum * 3 + 2) << ")" << endl;
    }
}

// Move to straight leg position
void HexapodControl::jumpToZero()
{
    // Blank vector for desired angles
    Eigen::VectorXd zeroAnglesVector(18);

    // Desired angles array for each leg
    float zeroAngles[3] = {0, 0 * M_PI / 180, 360 * M_PI / 180};

    // Move angle arrays to eigen vector
    for (int i = 0; i < 18; ++i)
    {
        zeroAnglesVector[i] = zeroAngles[i % 3]; // Repeat the set of 3 angles
    }

    // Set Angles
    setAngs(zeroAnglesVector);
}
// Move to basic standing position
void HexapodControl::jumpToBasic()
{
    // Blank vector for desired angles
    Eigen::VectorXd basicAnglesVector(18);

    // Desired angles array for each leg
    float basicAngles[3] = {0 * M_PI / 180, 40 * M_PI / 180, (360 - 102) * M_PI / 180};

    // Move angle arrays to eigen vector
    for (int i = 0; i < 18; ++i)
    {
        basicAnglesVector[i] = basicAngles[i % 3]; // Repeat the set of 3 angles
    }

    // Set Angles
    setAngs(basicAnglesVector);
}
// Move to position that should fold back past limit when power disabled
void HexapodControl::jumpToOff()
{
    // Blank vector for desired angles
    Eigen::VectorXd offAnglesVector(18);

    // Desired angles array for each leg
    float offAngles[3] = {0 * M_PI / 180, 135 * M_PI / 180, (360 - 163) * M_PI / 180};

    // Move angle arrays to eigen vector
    for (int i = 0; i < 18; ++i)
    {
        offAnglesVector[i] = offAngles[i % 3]; // Repeat the set of 3 angles
    }

    // Set Angles
    setAngs(offAnglesVector);
    // moveLegsToPos(offAnglesVector);
}
// Move to curled position
void HexapodControl::jumpToCurled()
{
    // Blank vector for desired angles
    Eigen::VectorXd offAnglesVector(18);
    // Desired angles array for each leg
    float offAngles[3] = {0 * M_PI / 180, 135 * M_PI / 180, (360 - 158) * M_PI / 180};

    // Move angle arrays to eigen vector
    for (int i = 0; i < 18; ++i)
    {
        offAnglesVector[i] = offAngles[i % 3]; // Repeat the set of 3 angles
    }

    // Set Angles
    setAngs(offAnglesVector);
}

// Move to initial position for walk cycle
void HexapodControl::moveToOff()
{
    // Blank VectorXd for desired positions
    Eigen::VectorXd legOffPos(18);

    // Desired position double for each leg endpoint in incremental position
    double tempPosA[18] = {
        0.235586, 0.362771, 0.0,
        0.382555, 0.0, 0.0,
        0.235586, -0.362771, 0.0,
        -0.235586, -0.362771, 0.0,
        -0.382555, 0.0, 0.0,
        -0.235586, 0.362771, 0.0};

    // Map desired pos double to desired pos Eigen VectorXd
    legOffPos = Eigen::Map<Eigen::VectorXd>(tempPosA, 18);

    // Move legs to the desired position
    moveLegsToPos(legOffPos, 1000);

    // Desired position double for each leg endpoint in off position
    double tempPosB[18] = {
        0.19, 0.29, 0.0,
        0.29, 0, 0.0,
        0.19, -0.29, 0.0,
        -0.19, -0.29, 0.0,
        -0.29, 0, 0.0,
        -0.19, 0.29, 0.0};

    // Map desired pos double to desired pos Eigen VectorXd
    legOffPos = Eigen::Map<Eigen::VectorXd>(tempPosB, 18);

    // Move legs to the desired position
    moveLegsToPos(legOffPos, 1000);
}

// Move to initial position for walk cycle
void HexapodControl::stand()
{
    // Blank VectorXd for desired positions
    Eigen::VectorXd legstandPos(18);

    // Desired position double for each leg endpoint in incremental position
    double tempPosA[18] = {
        0.235586, 0.362771, 0,
        0.382555, 0, 0,
        0.235586, -0.362771, 0,
        -0.235586, -0.362771, 0,
        -0.382555, 0, 0,
        -0.235586, 0.362771, 0};

    // Map desired pos double to desired pos Eigen VectorXd
    legstandPos = Eigen::Map<Eigen::VectorXd>(tempPosA, 18);

    // Move legs to the desired position
    moveLegsToPos(legstandPos, 1000);

    moveToStand(standDuration);
}

// Move to initial position for walk cycle
void HexapodControl::moveToStand(float dur)
{
    // Blank VectorXd for desired positions
    Eigen::VectorXd legstandPos(18);

    // Map stand position double to desired pos Eigen VectorXd
    legstandPos = Eigen::Map<Eigen::VectorXd>(standPos, 18);

    // Move legs to the desired position
    moveLegsToPos(legstandPos, dur);
}

void HexapodControl::moveLegsToPos(const Eigen::VectorXd &desiredPos, float dur)
{
    // Variables for the mathematical operations
    Eigen::Vector3d desiredSpatialVelocity;
    Eigen::Vector3d legJointVelocity;
    Eigen::VectorXd desiredAngularVelocities(18);
    Eigen::VectorXd nextAngles(18);
    Eigen::MatrixXd jacobian;
    Eigen::MatrixXd jacobianPseudoInverse;
    Eigen::VectorXd posOffset(18);

    // Calculate position offset between desired position and current pos
    posOffset = desiredPos - pos;

    // Simulation Cycle
    for (int i = 0; i < dur / rsStep; i++)
    {
        // Reset desired angular velocities to zero
        desiredAngularVelocities.setZero();

        // Calculate control angular velocities per leg
        for (int legNum = 0; legNum < 6; legNum++)
        {
            // Velocity Control Calculations
            desiredSpatialVelocity << posOffset(legNum * 3), posOffset(legNum * 3 + 1), posOffset(legNum * 3 + 2);
            jacobian = getJacobian(legNum);
            jacobianPseudoInverse = jacobian.completeOrthogonalDecomposition().pseudoInverse();
            legJointVelocity = jacobianPseudoInverse * (desiredSpatialVelocity / (dur / 1000));

            // Add each leg velocity to the full desired angular velocity array
            for (int joint = 0; joint < 3; joint++)
            {
                desiredAngularVelocities[legNum * 3 + joint] = legJointVelocity[joint]; // Repeat the set of 3 angles
            }
        }

        // Find next aqngles using discrete integration
        nextAngles = currentAngles + desiredAngularVelocities * (rsStep / 1000);

        #ifdef USE_SIMULATOR
            // Send Velocities to the simulator
            simulator->setSimVelocity(nextAngles, desiredAngularVelocities);
        #endif

        // Update class angles and position for current state
        currentAngles = nextAngles;
        updatePos();

        // Implement Real time delay
        rsLoop.realTimeDelay();

        sendAngs();

        #ifdef USE_SIMULATOR
            // Integrate the Simulator server
            simulator->server->integrateWorldThreadSafe();
        #endif
    }

    // Set current angles as the desired holding angles when not performing operation
    desiredAngles = currentAngles;
}

void HexapodControl::moveLegToPos(const Eigen::Vector3d &desiredPos, const int legNum)
{

    Eigen::Vector3d desiredSpatialVelocity;
    Eigen::Vector3d legJointVelocity;
    Eigen::VectorXd desiredAngularVelocities(18);
    Eigen::VectorXd nextAngles(18);
    Eigen::MatrixXd jacobian;
    Eigen::MatrixXd jacobianPseudoInverse;

    Eigen::Vector3d currentPos = {pos((legNum - 1) * 3), pos((legNum - 1) * 3 + 1), pos((legNum - 1) * 3 + 2)};
    Eigen::Vector3d posOffset = desiredPos - currentPos;

    int dis = posOffset.norm() * 2000;

    int dur = 1000;

    // desiredSpatialVelocity = posOffset/(dur);
    desiredSpatialVelocity = posOffset / (dur / 1000);

    // Test Control Variables
    // float radius = 0.07; // meters
    // double period = 2;   // HZ
    // double cycles = 1;

    // Find Duration
    // int dur = period * cycles * 1000; // ms

    // cout<<endl<< "Leg " << legNum <<endl;
    // cout<< "currentPos: " << currentPos.transpose() <<endl;
    // cout<< "desiredPos: " << desiredPos.transpose() <<endl;
    // cout<< "posOffset: " << posOffset.transpose() <<endl;
    // cout<< "distance: " << dis <<endl;
    // cout<< "desiredSpatialVelocity: " << desiredSpatialVelocity.transpose() <<endl;
    // cout<< "legJointVelocity: " << legJointVelocity.transpose() <<endl;

    // Update time for real time loop
    rsLoop.updateTimeDelay();

    for (int i = 0; i <= dur / rsStep; i++)
    {
        // Reset desired angular velocities to zero
        desiredAngularVelocities.setZero();

        // Velocity Control Calculations
        jacobian = getJacobian(legNum - 1);
        jacobianPseudoInverse = jacobian.completeOrthogonalDecomposition().pseudoInverse();
        legJointVelocity = jacobianPseudoInverse * desiredSpatialVelocity;

        // Add each leg velocity to the full desired angular velocity array
        for (int joint = 0; joint < 3; joint++)
        {
            desiredAngularVelocities[(legNum - 1) * 3 + joint] = legJointVelocity[joint]; // Repeat the set of 3 angles
        }

        // Find next aqngles using discrete integration
        nextAngles = currentAngles + desiredAngularVelocities * (rsStep / 1000);

        setAngs(nextAngles);

        // Implement Real time delay
        rsLoop.realTimeDelay();
    }
}

// Set Angles Overload for 1x3 eigen vector
void HexapodControl::setAngs(const Eigen::VectorXd &angs)
{
    // Update class angles and position for current state
    currentAngles = angs;
    updatePos();

    // Send Angles to simulator and arduino (needs update)
    sendAngs();
}
// Send the angles of the servo motors to the arduino (need update)
void HexapodControl::sendAngs()
{
    // Copy of current angles to adjust for formatting without changing class parameter
    Eigen::VectorXd modifiedAngs = currentAngles;

    // For Sending to Arduino
    if (arduinoConnected)
    {
        // cout <<endl;

        // std::vector<std::bitset<bitLength>> angleBinaryRepresentation(modifiedAngs.size());

        const double scale_factor = 100.0;

        // Create packet: start byte, data, and checksum
        std::vector<uint8_t> packet;
        
        // Start byte (e.g., 0xA5)
        packet.push_back(0xA5);

        for (int i = 0; i < modifiedAngs.size(); i++)
        {
            double baseValue = modifiedAngs[i] * 180 / M_PI;
            if ((i + 1) % 3 == 0)
            {
                baseValue = -baseValue + 360;
            }

            baseValue += angleInits[i];

            if (i == 10 || i == 11 || i == 13 || i == 14 || i == 16 || i == 17)
            {
                baseValue = 180 - baseValue;
            };


            // Calc Out of range
            if (!i % 3)
            {
                if (baseValue > angleInits[i] + 60)
                {
                    // cout << "Coxa out of range, " << baseValue << ", capping arduino at " << angleInits[i] + 60 << endl;
                    baseValue = angleInits[i] + 60;
                }
                else if (baseValue < angleInits[i] - 60)
                {
                    // cout << "Coxa out of range, " << baseValue << ", capping arduino at " << angleInits[i] - 60 << endl;
                    baseValue = angleInits[i] - 60;
                }
            }
            else if (i % 3)
            {
                if (baseValue > 180)
                {
                    // cout << "Femur out of range, " << baseValue << ", capping arduino at " << 180 << endl;
                    baseValue = 180;
                }
                else if (baseValue < 0)
                {
                    // cout << "Femur out of range, " << baseValue << ", capping arduino at " << 0 << endl;
                    baseValue = 0;
                }
            }
            else
            {
                if (baseValue > 180)
                {
                    // cout << "Tibia out of range, " << baseValue << ", capping arduino at " << 180 << endl;
                    baseValue = 180;
                }
                else if (baseValue < 0)
                {
                    // cout << "Tibia out of range, " << baseValue << ", capping arduino at " << 0 << endl;
                    baseValue = 0;
                }
            }


            uint16_t scaled_value = static_cast<uint16_t>(round(baseValue * scale_factor));

            // cout << "DOF " << i+1 << ": " << baseValue << ", " << scaled_value << endl;

            // Optional: Ensure that the scaled value is within the 16-bit range
            scaled_value = std::min(scaled_value, static_cast<uint16_t>(65535));

            // High byte (first 8 bits)
            packet.push_back((scaled_value >> 8) & 0xFF);

            // Low byte (last 8 bits)
            packet.push_back(scaled_value & 0xFF);
            
        }
        
        // Compute the checksum (sum of all data bytes, modulo 256)
        uint8_t checksum = 0;
        for (size_t i = 1; i < packet.size(); ++i) {  // Skip the start byte
            checksum += packet[i];
        }
        checksum &= 0xFF;

        // Append checksum to packet
        packet.push_back(checksum);

            // for (const auto& byte : result) {
            //     std::cout << std::bitset<8>(byte) << " ";
            // }
            // cout<<endl;

        // std::cout <<endl<< "Sending packet: ";
        // for (auto byte : packet) {
        //     std::cout << std::hex << static_cast<int>(byte) << " ";
        // }

        arduino->sendPacket(packet);
    }

    // For simulator (NOT IDEAL)
    #ifdef USE_SIMULATOR
        // Set Simulation angles (NOT IDEAL) 
        simulator->setSimAngle(modifiedAngs);

        // Set current angles as the desired holding angles when not performing operation
        desiredAngles = modifiedAngs;
    #endif
}

void HexapodControl::jacobianTest(const int &style)
{
    // Test Control Variables
    float radius = 0.04; // meters
    double period = jacDuration/1000;   // secs
    double cycles = 2;

    // Find Duration
    int dur = period * cycles * 1000; // ms

    // Create vector variables for control calculations
    Eigen::Vector3d desiredSpatialVelocity;
    Eigen::Vector3d legJointVelocity;
    Eigen::VectorXd desiredAngularVelocities(18);
    Eigen::VectorXd nextAngles(18);
    Eigen::MatrixXd jacobian;
    Eigen::MatrixXd jacobianPseudoInverse;

    // Calculate expected operation duration
    operationDuration = dur / rsStep + 0 / rsStep;

    // Simulation Cycle
    for (int i = 0; i <= dur / rsStep + 0 / rsStep; i++)
    {
        // Operation stop interrupt
        if (operationDuration < 0)
        {
            // cout << "exiting" <<endl;
            break;
        }

        // Hold position for 1 seconds
        // if (i < 1000 / rsStep)
        // {
        //     #ifdef USE_SIMULATOR
        //         // Set sim velocity to 0
        //         simulator->setSimVelocity(desiredAngles, Eigen::VectorXd::Zero(18));
        //     }
        //     // Implement Real time delay
        //     rsLoop.realTimeDelay();

        //     #ifdef USE_SIMULATOR
        //         // Integrate the Simulator server
        //         simulator->server->integrateWorldThreadSafe();
        //     }
        //     // Decrement operation duration for limit
        //     operationDuration--;
        //     // Skip remaining loop
        //     continue;
        // }

        // Calculate control angular velocities per leg
        for (int legNum = 0; legNum < 6; legNum++)
        {
            // Determine jacobian test style
            if (style == 0)
            {
                // Circle Jacobian test in Y-Z plane
                desiredSpatialVelocity << 0,
                    radius * 2 / period * M_PI * cos(2 / period * M_PI * (i) * (rsStep / 1000)),
                    -radius * 2 / period * M_PI * sin(2 / period * M_PI * (i) * (rsStep / 1000))  * ((i < (0.5 * dur / rsStep)) * 2 -1);

                // Inf Jacobian test in Y-Z plane
                // desiredSpatialVelocity << 0,
                //     radius * 2 / period * M_PI * cos(2 / period * M_PI * (i) * (rsStep / 1000)),
                //     -radius / period * M_PI * sin(4 / period * M_PI * (i) * (rsStep / 1000));
            }
            else if (style == 1)
            {
                // Circle Jacobian test in X-Z plane
                desiredSpatialVelocity << radius * 2 / period * M_PI * sin(2 / period * M_PI * (i) * (rsStep / 1000)) * ((i < (0.5 * dur / rsStep)) * 2 -1),
                    0,
                    -radius * 2 / period * M_PI * cos(2 / period * M_PI * (i) * (rsStep / 1000));

                //  Inf Jacobian test in X-Z plane
                // desiredSpatialVelocity << radius / period * M_PI * sin(4 / period * M_PI * (i) * (rsStep / 1000)),
                //     0,
                //     -radius * 2 / period * M_PI * cos(2 / period * M_PI * (i) * (rsStep / 1000));
            }
            else if (style == 2)
            {
                // Circle Jacobian test in X-Y plane
                desiredSpatialVelocity << radius * 2 / period * M_PI * cos(2 / period * M_PI * (i) * (rsStep / 1000)),
                    radius * 2 / period * M_PI * sin(2 / period * M_PI * (i) * (rsStep / 1000))  * ((i < (0.5 * dur / rsStep)) * 2 -1),
                    0;

                // Inf Jacobian test in X-Y plane
                // desiredSpatialVelocity << radius * 2 / period * M_PI * cos(2 / period * M_PI * (i) * (rsStep / 1000)),
                //     radius / period * M_PI * sin(4 / period * M_PI * (i) * (rsStep / 1000)),
                //     0;
            }

            // Velocity Control Calculations
            jacobian = getJacobian(legNum);
            jacobianPseudoInverse = jacobian.completeOrthogonalDecomposition().pseudoInverse();
            legJointVelocity = jacobianPseudoInverse * desiredSpatialVelocity;

            // Add each leg velocity to the full desired angular velocity array
            for (int joint = 0; joint < 3; joint++)
            {
                desiredAngularVelocities[legNum * 3 + joint] = legJointVelocity[joint]; // Repeat the set of 3 angles
            }
        }

        // Find next aqngles using discrete integration
        nextAngles = currentAngles + desiredAngularVelocities * (rsStep / 1000);

        // Send the angles to the arduino and simulation as desired
        setAngs(nextAngles);

        #ifdef USE_SIMULATOR
            // Send Velocities to the simulator
            simulator->setSimVelocity(nextAngles, desiredAngularVelocities);
        #endif

        // Update class angles and position for current state
        currentAngles = nextAngles;
        updatePos();

        // Implement Real time delay
        rsLoop.realTimeDelay();

        #ifdef USE_SIMULATOR
            // Integrate the Simulator server
            simulator->server->integrateWorldThreadSafe();
        #endif

        // Decrement operation duration for limit
        operationDuration--;
    }

    // Set current angles as the desired holding angles when not performing operation
    desiredAngles = currentAngles;
}

void HexapodControl::walk(double vel, double ang)
{    
    // Reverse step stance if direction change from last step is greater than +- pi/2
    // Normalize the angles to the range -pi to pi
    auto normalizeAngle = [](double angle) -> double {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    };

    vel = moveVectorMag;

    if (vel <= 0.05) {
        ang = -lastAngle;
    }

    // Normalize both angles to the -pi to pi range
    ang = normalizeAngle(ang);
    lastAngle = normalizeAngle(lastAngle);

    // Calculate the difference and normalize to [-pi, pi] range
    double angleDifference = normalizeAngle(ang - lastAngle);

    // If angle difference is greater than +-pi/2 then switch leg stances
    if (abs(angleDifference) > M_PI / 2) {
        for (size_t i = 0; i < standing.size(); ++i) {
            standing[i] = !standing[i];
        }
    }

    // Update lastAngle to the current angle for future comparisons
    lastAngle = ang;

    double desiredPos[18];         // Step Endpoint
    double horizontalDistance[6];  // Array to store horizontal distances
    double angle[6];               // Array to store angles

    // For each leg (Could be changed to per stance if not single leg with angle normalisation to reduce operation time)
    for (size_t j = 0; j<6; j++) {

        if (vel <= 0.05) {
            desiredPos[j*3] = standPos[j*3];
            desiredPos[j*3+1] = standPos[j*3+1];
        } else {
            // Calculate desired position for next step disregarding current position
            desiredPos[j*3] = standPos[j*3] - stepRadius * cos(ang) * (standing[j] * 2 - 1);
            desiredPos[j*3+1] = standPos[j*3+1] - stepRadius * sin(ang) * (standing[j] * 2 - 1);
        }
        
    
        // Calculate step offset from current position
        double x = desiredPos[j*3] - pos[j*3];
        double y = desiredPos[j*3+1] - pos[j*3+1];

        // Calculate djusted horizontal distance per leg (Should be equal)
        horizontalDistance[j] = std::sqrt(x * x + y * y);

        // Calculate adjusted step angle (All legs in same stance should be equal with other stance being a pi offset)
        angle[j] = std::atan2(y, x);
    }

    // Full step duration (support and sweep)
    double T = minStepDuration/1000; // S

    // Time counter
    double t = 0.0;   

    // Create vector variables for control calculations
    Eigen::Vector3d desiredSpatialVelocity;
    Eigen::Vector3d legJointVelocity;
    Eigen::VectorXd desiredAngularVelocities(18);
    Eigen::VectorXd nextAngles(18);
    Eigen::MatrixXd jacobian;
    Eigen::MatrixXd jacobianPseudoInverse;
    double horizontalMovement;
    double verticalMovement;

    // Calculate expected duration for operation
    operationDuration = T/2 * 1000 / rsStep;

    // Init proportional speed control variables
    double rat1;
    double rat2;

    // Simulation Cycle
    // for (int i = 0; i < T/2 * 1000 / rsStep; i++)
    while (true)
    {

        // Calculate proportional speed control
        if (moveVectorMag > 0.05) {
            // Operation cownt down ratio
            rat1 = operationDuration / (T/2 * 1000 / rsStep);   
            // Completed Gait Cycle ratio
            rat2 = t/T; 
            // Recalculate full time based on new velocity
            T = (minStepDuration/1000)/moveVectorMag;  
            // Adjust real time counter value to ensure trajectory equations remain at correct stage
            t = rat2*T; 
            // adjust operation duration to for new velocity
            operationDuration =  (T/2 * 1000 / rsStep) * rat1; 

            // cout <<operationDuration << endl;
        }
        
        // Trajectory parameters
        vector<double> a = {-1.0 / 2.0, -2.0 / T, 0, 160.0 / pow(T, 3), -480.0 / pow(T, 4), 384.0 / pow(T, 5), 0};
        vector<double> b = {0, 0, 0, 512.0 / pow(T, 3), -3072.0 / pow(T, 4), 6144.0 / pow(T, 5), -4096.0 / pow(T, 6)};

        // Incremement time counter
        t+= 0.001 * rsStep;

        // Operation stop interrupt
        if (operationDuration <= 0)
        {
            break;
        }

        // Calculate control angular velocities per leg
        for (int legNum = 0; legNum < 6; legNum++)
        {
            
            if (standing[legNum])
            {
                // For Leg in support phase
                horizontalMovement = (2.0 * horizontalDistance[legNum]) / (T);
                verticalMovement = 0;

            } else {
                // For Leg in swing phase
                horizontalMovement = horizontalDistance[legNum] * (6 * a[6] * pow(t, 5) + 5 * a[5] * pow(t, 4) + 4 * a[4] * pow(t, 3) + 3 * a[3] * pow(t, 2) + 2 * a[2] * t + a[1]);
                verticalMovement =   stepHeight * (6 * b[6] * pow(t, 5) + 5 * b[5] * pow(t, 4) + 4 * b[4] * pow(t, 3) + 3 * b[3] * pow(t, 2) + 2 * b[2] * t + b[1]);
            }

            // Distribute horizontal velocity between X and Y vectors
            desiredSpatialVelocity << 
                    horizontalMovement * cos(angle[legNum]),
                    horizontalMovement * sin(angle[legNum]),
                    verticalMovement;

            // Velocity Control Calculations
            jacobian = getJacobian(legNum);
            jacobianPseudoInverse = jacobian.completeOrthogonalDecomposition().pseudoInverse();
            legJointVelocity = jacobianPseudoInverse * desiredSpatialVelocity;

            // Add each leg velocity to the full desired angular velocity array
            for (int joint = 0; joint < 3; joint++)
            {
                desiredAngularVelocities[legNum * 3 + joint] = legJointVelocity[joint]; // Repeat the set of 3 angles
            }
        }

        // Find next aqngles using discrete integration
        nextAngles = currentAngles + desiredAngularVelocities * (rsStep / 1000);

        // Send the angles to the arduino and simulation as desired
        setAngs(nextAngles);

        #ifdef USE_SIMULATOR
            // Send Velocities to the simulator
            simulator->setSimVelocity(nextAngles, desiredAngularVelocities);
        #endif

        // Update class angles and position for current state
        currentAngles = nextAngles;
        updatePos();

        // Implement Real time delay
        rsLoop.realTimeDelay();

        #ifdef USE_SIMULATOR
            // Integrate the Simulator server
            simulator->server->integrateWorldThreadSafe();
        #endif

        // Decrement operation duration for limit
        operationDuration--;
    }

    operationDuration = 0;

    // Set current angles as the desired holding angles when not performing operation
    desiredAngles = currentAngles;

    // Switch leg stances around for next leg stance
    for (size_t i = 0; i < standing.size(); ++i) {
        standing[i] = !standing[i];
    }
}