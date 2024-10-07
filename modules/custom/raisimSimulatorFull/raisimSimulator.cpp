#include "raisimSimulator.h"
#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <chrono>
#include <memory>

using namespace std;
using namespace raisim;

// Constructor - Sets key variables and starts initilisation
RaisimSimulator::RaisimSimulator(const float rsStep, Path binaryPath, const string URDFName) : rsStep(rsStep), binaryPath(binaryPath), URDFName(URDFName)
{
    initialize(rsStep);
}

// Destructor - Kill the Raisim Simulation
RaisimSimulator::~RaisimSimulator()
{
    server->killServer();
}

// Send joint angles to simulation model with Vector3d input
void RaisimSimulator::setSimAngle(Eigen::VectorXd angs)
{
    // Send joint angles to simulation model
    hexapodLegModel->setGeneralizedCoordinate(angs);
}


void RaisimSimulator::setSimVelocity(Eigen::VectorXd nextAngles, Eigen::VectorXd desiredAngularVelocities)
{
    
    // Get Current Angles
    raisim::VecDyn currentAnglesVecDyn = hexapodLegModel->getGeneralizedCoordinate();
    raisim::VecDyn currentAngularVelocitiesVecDyn = hexapodLegModel->getGeneralizedVelocity();
    // Convert raisim::VecDyn type to Eigen::Vector3d
    Eigen::VectorXd currentAngles = convertVecDynToEigen(currentAnglesVecDyn);
    Eigen::VectorXd currentAngularVelocities = convertVecDynToEigen(currentAngularVelocitiesVecDyn);
    // Find controlled angular velocities through PI controller
    Eigen::VectorXd controlledAngularVelocities = currentAngularVelocities + Kp.cwiseProduct(desiredAngularVelocities - currentAngularVelocities) + Ki.cwiseProduct(nextAngles - currentAngles);
    // Set the simulation model joint velocities

    // cout<<"Setting Velocity"<<endl;

    // hexapodLegModel->setGeneralizedVelocity(controlledAngularVelocities);
    hexapodLegModel->setGeneralizedVelocity(desiredAngularVelocities);
}

// Initilise the Raisim Simulation
void RaisimSimulator::initialize(const float rsStep)
{
    // Make World and Size
    world.setTimeStep(rsStep / 1000);
    auto ground = world.addGround(-2);
    
    world.setGravity(Eigen::Vector3d(0, 0, 0));
    // Add Hexapod Leg Model to world
    addModel();

    // Build and launch Server
    server = make_unique<RaisimServer>(&world);
    server->launchServer(8080);

    // Wait for server connection
    cout << "Awaiting Connection to raisim server" << endl;
    while (!server->isConnected())
        ;
    cout << "Server Connected" << endl;
}

// Add Hexapod Leg Model to the Simulation
void RaisimSimulator::addModel()
{
    // Add the model to the world
    hexapodLegModel = shared_ptr<ArticulatedSystem>(world.addArticulatedSystem(binaryPath.getDirectory() + "/models/hexapod/urdf/" + URDFName));
    hexapodLegModel->setName("HexapodModel");

    // Remove Collision Meshes
    for (int i = 0; i <= 3; ++i)
    {
        for (int j = i + 1; j <= 3; ++j)
        {
            hexapodLegModel->ignoreCollisionBetween(i, j);
        }
    }
}



Eigen::VectorXd RaisimSimulator::convertVecDynToEigen(const raisim::VecDyn& vecDyn) {
    Eigen::VectorXd eigenVec(vecDyn.size());
    for (int i = 0; i < vecDyn.size(); ++i) {
        eigenVec[i] = vecDyn[i];
    }
    return eigenVec;
}