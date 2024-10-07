#include "../modules/custom/arduinoConnection/arduinoController.h"
#include "../modules/custom/rsTimedLoop/rsTimedLoop.h"
#include "hexapodLeg.h"
#include <iostream>
#include <fstream>
#include <memory>
#include "raisim/RaisimServer.hpp"

using namespace std;

const bool raisimSimulator = true;
const float rsStep = 1; // Real Time Step (ms)

RSTimedLoop rsLoop(rsStep);

void parseCommand(const string& command, HexapodLeg &leg);

// Main control function
int main(int argc, char* argv[]) {

    // Set the path for raisim simulator
    Path binaryPath = raisim::Path::setFromArgv(argv[0]);

    // Setup communication file stream for arduino connection
    fstream arduinoPort("/dev/ttyACM0");

    // Create arduino controller as blank unique pointer
    unique_ptr<ArduinoController> arduino;

    // Assume arduino Connected
    bool arduinoConnected = true;

    if (arduinoPort) {
        // Set the Arduino Controller up with port and abud rate
        arduino = make_unique<ArduinoController>("/dev/ttyACM0", 921600);

        cout << "Arduino connected." << endl;
        
    } else {
        // Set arduino as a blank controller
        arduino = make_unique<ArduinoController>();

        // Change sim mode
        arduinoConnected = false;

        cout << "Arduino not connected. Running in simulation mode." << endl;
    }
    
    // Create HexapodLeg 
    HexapodLeg leg(1, move(arduino), rsLoop, arduinoConnected, raisimSimulator, rsStep, binaryPath);

    // Placeholder variables for command input
    string command;

    // Control loop
    while (true) {
        // Get Command
        cout << "Enter command: ";
        cin >> command;

        // Exit clause
        if (command == "exit") {
            break;
        };

        // Parse the command for desired control
        parseCommand(command, leg);
    }
    
    return 0;
}

// Takes command input and if valid, performs the desired operations
void parseCommand(const string& command, HexapodLeg &leg) {
    if (command == "zero") {
        leg.jumpToZero();
    } else if (command == "basic") {
        leg.jumpToBasic();
    } else if (command == "off") {
        leg.jumpToOff();
    } else if (command == "ikTest") {
        leg.doBodyIKTest();
    } else if (command == "jacTest") {
        leg.doJacobianTest(1);
    } else {
        cout << "Unknown command: " << command << endl;
    }
}