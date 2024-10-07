#include "../modules/custom/arduinoConnection/arduinoController.h"
#include "../modules/custom/rsTimedLoop/rsTimedLoop.h"
#include "hexapod.h"
#include <iostream>
#include <fstream>
#include <memory>
#include "raisim/RaisimServer.hpp"

using namespace std;

const bool raisimSimulator = true;
const float rsStep = 10; // Real Time Step (ms)

RSTimedLoop rsLoop(rsStep);

void parseCommand(const string& command, Hexapod &leg);

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
    
    // Create Hexapod 
    Hexapod hexapod(1, move(arduino), rsLoop, arduinoConnected, raisimSimulator, rsStep, binaryPath);

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
        parseCommand(command, hexapod);
    }
    
    return 0;
}

// Takes command input and if valid, performs the desired operations
void parseCommand(const string& command, Hexapod &hexapod) {
    if (command == "zero") {
        hexapod.jumpToZero();
    } else if (command == "basic") {
        hexapod.jumpToBasic();
    } else if (command == "off") {
        hexapod.jumpToOff();
    // } else if (command == "ikTest") {
    //     hexapod.doBodyIKTest();
    } else if (command == "jacTest") {
        hexapod.doJacobianTest(1);
    } else if (command == "stand") {
        hexapod.stand();
    } else if (command == "printPos") {
        hexapod.printPos();
    } else if (command == "curl") {
        hexapod.jumpToCurled();
    }
    else {
        cout << "Unknown command: " << command << endl;
    }
}