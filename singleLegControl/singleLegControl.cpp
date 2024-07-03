#include "../modules/custom/arduinoConnection/arduinoController.h"
#include "../modules/custom/rsTimedLoop/rsTimedLoop.h"
#include "hexapodLeg.h"
#include <iostream>
#include <fstream>
#include <memory>

using namespace std;

const float rsStep = 20; // Real Time Step (ms)

RSTimedLoop rsLoop(rsStep);
// ArduinoController arduino("/dev/ttyACM0", 115200);

void parseCommand(const string& command, HexapodLeg &leg);

int main() {
    fstream arduinoPort("/dev/ttyACM0");

    unique_ptr<ArduinoController> arduino;
    bool simulationMode;

    if (arduinoPort) {
        arduino = make_unique<ArduinoController>("/dev/ttyACM0", 115200);
        simulationMode = false;
        cout << "Arduino connected." << endl;
        
    } else {
        arduino = make_unique<ArduinoController>();
        simulationMode = true;
        cout << "Arduino not connected. Running in simulation mode." << endl;
    }
    
    HexapodLeg leg(1, move(arduino), rsLoop, simulationMode, rsStep);

    string command;
    while (true) {
        cout << "Enter command: ";
        cin >> command;

        if (command == "exit") {
            break;
        };

        parseCommand(command, leg);
    }
    
    return 0;
}

void parseCommand(const string& command, HexapodLeg &leg) {
    if (command == "zero") {
        leg.moveToZero();
    } else if (command == "basic") {
        leg.moveToBasic();
    } else if (command == "off") {
        leg.moveToOff();
    } else if (command == "ikTest") {
        leg.doIKTest();
    } else if (command == "jacTest") {
        leg.doJacobianTest(1);
    } else {
        cout << "Unknown command: " << command << endl;
    }
}