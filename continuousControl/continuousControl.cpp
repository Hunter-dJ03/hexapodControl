#ifdef USE_SIMULATOR
    const bool raisimSimulator = true;
    #include "raisim/RaisimServer.hpp"
    #include <raisim/Path.hpp>
#else
    const bool raisimSimulator = false;
#endif

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <linux/input.h>
#include <cmath>
#include <thread>
#include <atomic>
#include <chrono>
#include <csignal>
#include "../modules/custom/utilities/utils.h"
#include <iomanip>
#include "../modules/custom/arduinoConnection/arduinoController.h"
#include "../modules/custom/rsTimedLoop/rsTimedLoop.h"
#include "hexapod.h"
#include <fstream>
#include <memory>
#include "../modules/custom/getController/getController.h"



using namespace std;

// Operation Parameters
const float rsStep = 5; // Real Time Step (ms)

// Real Time Loop initialisation
RSTimedLoop rsLoop(rsStep);

// Variables to share between threads
atomic<bool> running(true);
atomic<float> inputAxisX(0.0f);
atomic<float> inputAxisY(0.0f);
atomic<bool> leftBumper(false);
atomic<bool> rightBumper(false);
atomic<bool> buttonA(false);
atomic<bool> buttonB(false);
atomic<bool> buttonY(false);
atomic<bool> buttonX(false);
atomic<int> dPadX(0);
atomic<int> dPadY(0);
atomic<float> moveVectorMag;
atomic<float> moveVectorAng;

// Interrupt handler
void signalHandler(int signum) {
    running = false; // Stop the loops
}

// Function for reading from controller in new thread
void readController(HexapodControl& hexapod) {
    // Controller connection status
    int fd;

    // Loop until the Controller is successfully opened
    while (true) {

        // Input event stream for contorller (might need to adjust for when event changes )
        const string device = SteelSeriesDeviceFinder::get_device_path("SteelSeries Stratus XL");

        // Access controller
        fd = open(device.c_str(), O_RDONLY);

        if (fd == -1) {
            cerr << "Failed to open input device "<<device<<". Retrying..." << endl;
            sleep(1);  // Wait for 1 second before trying again
        } else {
            cout << "Connected to device: " << device << endl;
            break;  // Exit loop when successfully opened
        }
    }

    struct input_event ev;

    // Counter for low velocity to allow joystick parsing quickly through zero zone
    static int lowVelCounter = 0;

    // Controlle reading cycle
    while (running) {
        // Read event
        ssize_t n = read(fd, &ev, sizeof(ev));

        // Check if event read failes
        if (n == (ssize_t)-1) {
            cerr << "Failed to read input event." << endl;
            break;
        }

        // Only read button press (key) and joystick (abs) readings
        if (ev.type == EV_ABS || ev.type == EV_KEY) {
            // Handle deadzone for Right Trigger (code 9) and Left Trigger (code 10)
            if ((ev.code == 9 || ev.code == 10) && abs(ev.value) < 2048) {
                continue; // Skip further processing if within the deadzone
            }

            // Switch case for each event handling
            switch (ev.code) {
                case 0: // Left Joystick X
                    // cout << "Left Joystick X: " << ev.value << " (" 
                    //           << (ev.value < 0 ? "Left" : "Right") << ")" << endl;

                    inputAxisX = ev.value;
                    break;
                case 1: // Left Joystick Y
                    // cout << "Left Joystick Y: " << ev.value << " (" 
                    //           << (ev.value < 0 ? "Up" : "Down") << ")" << endl;

                    inputAxisY = ev.value;
                    break;
                // case 2: // Right Joystick X
                //     // cout << "Right Joystick X: " << ev.value << " (" 
                //     //           << (ev.value < 0 ? "Left" : "Right") << ")" << endl;
                //     break;
                // case 5: // Right Joystick Y
                //     // cout << "Right Joystick Y: " << ev.value << " (" 
                //     //           << (ev.value < 0 ? "Up" : "Down") << ")" << endl;
                //     break;
                case 16: // Dpad X
                    // cout << "Dpad X: " << ev.value << " (" 
                    //           << (ev.value < 0 ? "Left" : "Right") << ")" << endl;
                    dPadX = ev.value;
                    break;
                case 17: // DPad Y
                    // cout << "DPad Y: " << ev.value << " (" 
                    //           << (ev.value > 0 ? "Down" : "Up") << ")" << endl;
                    dPadY = ev.value;
                    break;
                // case 9: // Right Trigger
                //     // cout << "Right Trigger: " << ev.value << endl;
                //     break;
                // case 10: // Left Trigger
                //     // cout << "Left Trigger: " << ev.value << endl;
                //     break;
                // case 317: // Left Joystick Click
                //     // cout << "Left Joystick Click: " << ev.value << " (" 
                //     //           << (ev.value == 1 ? "Pressed" : "Released") << ")" << endl;
                //     break;
                case 318: // Right Joystick Click
                    // cout << "Right Joystick Click: " << ev.value << " (" 
                    //           << (ev.value == 1 ? "Pressed" : "Released") << ")" << endl;
                    running = false;
                    break;
                case 304: // Button A
                    // cout << "Button A: " << ev.value << " (" 
                    //           << (ev.value == 1 ? "Pressed" : "Released") << ")" << endl;
                    buttonA = ev.value;
                    break;
                case 307: // Button X
                    // cout << "Button X: " << ev.value << " (" 
                    //           << (ev.value == 1 ? "Pressed" : "Released") << ")" << endl;
                    buttonX = ev.value;
                    break;
                case 305: // Button B
                    static bool wasButtonBPressed = false; // Track the previous state of Button Y
                    // cout<<"ButtonB Pressed: " <<endl;
                    if (ev.value == 1 && !wasButtonBPressed) {
                        // Button B is pressed and was not pressed before (i.e., transition from unpressed to pressed)
                        cout<<"ButtonB Pressed: " <<endl;
                        
                        buttonB = true;
                        
                        hexapod.operationDuration = 0;
                        
                        wasButtonBPressed = true; // Update the state to pressed
                    } else {
                        // Button B is released, reset the state
                        buttonB = false;
                        wasButtonBPressed = false;
                    }
                    break;
                case 308: // Button Y
                    static bool wasButtonYPressed = false; // Track the previous state of Button Y
                    
                    if (ev.value == 1 && !wasButtonYPressed) {
                        // Button Y is pressed and was not pressed before (i.e., transition from unpressed to pressed)
                        buttonY = true;
                        
                        // Output the desired information
                        cout << "Current Angles: " << hexapod.currentAngles << endl;
                        cout << "Desired Angles: " << hexapod.desiredAngles << endl;
                        hexapod.printPos();
                        cout <<endl;
                        
                        wasButtonYPressed = true; // Update the state to pressed
                    } else {
                        // Button Y is released, reset the state
                        buttonY = false;
                        wasButtonYPressed = false;
                    }
                    break;
                case 311: // Right Bumper
                    // cout << "Right Bumper: " << ev.value << " (" 
                    //           << (ev.value == 1 ? "Pressed" : "Released") << ")" << endl;
                    rightBumper = ev.value;
                    break;
                case 310: // Left Bumper
                    // cout << "Left Bumper: " << ev.value << " (" 
                    //           << (ev.value == 1 ? "Pressed" : "Released") << ")" << endl;
                    leftBumper = ev.value;
                    break;
                default:
                    // cout << "Unknown event. Code: " << ev.code << " Value: " << ev.value << endl;
                    break;
            }
        }

        // Calculate the velocity magnitude and angle
        moveVectorMag = Utils::constrain(sqrt(pow(inputAxisX, 2) + pow(inputAxisY, 2)), 0, 2047)   / 2047 ;
        moveVectorAng = atan2(-inputAxisY, inputAxisX);

        // Store vector magnitude in hexapod class
        hexapod.moveVectorMag = moveVectorMag;

        // hexapod.di

        // Check if moveVectorMag is less than 0.05
        if (moveVectorMag < 0.05) {
            // Increment the counter if below 0.05
            lowVelCounter++;
        } else {
            // Reset the counter if it goes above 0.05
            lowVelCounter = 0;
        }

        // Handle low velocity magnitude to allow joystick parsing quickly through zero zone and distinguish from joystick release / stop
        // hexapod.operationDuration *= (lowVelCounter < 6);

        hexapod.directed = (lowVelCounter < 6);

        if (moveVectorMag < 0.05 && !hexapod.directed && lowVelCounter < 20) {
            hexapod.operationDuration = 0;
        } 
    }

    // close the device connection
    close(fd);
}

// Run the hexapod
void runHexapod(HexapodControl& hexapod) {
    
    // Record start time
    auto startTime = chrono::high_resolution_clock::now();

    // Set real time loop next time
    hexapod.rsLoop.updateTimeDelay();

    // Simulation Loop
    while (running) {
        
        // Preset holding position
        #ifdef USE_SIMULATOR
            hexapod.simulator->setSimVelocity(hexapod.desiredAngles, Eigen::VectorXd::Zero(18));
        #endif

        // Update the current position
        hexapod.updatePos();
        
        // If the hexapod has been deployed
        if (hexapod.active) {   

            // Handle walking
            if (moveVectorMag >= 0.05) {
                
                // cout << "\rMove Vector: " << moveVectorMag *100 << "\% at "<< moveVectorAng << endl;

                // Walk
                hexapod.walk(moveVectorMag, moveVectorAng);

                // Move back to stand position
                if (moveVectorMag < 0.05) {
                    hexapod.moveToStand(hexapod.standDuration);
                }  
            } 

            // Jacobian Tests
            if (buttonX) {
                if (dPadX == -1) {
                    hexapod.jacobianTest(0);
                    hexapod.moveToStand(hexapod.standDuration);
                } else if (dPadX == 1) {
                    hexapod.jacobianTest(1);
                    hexapod.moveToStand(hexapod.standDuration);
                } else if (dPadY == 1) {
                    hexapod.jacobianTest(2);
                    hexapod.moveToStand(hexapod.standDuration);
                }
            }

        }
        
        // Activate or Deactivate the rover;
        if (leftBumper && rightBumper) {

            if (!hexapod.active) {
                cout<<endl<<"standing" <<endl;
                hexapod.active = true;
                hexapod.stand();
                
            } else {
                cout<<endl<<"turning off" <<endl;
                hexapod.active = false;
                hexapod.moveToOff();
                
            }
           
        } 
        
        // Display current leg information
        if (buttonY) {
            cout<<"Current Angles: "<< hexapod.currentAngles<<endl;
            cout<<"Desired Angles: "<< hexapod.desiredAngles<<endl;
            hexapod.printPos();
        }

        // Display current leg information
        if (buttonB) {
            hexapod.jumpToZero();
        }
        // Display current leg information
        if (buttonA) {
            hexapod.jumpToOff();
        }

        // Real time delay and server integration;
        hexapod.rsLoop.realTimeDelay();

        #ifdef USE_SIMULATOR
            hexapod.simulator->server->integrateWorldThreadSafe();
        #endif
    }

    // Calculate and display the duration
    auto endTime = chrono::high_resolution_clock::now();
    chrono::duration<double> elapsed = endTime - startTime;
    cout << "\nrunHexapod loop duration: " << elapsed.count() << " seconds." << endl;

    #ifdef USE_SIMULATOR
        // Close the raisim simulator
        hexapod.simulator->server->killServer();
    #endif
}

// initial function
int main(int argc, char* argv[]) {

    #ifdef USE_SIMULATOR
        Path binaryPath = raisim::Path::setFromArgv(argv[0]);
    #endif

    // Setup communication file stream for arduino connection
    fstream arduinoPort("/dev/ttyACM0");

    // Create arduino controller as blank unique pointer
    unique_ptr<ArduinoController> arduino;

    // Assume arduino Connected
    bool arduinoConnected = true;

    // Handle arduino connection
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
    HexapodControl hexapod(1, move(arduino), rsLoop, arduinoConnected, raisimSimulator, rsStep
    #ifdef USE_SIMULATOR
        , binaryPath
    #endif
    );

    #ifdef USE_SIMULATOR
        hexapod.simulator->server->focusOn(hexapod.simulator->hexapodLegModel.get());
    #endif

    // Start the controller input reading in a separate thread
    thread input_thread(readController, ref(hexapod));
    // Register signal handler
    signal(SIGINT, signalHandler);

    // Run the simulation in the main thread
    runHexapod(hexapod);

    return 0;
}
