#ifndef ARDUINOCONTROLLER_H
#define ARDUINOCONTROLLER_H

#include <string>
#include <boost/asio.hpp>
#include <thread>
#include <atomic>
#include <bitset>

using namespace std;

class ArduinoController {
public:
    ArduinoController(const string& port, unsigned int baud_rate);
    ArduinoController();
    ~ArduinoController();

    void sendStringCommand(const string& command);
    void sendBitSetCommand(std::vector<uint8_t>& command);
    void sendPacket(const vector<uint8_t> &packet);
    void closeConnection();
    string readData();

private:
    boost::asio::io_service io;
    boost::asio::serial_port serial;
    thread read_thread;
    atomic<bool> running;

    // int maxBitSent = 0;
};

#endif // ARDUINOCONTROLLER_H
