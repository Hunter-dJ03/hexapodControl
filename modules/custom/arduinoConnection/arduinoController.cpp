#include "arduinoController.h"
#include <iostream>
#include <boost/asio.hpp>
#include <thread>

// Default Constructor - Blank initilisation for when arduino not connected
ArduinoController::ArduinoController() : serial(io) {};

// Constructor - Initilises the serial connection on set port with set baud rate
ArduinoController::ArduinoController(const std::string &port, unsigned int baud_rate)
    : serial(io, port), running(true)
{
    serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
}

// Destructor - Clears the arduino connections
ArduinoController::~ArduinoController()
{
    closeConnection();
}

// Sends input command to arduino
void ArduinoController::sendStringCommand(const std::string &command)
{
    boost::asio::write(serial, boost::asio::buffer(command + "\n"));
}

// Sends input command to arduino
void ArduinoController::sendPacket(const vector<uint8_t> &packet)
{
    boost::asio::write(serial, boost::asio::buffer(packet));
}

// Sends input command to arduino
void ArduinoController::sendBitSetCommand(std::vector<uint8_t> &command)
{
    boost::asio::write(serial, boost::asio::buffer(command));

    // std::size_t bits_sent = command.size() * 8; // Each byte in the bitstream contains 8 bits
    // std::cout << "Sending " << bits_sent << " bits" << std::endl;

    // if (bits_sent > maxBitSent)
    // {
    //     maxBitSent = bits_sent;
    // };

    // std::cout << "Max Bits Sent " << maxBitSent << std::endl;
}

// Reads commands from the serial connection with arduino
std::string ArduinoController::readData()
{
    // Preset varaibles declarations
    char c;
    std::string result;
    boost::system::error_code ec;

    // Reading from the serial port one character at a time
    while (boost::asio::read(serial, boost::asio::buffer(&c, 1), ec))
    {
        // Break the loop if a newline character is encountered
        if (c == '\n')
            break;
        // Append the read character to the result string
        result += c;
    }

    // Check for errors and display error string 
    if (ec && ec != boost::asio::error::eof)
    {
        std::cerr << "Error reading from Arduino: " << ec.message() << std::endl;
    }

    // Return read result
    return result;
}

// Close the serial connection
void ArduinoController::closeConnection()
{
    running = false;
    if (serial.is_open())
    {
        serial.close();
    }
}
