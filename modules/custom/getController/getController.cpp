// getController.cpp

#include "getController.h"
#include <iostream>
#include <fstream>
#include <regex>
#include <fcntl.h>
#include <unistd.h>
#include <linux/input.h>
#include <sys/ioctl.h>
#include <bitset> // For binary representation

// Implementation of static function to find the device path
std::string SteelSeriesDeviceFinder::get_device_path(const std::string &device_name)
{
    std::vector<std::string> event_paths = find_devices_by_name(device_name);

    // Iterate through all found devices and check their capabilities
    for (const auto &event_path : event_paths)
    {
        if (check_event_capabilities(event_path))
        {
            return event_path; // Return the first matching device
        }
    }
    return ""; // Return empty string if no suitable device is found
}

std::vector<std::string> SteelSeriesDeviceFinder::find_devices_by_name(const std::string &device_name)
{
    std::ifstream infile("/proc/bus/input/devices");
    std::string line;
    std::vector<std::string> event_paths;
    bool is_target_device = false;
    std::regex event_regex("event[0-9]+");

    // Adjusted to find the exact match for "SteelSeries Stratus XL" (not "Keyboard" or "Mouse")
    std::string full_device_name = "SteelSeries Stratus XL";

    while (std::getline(infile, line))
    {
        // Check if the line contains the device name but not "Keyboard" or "Mouse"
        if (line.find(full_device_name) != std::string::npos &&
            line.find("Keyboard") == std::string::npos &&
            line.find("Mouse") == std::string::npos)
        {
            is_target_device = true;
        }

        // If we're inside the correct device block, look for event ID
        if (is_target_device && std::regex_search(line, event_regex))
        {
            // Extract the event number
            std::smatch match;
            if (std::regex_search(line, match, event_regex))
            {
                std::string event_path = "/dev/input/" + match.str();
                event_paths.push_back(event_path); // Store the event path
                is_target_device = false;          // Reset flag, we've moved to the next block
            }
        }
    }

    infile.close();
    return event_paths;
}
// Check if the device has the required event codes
// Check if the device has the correct set of event types
// Check if the device has the correct set of event types and specific event codes
// Check if the device has the correct set of event types and specific event codes
bool SteelSeriesDeviceFinder::check_event_capabilities(const std::string &event_path)
{
    int fd = open(event_path.c_str(), O_RDONLY);
    if (fd == -1)
    {
        std::cerr << "Failed to open device: " << event_path << std::endl;
        return false;
    }

    // Define the event codes you're interested in (e.g., EV_ABS, EV_KEY, etc.)
    unsigned long bitmask[EV_MAX / (8 * sizeof(unsigned long))]; // Declare the bitmask
    // memset(bitmask, 0, sizeof(bitmask));  // Zero-initialize the bitmask

    ioctl(fd, EVIOCGBIT(0, EV_MAX), bitmask); // Get the event bitmask

    // std::cout << "Supported Event Codes for device " << event_path << ":\n";

    bool has_ev_key = false;
    bool has_ev_abs = false;
    bool supports_abs_x = false;
    bool supports_abs_y = false;

    // Check for EV_KEY and EV_ABS (general event types)
    if (bitmask[EV_KEY / (8 * sizeof(unsigned long))])
    {
        has_ev_key = true;
        // std::cout << "EV_KEY is supported.\n";
    }
    if (bitmask[EV_ABS / (8 * sizeof(unsigned long))])
    {
        has_ev_abs = true;
        // std::cout << "EV_ABS is supported.\n";
    }

    // Only check specific codes like ABS_X, ABS_Y if the device is a controller (not a keyboard)
    if (has_ev_abs)
    {
        if (bitmask[EV_ABS / (8 * sizeof(unsigned long))] & (1UL << ABS_X))
        {
            supports_abs_x = true;
            // std::cout << "ABS_X (X Axis) is supported.\n";
        }
        if (bitmask[EV_ABS / (8 * sizeof(unsigned long))] & (1UL << ABS_Y))
        {
            supports_abs_y = true;
            // std::cout << "ABS_Y (Y Axis) is supported.\n";
        }
    }

    // We know the correct controller supports both EV_KEY and EV_ABS, as well as ABS_X, ABS_Y
    if (has_ev_key && has_ev_abs && supports_abs_x && supports_abs_y)
    {
        // std::cout << "This device matches the correct capabilities!" << std::endl;
        close(fd);
        return true;
    }

    close(fd);
    return false; // If it doesn't match, return false
}

// Helper function to convert event code to human-readable name (simplified for example)
std::string SteelSeriesDeviceFinder::event_code_to_name(int code)
{
    switch (code)
    {
    case EV_KEY:
        return "EV_KEY";
    case EV_ABS:
        return "EV_ABS";
    case EV_REL:
        return "EV_REL";
    case EV_SYN:
        return "EV_SYN";
    default:
        return "Unknown event code";
    }
}