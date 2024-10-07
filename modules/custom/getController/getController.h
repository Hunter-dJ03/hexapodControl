// getController.h

#ifndef GETCONTROLLER_H
#define GETCONTROLLER_H

#include <string>
#include <vector>

class SteelSeriesDeviceFinder {
public:
    // Static function to find and return the correct event device path
    static std::string get_device_path(const std::string& device_name);

private:
    // Private helper methods
    static std::vector<std::string> find_devices_by_name(const std::string& device_name);
    static bool check_event_capabilities(const std::string &event_path);

    // Helper function to convert event codes to human-readable names
    static std::string event_code_to_name(int code);
};

#endif // GETCONTROLLER_H
