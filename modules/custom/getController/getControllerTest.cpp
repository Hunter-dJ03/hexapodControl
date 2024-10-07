// main.cpp

#include <iostream>
#include "getController.h"

int main() {
    // Call the static function without creating an object
    std::string device_path = SteelSeriesDeviceFinder::get_device_path("SteelSeries Stratus XL");

    if (!device_path.empty()) {
        std::cout << "Correct event found: " << device_path << std::endl;
    } else {
        std::cerr << "Correct event device not found!" << std::endl;
    }

    return 0;
}
