// sudo apt install chrony -y

#include "ppm_manager.hpp"
#include <iostream>
#include <thread>
#include <chrono>

int main()
{
    PPMManager ppmManager;
    ppmManager.startPPMUpdateLoop(600); // Update every 10 minutes

    while (true)
    {
        std::cout << "Current PPM: " << ppmManager.getCurrentPPM() << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(30));
    }

    return 0;
}
