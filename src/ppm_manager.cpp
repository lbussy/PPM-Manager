#include "ppm_manager.hpp"

#include <chrono>
#include <cstdlib>
#include <cmath>
#include <cstdio>
#include <iostream>

/**
 * @brief Constructs a PPMManager instance.
 *
 * Initializes the PPM tracking system. If the system time is synchronized,
 * attempts to retrieve the initial PPM value from Chrony. If Chrony is unavailable,
 * falls back to clock drift measurement.
 */
PPMManager::PPMManager() : ppm_value(0.0), running(false)
{
    if (isTimeSynchronized())
    {
        double initial_ppm = getChronyPPM();
        if (initial_ppm == -9999.0)
        {
            std::cerr << "Warning: Chrony not found. Falling back to clock drift measurement." << std::endl;
            initial_ppm = measureClockDrift();
        }
        std::cout << "Initial PPM: " << initial_ppm << std::endl;
        ppm_value = initial_ppm;
    }
    else
    {
        std::cerr << "Warning: System time is not synchronized." << std::endl;
    }
}

/**
 * @brief Destroys the PPMManager instance.
 *
 * Ensures the PPM update loop stops before the object is destroyed.
 */
PPMManager::~PPMManager()
{
    stop(); // Ensure the thread stops on destruction
}

/**
 * @brief Checks if the system time is synchronized.
 *
 * Uses `timedatectl` to determine whether NTP synchronization is active.
 *
 * @return True if the system time is synchronized, false otherwise.
 */
bool PPMManager::isTimeSynchronized()
{
    FILE *pipe = popen("timedatectl show --property=NTPSynchronized --value", "r");
    if (!pipe)
        return false;

    char buffer[8];
    if (fgets(buffer, sizeof(buffer), pipe) == nullptr)
    {
        pclose(pipe);
        return false;
    }
    pclose(pipe);
    return (buffer[0] == 'y'); // 'y' indicates synchronization
}

/**
 * @brief Retrieves the initial PPM value from Chrony.
 *
 * Uses `chronyc tracking` to get the frequency offset in PPM.
 * Returns `-9999.0` if Chrony is unavailable.
 *
 * @return The PPM value from Chrony, or `-9999.0` on failure.
 */
double PPMManager::getChronyPPM()
{
    FILE *pipe = popen("chronyc tracking | grep 'Frequency' | awk '{print $3}'", "r");
    if (!pipe)
        return -9999.0;

    char buffer[128];
    if (fgets(buffer, sizeof(buffer), pipe) == nullptr)
    {
        pclose(pipe);
        return -9999.0;
    }
    pclose(pipe);
    return std::stod(buffer);
}

/**
 * @brief Measures clock drift over a specified duration.
 *
 * Uses `clock_gettime()` to compare elapsed time with expected time,
 * then calculates PPM deviation.
 *
 * @param seconds The duration in seconds to measure drift (default: 300s).
 * @return The calculated PPM drift value.
 */
double PPMManager::measureClockDrift(int seconds)
{
    struct timespec start, end;
    clock_gettime(CLOCK_REALTIME, &start);
    std::this_thread::sleep_for(std::chrono::seconds(seconds));
    clock_gettime(CLOCK_REALTIME, &end);

    double elapsed = (end.tv_sec - start.tv_sec) + (end.tv_nsec - start.tv_nsec) / 1e9;
    double expected = static_cast<double>(seconds);

    return ((elapsed - expected) / expected) * 1e6; // Convert to PPM
}

/**
 * @brief The internal update loop for recalculating PPM.
 *
 * Periodically updates the PPM value by averaging Chrony and measured drift values.
 * The loop continues running while `running` is true.
 *
 * @param interval_seconds The interval in seconds between PPM updates.
 */
void PPMManager::ppmUpdateLoop(int interval_seconds)
{
    while (running)
    {
        double chrony_ppm = getChronyPPM();
        double measured_ppm = measureClockDrift();

        if (std::abs(measured_ppm) > 200)
        {
            std::cerr << "Warning: Measured PPM (" << measured_ppm << ") exceeds threshold. Ignoring." << std::endl;
            continue;
        }

        double avg_ppm = (chrony_ppm + measured_ppm) / 2.0;

        {
            std::lock_guard<std::mutex> lock(ppm_mutex);
            ppm_value = avg_ppm;
        }

        std::cout << "Updated PPM: " << avg_ppm
                  << " (Chrony: " << chrony_ppm
                  << ", Measured: " << measured_ppm << ")." << std::endl;

        std::this_thread::sleep_for(std::chrono::seconds(interval_seconds));
    }
}

/**
 * @brief Starts the PPM update loop in a background thread.
 *
 * Ensures that only one instance of the loop runs at a time.
 *
 * @param interval_seconds The interval in seconds between PPM updates.
 */
void PPMManager::startPPMUpdateLoop(int interval_seconds)
{
    if (running)
    {
        std::cerr << "Warning: PPM update loop is already running." << std::endl;
        return;
    }
    running = true;
    ppm_thread = std::thread(&PPMManager::ppmUpdateLoop, this, interval_seconds);
}

/**
 * @brief Stops the PPM update loop safely.
 *
 * Ensures the thread is properly terminated before stopping.
 */
void PPMManager::stop()
{
    if (running)
    {
        running = false;
        if (ppm_thread.joinable())
        {
            ppm_thread.join();
        }
    }
}

/**
 * @brief Retrieves the latest calculated PPM value.
 *
 * Ensures thread-safe access to the PPM value.
 *
 * @return The most recent PPM measurement.
 */
double PPMManager::getCurrentPPM()
{
    std::lock_guard<std::mutex> lock(ppm_mutex);
    return ppm_value;
}
