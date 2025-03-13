/**
 * @file ppm_manager.cpp
 * @brief Implementation of the PPMManager class for managing periodic PPM calculations.
 *
 * This file defines the PPMManager class, which manages periodic PPM (Parts
 * Per Million) calculations to track clock drift. 1 PPM = 1 microsecond of
 * drift every second.
 *
 * It retrieves PPM values from Chrony (if available) and periodically updates
 * them using system timing functions.
 *
 * This software is distributed under the MIT License. See LICENSE.md for
 * details.
 *
 * Copyright (C) 2025 Lee C. Bussy (@LBussy). All rights reserved.
 */

#include "ppm_manager.hpp"

#include <chrono>
#include <cstdlib>
#include <cmath>
#include <cstdio>

/**
 * @brief Constructs a PPMManager instance.
 *
 * Initializes internal values but does not start the update loop.
 */
PPMManager::PPMManager() : ppm_value(0.0), running(false) {}

/**
 * @brief Initializes the PPMManager.
 *
 * Checks if the system time is synchronized, attempts to retrieve the initial PPM
 * from Chrony, and falls back to clock drift measurement if necessary.
 *
 * @return A PPMStatus indicating success or failure reason.
 */
PPMStatus PPMManager::initialize()
{
    if (!isTimeSynchronized())
    {
        return PPMStatus::ERROR_UNSYNCHRONIZED_TIME;
    }

    double initial_ppm = getChronyPPM();
    if (initial_ppm == -9999.0)
    {
        initial_ppm = measureClockDrift();
        return PPMStatus::ERROR_CHRONY_NOT_FOUND;
    }

    ppm_value = initial_ppm;
    return PPMStatus::SUCCESS;
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

    const int check_interval = 1;
    int elapsed_seconds = 0;

    while (running && elapsed_seconds < seconds)
    {
        std::this_thread::sleep_for(std::chrono::seconds(check_interval));
        if (!running)
            return 0.0;
        elapsed_seconds += check_interval;
    }

    clock_gettime(CLOCK_REALTIME, &end);

    double elapsed = (end.tv_sec - start.tv_sec) + (end.tv_nsec - start.tv_nsec) / 1e9;
    double expected = static_cast<double>(seconds);

    return ((elapsed - expected) / expected) * 1e6;
}

/**
 * @brief The internal update loop for recalculating PPM.
 *
 * Periodically updates the PPM value by averaging Chrony and measured drift values.
 * The loop continues running while `running` is true.
 *
 * @param interval_seconds The interval in seconds between PPM updates.
 */
PPMStatus PPMManager::ppmUpdateLoop(int interval_seconds)
{
    const int check_interval = 1;
    while (running)
    {
        double chrony_ppm = getChronyPPM();
        double measured_ppm = measureClockDrift();

        if (std::abs(measured_ppm) > 200)
        {
            return PPMStatus::WARNING_HIGH_PPM;
        }

        double avg_ppm = (chrony_ppm + measured_ppm) / 2.0;

        {
            std::lock_guard<std::mutex> lock(ppm_mutex);
            ppm_value = avg_ppm;
        }

        for (int i = 0; i < interval_seconds; i += check_interval)
        {
            if (!running)
                return PPMStatus::SUCCESS;
            std::this_thread::sleep_for(std::chrono::seconds(check_interval));
        }
    }
    return PPMStatus::SUCCESS;
}

/**
 * @brief Starts the PPM update loop in a background thread.
 *
 * Ensures that only one instance of the loop runs at a time.
 *
 * @param interval_seconds The interval in seconds between PPM updates.
 */
PPMStatus PPMManager::startPPMUpdateLoop(int interval_seconds)
{
    if (running)
    {
        return PPMStatus::SUCCESS;
    }
    running = true;
    ppm_thread = std::thread(&PPMManager::ppmUpdateLoop, this, interval_seconds);
    return PPMStatus::SUCCESS;
}

/**
 * @brief Stops the PPM update loop safely.
 *
 * Ensures the thread is properly terminated before stopping.
 */
PPMStatus PPMManager::stop()
{
    if (running)
    {
        running = false;
        if (ppm_thread.joinable())
        {
            ppm_thread.join();
        }
    }
    return PPMStatus::SUCCESS;
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
