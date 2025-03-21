/**
 * @file ppm_manager.cpp
 * @brief Implementation of the PPMManager class for managing periodic PPM
 * calculations.
 *
 * Copyright (C) 2025 Lee C. Bussy (@LBussy). All rights reserved.
 *
 * This file declares the PPMManager class, which manages periodic PPM (Parts
 * Per Million) calculations to track clock drift. 1 PPM = 1 microsecond of
 * drift every second.
 *
 * It retrieves PPM values from Chrony (if available) and periodically updates
 * them using system timing functions.
 *
 * This software is distributed under the MIT License. See LICENSE.md for
 * details.
 */

#include "ppm_manager.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <numeric>
#include <optional>
#include <thread>

#ifdef DEBUG_PPMMANAGER
#include <iomanip>
#include <ctime>
#include <iostream>
#include <sstream>
#include <string>
#endif

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

    // Fetch PPM from Chrony
    std::optional<double> chrony_ppm_opt = getChronyPPM();

    // Check if Chrony is available
    if (!chrony_ppm_opt.has_value())
    {
        ppm_value = measureClockDrift(clock_drift_interval); // Use system clock fallback
        return PPMStatus::ERROR_CHRONY_NOT_FOUND;
    }

    // Safely extract the Chrony PPM value
    ppm_value = *chrony_ppm_opt;

    // Start the loop
    startPPMUpdateLoop();

    return PPMStatus::SUCCESS;
}

/**
 * @brief Registers a callback to be invoked when the PPM value changes.
 *
 * The callback function receives the new PPM value.
 *
 * @param callback A function or lambda that takes a double PPM value.
 */
void PPMManager::setPPMCallback(std::function<void(double)> callback)
{
    std::lock_guard<std::mutex> lock(ppm_mutex);
    ppm_callback = std::move(callback);
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
std::optional<double> PPMManager::getChronyPPM()
{
    FILE *pipe = popen("chronyc tracking | grep 'Frequency' | awk '{print $3}'", "r");
    if (!pipe)
        return std::nullopt;

    char buffer[128];
    std::string result;

    if (fgets(buffer, sizeof(buffer), pipe) != nullptr)
    {
        result = buffer;
    }

    pclose(pipe);

    try
    {
        double ppm = std::stod(result);
        if (ppm < -500.0 || ppm > 500.0 || std::isnan(ppm))
        {
            return std::nullopt; // Chrony value out of valid range
        }
        return ppm;
    }
    catch (...)
    {
        return std::nullopt; // Failed to parse PPM value
    }
}

/**
 * @brief Measures clock drift over a specified duration.
 *
 * Uses `clock_gettime()` to compare elapsed time with expected time,
 * then calculates PPM deviation.
 *
 * @param seconds The duration in seconds to measure drift
 * @return The calculated PPM drift value.
 */
double PPMManager::measureClockDrift(int seconds)
{
    struct timespec start_real, end_real;
    struct timespec start_mono, end_mono;

    clock_gettime(CLOCK_REALTIME, &start_real);
    clock_gettime(CLOCK_MONOTONIC, &start_mono);

    const int check_interval = 1;
    int elapsed_seconds = 0;

    while (running && elapsed_seconds < seconds)
    {
        std::this_thread::sleep_for(std::chrono::seconds(check_interval));
        if (!running)
            return 0.0;
        elapsed_seconds += check_interval;
    }

    clock_gettime(CLOCK_REALTIME, &end_real);
    clock_gettime(CLOCK_MONOTONIC, &end_mono);

    double elapsed_real = (end_real.tv_sec - start_real.tv_sec) +
                          (end_real.tv_nsec - start_real.tv_nsec) / 1e9;
    double expected = static_cast<double>(seconds);

    // Scale PPM calculation by 1e3 instead of 1e6 to reduce sensitivity
    double drift_real = ((elapsed_real - expected) / expected) * 1e3;

    // Store in history for averaging
    ppm_history.push_back(drift_real);
    if (ppm_history.size() > max_history_size)
    {
        ppm_history.pop_front();
    }

    double avg_measured_ppm = std::accumulate(ppm_history.begin(), ppm_history.end(), 0.0) / ppm_history.size();

#ifdef DEBUG_PPMMANAGER
    double elapsed_mono = (end_mono.tv_sec - start_mono.tv_sec) +
                          (end_mono.tv_nsec - start_mono.tv_nsec) / 1e9;
    double drift_mono = ((elapsed_mono - expected) / expected) * 1e3;
    std::cerr << "[DEBUG] Clock Drift Measurement" << std::endl;
    std::cerr << "  Start (REALTIME): " << start_real.tv_sec << "." << start_real.tv_nsec << "s" << std::endl;
    std::cerr << "  End (REALTIME):   " << end_real.tv_sec << "." << end_real.tv_nsec << "s" << std::endl;
    std::cerr << "  Elapsed (REALTIME): " << elapsed_real << "s" << std::endl;
    std::cerr << "  Start (MONOTONIC): " << start_mono.tv_sec << "." << start_mono.tv_nsec << "s" << std::endl;
    std::cerr << "  End (MONOTONIC):   " << end_mono.tv_sec << "." << end_mono.tv_nsec << "s" << std::endl;
    std::cerr << "  Elapsed (MONOTONIC): " << elapsed_mono << "s" << std::endl;
    std::cerr << "  Expected duration: " << expected << "s" << std::endl;
    std::cerr << "  Drift (REALTIME): " << drift_real << " ppm" << std::endl;
    std::cerr << "  Drift (MONOTONIC): " << drift_mono << " ppm" << std::endl;
    std::cerr << "  Smoothed Measured PPM: " << avg_measured_ppm << " ppm" << std::endl;
#endif

    return avg_measured_ppm;
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
        // Fetch Chrony PPM
        std::optional<double> chrony_ppm_opt = getChronyPPM();
        bool chrony_available = chrony_ppm_opt.has_value();
        double chrony_ppm = chrony_available ? *chrony_ppm_opt : 0.0;

        // Measure clock drift (use a longer sample interval)
        double measured_ppm = measureClockDrift(1025); // Use Chrony update interval

        // If measured PPM differs by more than 10x from Chrony, discard it
        if (chrony_available && std::abs(measured_ppm - chrony_ppm) > 10.0 * chrony_ppm)
        {
            measured_ppm = chrony_ppm;
        }

        double final_ppm = chrony_available ? chrony_ppm : measured_ppm;

        // Update ppm_value if there's a significant change
        {
            std::lock_guard<std::mutex> lock(ppm_mutex);
            if (std::abs(final_ppm - ppm_value.load()) > 0.01)
            {
                ppm_value = final_ppm;

                ppm_history.push_back(final_ppm);
                if (ppm_history.size() > max_history_size)
                {
                    ppm_history.pop_front();
                }

                if (ppm_callback)
                {
                    ppm_callback(final_ppm);
                }
            }
        }

#ifdef DEBUG_PPMMANAGER
        // Print debug information
        auto now = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);
        std::cout << "[" << std::put_time(std::localtime(&now_c), "%Y-%m-%d %H:%M:%S") << "] "
                  << "Chrony PPM: " << chrony_ppm << " | Measured PPM: " << measured_ppm
                  << " | Final PPM: " << final_ppm << std::endl;
#endif

        // Sleep for interval_seconds, checking `running` status every second
        for (int i = 0; i < interval_seconds; i += check_interval)
        {
            if (!running)
                return PPMStatus::SUCCESS;
            std::this_thread::sleep_for(std::chrono::seconds(check_interval));
        }
    }

    return PPMStatus::SUCCESS;
}

std::optional<double> PPMManager::getChronyUpdateInterval()
{
    FILE *pipe = popen("chronyc tracking | grep 'Update interval' | awk '{print $4}'", "r");
    if (!pipe)
        return std::nullopt;

    char buffer[128];
    std::string result;

    if (fgets(buffer, sizeof(buffer), pipe) != nullptr)
    {
        result = buffer;
    }

    pclose(pipe);

    try
    {
        return std::stod(result);
    }
    catch (...)
    {
        return std::nullopt;
    }
}

double PPMManager::getDriftVariance()
{
    if (ppm_history.size() < 2)
    {
        return 0.0; // Not enough data
    }

    double mean = 0.0;
    for (double ppm : ppm_history)
    {
        mean += ppm;
    }
    mean /= ppm_history.size();

    double variance = 0.0;
    for (double ppm : ppm_history)
    {
        variance += (ppm - mean) * (ppm - mean);
    }
    variance /= ppm_history.size();

    return variance;
}

double PPMManager::calculateSmoothingFactor(double last_n_variance, double chrony_update_interval)
{
    // Adjust smoothing factor based on variance and Chrony update interval
    double variance_adjustment = std::min(1.0, 1.0 / (1.0 + last_n_variance / 10.0));
    double update_rate_adjustment = std::min(1.0, 1000.0 / chrony_update_interval);

    return 0.2 * variance_adjustment * update_rate_adjustment; // Base smoothing factor * variance modifier * update frequency modifier
}

double PPMManager::adjustChronyWeight(double chrony_update_interval)
{
    // If Chrony updates quickly (< 600s), prioritize it more
    if (chrony_update_interval < 600)
        return 85.0;

    // If Chrony updates slowly (> 1200s), reduce reliance
    if (chrony_update_interval > 1200)
        return 65.0;

    // Otherwise, adjust linearly
    return 75.0 - ((chrony_update_interval - 600) / 600.0) * 10.0;
}

/**
 * @brief Starts the PPM update loop in a background thread.
 *
 * Ensures that only one instance of the loop runs at a time.
 */
PPMStatus PPMManager::startPPMUpdateLoop()
{
    if (running)
    {
        return PPMStatus::SUCCESS;
    }
    running = true;
    ppm_thread = std::thread(&PPMManager::ppmUpdateLoop, this, ppm_update_interval);

    // Define the desired scheduling policy and use the provided priority.
    int policy = SCHED_RR; // Round-robin scheduling; alternatives include SCHED_FIFO
    sched_param sch_params;
    sch_params.sched_priority = ppm_loop_priority; // Use the optional parameter (defaults to 10)

    // Set the thread's scheduling policy and priority.
    int ret = pthread_setschedparam(ppm_thread.native_handle(), policy, &sch_params);

    if (ret != 0)
    {
#ifdef DEBUG_PPMMANAGER
        std::cerr << "Failed to set thread priority: " << std::strerror(ret) << std::endl;
#endif
        ;;
    }

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
