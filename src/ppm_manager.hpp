/**
 * @file ppm_manager.hpp
 * @brief Header file for the PPMManager class.
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
 *
 * Copyright (C) 2025 Lee C. Bussy (@LBussy). All rights reserved.
 */

#ifndef PPM_MANAGER_HPP
#define PPM_MANAGER_HPP

#include <atomic>
#include <mutex>
#include <thread>

/**
 * @enum PPMStatus
 * @brief Defines possible return statuses for PPMManager operations.
 */
enum class PPMStatus
{
    SUCCESS,                  ///< Operation completed successfully.
    WARNING_HIGH_PPM,         ///< Measured PPM exceeds a safe threshold.
    ERROR_CHRONY_NOT_FOUND,   ///< Chrony was not found, falling back to clock drift measurement.
    ERROR_UNSYNCHRONIZED_TIME ///< System time is not synchronized.
};

/**
 * @class PPMManager
 * @brief Manages periodic PPM (Parts Per Million) calculations to track clock drift.
 *
 * This class retrieves the initial PPM from Chrony (if available) and periodically
 * updates the PPM value by measuring clock drift using system timing functions.
 */
class PPMManager
{
public:
    /**
     * @brief Constructs a PPMManager instance.
     *
     * Initializes internal values but does not start the update loop.
     */
    PPMManager();

    /**
     * @brief Destroys the PPMManager instance.
     *
     * Ensures the update loop is properly stopped before destruction.
     */
    ~PPMManager();

    /**
     * @brief Initializes the PPMManager by checking synchronization and obtaining initial PPM.
     *
     * @return A PPMStatus indicating success or failure reason.
     */
    PPMStatus initialize();

    /**
     * @brief Starts the PPM update loop.
     *
     * Runs in a background thread and periodically updates the PPM value.
     *
     * @param interval_seconds The interval in seconds between PPM updates (default: 600s).
     * @return A PPMStatus indicating whether the loop started successfully.
     */
    PPMStatus startPPMUpdateLoop(int interval_seconds = 600);

    /**
     * @brief Stops the PPM update loop.
     *
     * Ensures the background thread terminates properly before stopping.
     *
     * @return A PPMStatus indicating whether the loop stopped successfully.
     */
    PPMStatus stop();

    /**
     * @brief Retrieves the latest calculated PPM value.
     *
     * Ensures thread-safe access to the PPM value.
     *
     * @return The current PPM value.
     */
    double getCurrentPPM();

    /**
     * @brief Checks whether the system time is synchronized.
     *
     * Uses system utilities to determine if the clock is synchronized.
     *
     * @return True if time is synchronized, false otherwise.
     */
    bool isTimeSynchronized();

private:
    std::atomic<double> ppm_value; ///< Stores the current PPM value.
    std::mutex ppm_mutex;          ///< Ensures thread-safe access to the PPM value.
    std::atomic<bool> running;     ///< Indicates whether the update loop is running.
    std::thread ppm_thread;        ///< Background thread for PPM updates.

    /**
     * @brief Retrieves the initial PPM value from Chrony.
     *
     * @return The PPM value from Chrony, or a fallback value on failure.
     */
    double getChronyPPM();

    /**
     * @brief Measures clock drift over a specified duration.
     *
     * Uses system time functions to measure how much the clock drifts
     * over a given duration and calculates PPM deviation.
     *
     * @param seconds The duration in seconds to measure clock drift (default: 300s).
     * @return The calculated PPM drift value.
     */
    double measureClockDrift(int seconds = 300);

    /**
     * @brief The internal update loop for recalculating PPM.
     *
     * Runs in a background thread and periodically updates the PPM value.
     *
     * @param interval_seconds The interval in seconds between PPM updates.
     * @return A PPMStatus indicating success or a warning if an anomaly is detected.
     */
    PPMStatus ppmUpdateLoop(int interval_seconds);
};

#endif // PPM_MANAGER_HPP
