/**
 * @file ppm_manager.hpp
 * @brief Header file for the PPMManager class.
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

#ifndef PPM_MANAGER_HPP
#define PPM_MANAGER_HPP

#include <atomic>
#include <deque>
#include <functional>
#include <mutex>
#include <optional>
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

    /**
     * @brief Registers a callback to be invoked when the PPM value changes.
     *
     * The callback function receives the new PPM value.
     *
     * @param callback A function or lambda that takes a double PPM value.
     */
    void setPPMCallback(std::function<void(double)> callback);

private:
    std::atomic<double> ppm_value;                   ///< Stores the current PPM value.
    std::mutex ppm_mutex;                            ///< Ensures thread-safe access to the PPM value.
    std::atomic<bool> running;                       ///< Indicates whether the update loop is running.
    std::thread ppm_thread;                          ///< Background thread for PPM updates.
    std::function<void(double)> ppm_callback;        ///< Callback function for PPM updates
    static constexpr int clock_drift_interval = 300; ///< Internval in seconds for measureClockDrift()
    static constexpr int ppm_update_interval = 300;  ///< Internval in seconds for startPPMUpdateLoop()
    static constexpr int ppm_loop_priority = 10;     ///< Default scheduling priority

    // PPM Source Weighting: These need not add up to 100, they will be normalized to % of 100
    double chrony_weight = 75;   ///< Weight given to chrony values when operating in mixed mode
    double measured_weight = 25; ///< Weight given to measured values when operating in mixed mode

    std::deque<double> ppm_history;
    static constexpr size_t max_history_size = 10;

    double calculateSmoothingFactor(double last_n_variance, double chrony_update_interval);
    double getDriftVariance();
    std::optional<double> getChronyUpdateInterval();
    double adjustChronyWeight(double chrony_update_interval);

    /**
     * @brief Retrieves the initial PPM value from Chrony.
     *
     * @return The PPM value from Chrony, or a fallback value on failure.
     */
    std::optional<double> getChronyPPM();

    /**
     * @brief Measures clock drift over a specified duration.
     *
     * Uses system time functions to measure how much the clock drifts
     * over a given duration and calculates PPM deviation.
     *
     * @param seconds The duration in seconds to measure clock drift.
     * @return The calculated PPM drift value.
     */
    double measureClockDrift(int seconds = clock_drift_interval);

    /**
     * @brief Starts the PPM update loop in a background thread.
     *
     * Ensures that only one instance of the loop runs at a time.
     *
     * @param interval_seconds The interval in seconds between PPM updates.
     */
    PPMStatus startPPMUpdateLoop();

    /**
     * @brief The internal update loop for recalculating PPM.
     *
     * Runs in a background thread and periodically updates the PPM value.
     *
     * @param interval_seconds The interval in seconds between PPM updates.
     * @param priority The scheduling priority for the thread.
     * @return A PPMStatus indicating success or a warning if an anomaly is detected.
     */
    PPMStatus ppmUpdateLoop(int interval_seconds);
};

#endif // PPM_MANAGER_HPP
