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
     * @brief Sets the scheduling policy and priority of the signal handling thread.
     *
     * @details
     * Uses `pthread_setschedparam()` to adjust the real-time scheduling policy and
     * priority of the signal handling worker thread.
     *
     * This function is useful for raising the importance of the signal handling
     * thread under high system load, especially when using `SCHED_FIFO` or
     * `SCHED_RR`.
     *
     * @param schedPolicy The scheduling policy (e.g., `SCHED_FIFO`, `SCHED_RR`, `SCHED_OTHER`).
     * @param priority The thread priority value to assign (depends on policy).
     *
     * @return `true` if the scheduling parameters were successfully applied,
     *         `false` otherwise.
     */
    bool setPriority(int schedPolicy, int priority);

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
     * @brief Check if the Chrony daemon is active under systemd.
     *
     * Invokes “systemctl is-active --quiet chronyd” and returns true
     * only if the Chrony service is running. This is the most direct
     * way to verify that your NTP client is alive when you have systemd.
     *
     * @return true if the Chrony service is active; false otherwise.
     */
    bool isChronyAlive();

    /**
     * @brief Registers a callback to be invoked when the PPM value changes.
     *
     * The callback function receives the new PPM value.
     *
     * @param callback A function or lambda that takes a double PPM value.
     */
    void setPPMCallback(std::function<void(double)> callback);

private:
    std::atomic<double> ppm_value_;            ///< Stores the current PPM value.
    std::mutex ppm_mutex_;                     ///< Ensures thread-safe access to the PPM value.
    std::atomic<bool> running_;                ///< Indicates whether the update loop is running.
    std::thread ppm_thread_;                   ///< Background thread for PPM updates.
    std::function<void(double)> ppm_callback_; ///< Callback function for PPM updates.

    static constexpr int clock_drift_interval_ = 300; ///< Interval in seconds for measuring clock drift.
    static constexpr int ppm_update_interval_ = 120;  ///< Interval in seconds between PPM updates.
    static constexpr int ppm_loop_priority_ = 10;     ///< Default scheduling priority.

    std::deque<double> ppm_history_;                ///< Queue to hold historical PPM values.
    static constexpr size_t max_history_size_ = 10; ///< Max size of ppm_history_.

    /**
     * @brief Retrieves the initial PPM value from Chrony.
     *
     * @return The PPM value from Chrony, or `std::nullopt` on failure.
     */
    std::optional<double> get_chrony_ppm();

    /**
     * @brief Measures clock drift over a specified duration.
     *
     * Uses system time functions to measure how much the clock drifts
     * over a given duration and calculates PPM deviation.
     *
     * @param seconds The duration in seconds to measure clock drift.
     * @return The calculated PPM drift value.
     */
    double measure_clock_drift(int seconds = clock_drift_interval_);

    /**
     * @brief Starts the PPM update loop in a background thread.
     *
     * Ensures that only one instance of the loop runs at a time.
     *
     * @return A PPMStatus indicating success.
     */
    PPMStatus start_ppm_update_loop();

    /**
     * @brief The internal update loop for recalculating PPM.
     *
     * Runs in a background thread and periodically updates the PPM value.
     *
     * @param interval_seconds The interval in seconds between PPM updates.
     * @return A PPMStatus indicating success or a warning if an anomaly is detected.
     */
    PPMStatus ppm_update_loop(int interval_seconds);
};

/**
 * @brief Global instance of the PPMManager class.
 *
 * Responsible for measuring and managing frequency drift (PPM)
 * using either NTP data or internal estimation methods.
 */
extern PPMManager ppmManager;

#endif // PPM_MANAGER_HPP
