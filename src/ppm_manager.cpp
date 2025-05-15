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
#include <cerrno>
#include <cstring>
#include <ctime>
#include <iostream>
#include <sstream>
#include <string>
#endif

/**
 * @brief Global instance of the PPMManager class.
 *
 * Responsible for measuring and managing frequency drift (PPM)
 * using either NTP data or internal estimation methods.
 */
PPMManager ppmManager;

/**
 * @brief Constructs a PPMManager instance.
 *
 * Initializes internal values but does not start the update loop.
 */
PPMManager::PPMManager() : ppm_value_(0.0), running_(false) {}

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
    // Simple chck to make sure chrony is alive, PPM can update later
    if (!isChronyAlive())
    {
        return PPMStatus::ERROR_UNSYNCHRONIZED_TIME;
    }

    // Fetch PPM from Chrony
    std::optional<double> chrony_ppm_opt = get_chrony_ppm();

    // Check if Chrony is available
    if (!chrony_ppm_opt.has_value())
    {
        ppm_value_.store(measure_clock_drift(clock_drift_interval_)); // Use system clock fallback
        return PPMStatus::ERROR_CHRONY_NOT_FOUND;
    }

    // Safely extract the Chrony PPM value
    ppm_value_.store(*chrony_ppm_opt);

    if (ppm_callback_)
        ppm_callback_(ppm_value_.load());

#ifdef DEBUG_PPMMANAGER
    std::cerr << "[DEBUG] :init() initial PPM = " << ppm_value_.load() << " ppm\n";
#endif

    // Start the loop
    start_ppm_update_loop();

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
    std::lock_guard<std::mutex> lock(ppm_mutex_);
    ppm_callback_ = std::move(callback);
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
 *         `false` otherwise (e.g., thread not running_ or `pthread_setschedparam()` failed).
 *
 * @note
 * The caller may require elevated privileges (e.g., CAP_SYS_NICE) to apply real-time priorities.
 * It is the caller's responsibility to ensure the priority value is valid for the given policy.
 */
bool PPMManager::setPriority(int schedPolicy, int priority)
{
    // Ensure that the worker thread is active and joinable
    if (!ppm_thread_.joinable())
    {
        return false;
    }

    // Set up the scheduling parameters
    sched_param sch_params;
    sch_params.sched_priority = priority;

    // Attempt to apply the scheduling policy and priority
    int ret = pthread_setschedparam(ppm_thread_.native_handle(), schedPolicy, &sch_params);

    return (ret == 0);
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
    // Ask chrony for its sources, silencing any errors
    FILE *pipe = popen("chronyc sources -n 2>/dev/null", "r");
    if (!pipe)
    {
        return false;
    }

    char buffer[128];
    bool synced = false;

    // Read each line; a leading '*' means we have a valid sync source
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr)
    {
        if (buffer[0] == '*')
        {
            synced = true;
            break;
        }
    }

    pclose(pipe);
    return synced;
}

/**
 * @brief Check if the Chrony daemon is active under systemd.
 *
 * Invokes “systemctl is-active --quiet chronyd” and returns true
 * only if the Chrony service is running. This is the most direct
 * way to verify that your NTP client is alive when you have systemd.
 *
 * @return true if the Chrony service is active; false otherwise.
 */
bool PPMManager::isChronyAlive()
{
    // This returns 0 if the service is active, non-zero otherwise
    int ret = std::system("systemctl is-active --quiet chronyd");
    // On most Linuxes, system() returns the child’s exit status directly,
    // so zero means “active”.
    return (ret == 0);
}

/**
 * @brief Retrieves the initial PPM value from Chrony.
 *
 * Uses `chronyc tracking` to get the frequency offset in PPM.
 * Returns `-9999.0` if Chrony is unavailable.
 *
 * @return The PPM value from Chrony, or `-9999.0` on failure.
 */
std::optional<double> PPMManager::get_chrony_ppm()
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
double PPMManager::measure_clock_drift(int seconds)
{
    struct timespec start_real, end_real;
    struct timespec start_mono, end_mono;

    clock_gettime(CLOCK_REALTIME, &start_real);
    clock_gettime(CLOCK_MONOTONIC, &start_mono);

    const int check_interval = 1;
    int elapsed_seconds = 0;

    while (running_ && elapsed_seconds < seconds)
    {
        std::this_thread::sleep_for(std::chrono::seconds(check_interval));
        if (!running_)
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
    ppm_history_.push_back(drift_real);
    if (ppm_history_.size() > max_history_size_)
    {
        ppm_history_.pop_front();
    }

    double avg_measured_ppm = std::accumulate(ppm_history_.begin(), ppm_history_.end(), 0.0) / ppm_history_.size();

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
 * The loop continues running_ while `running_` is true.
 *
 * @param interval_seconds The interval in seconds between PPM updates.
 */
PPMStatus PPMManager::ppm_update_loop(int interval_seconds)
{
    const int check_interval = 1;

    while (running_)
    {
        // Fetch Chrony PPM
        std::optional<double> chrony_ppm_opt = get_chrony_ppm();
        bool chrony_available = chrony_ppm_opt.has_value();
        double chrony_ppm = chrony_available ? *chrony_ppm_opt : 0.0;

        // Measure clock drift (use a longer sample interval)
        double measured_ppm = measure_clock_drift(1025); // Use Chrony update interval

        // If measured PPM differs by more than 10x from Chrony, discard it
        if (chrony_available && std::abs(measured_ppm - chrony_ppm) > 10.0 * chrony_ppm)
        {
            measured_ppm = chrony_ppm;
        }

        double final_ppm = chrony_available ? chrony_ppm : measured_ppm;

        // Update ppm_value_ if there's a significant change
        {
            std::lock_guard<std::mutex> lock(ppm_mutex_);
            if (std::abs(final_ppm - ppm_value_.load()) > 0.01)
            {
                ppm_value_.store(final_ppm);

                ppm_history_.push_back(final_ppm);
                if (ppm_history_.size() > max_history_size_)
                {
                    ppm_history_.pop_front();
                }

                if (ppm_callback_)
                {
                    // Capture the callback and value by copy, then detach
                    std::thread([cb = ppm_callback_, val = final_ppm]()
                                { cb(val); })
                        .detach();
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

        // Sleep for interval_seconds, checking `running_` status every second
        for (int i = 0; i < interval_seconds; i += check_interval)
        {
            if (!running_)
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
 */
PPMStatus PPMManager::start_ppm_update_loop()
{
    if (running_)
    {
        return PPMStatus::SUCCESS;
    }
    running_ = true;
    ppm_thread_ = std::thread(&PPMManager::ppm_update_loop, this, ppm_update_interval_);

    // Define the desired scheduling policy and use the provided priority.
    int policy = SCHED_RR; // Round-robin scheduling; alternatives include SCHED_FIFO
    sched_param sch_params;
    sch_params.sched_priority = ppm_loop_priority_; // Use the optional parameter (defaults to 10)

    // Set the thread's scheduling policy and priority.
    int ret = pthread_setschedparam(ppm_thread_.native_handle(), policy, &sch_params);

    if (ret != 0)
    {
#ifdef DEBUG_PPMMANAGER
        std::cerr << "Failed to set thread priority: " << ::strerror(ret) << std::endl;
#endif
        ;
        ;
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
    if (running_)
    {
        running_ = false;
        if (ppm_thread_.joinable())
        {
            ppm_thread_.join();
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
    std::lock_guard<std::mutex> lock(ppm_mutex_);
#ifdef DEBUG_PPMMANAGER
    std::cout << "[DEBUG] :getCurrentPPM() PPM Value: " << ppm_value_ << std::endl;
#endif
    return ppm_value_;
}
