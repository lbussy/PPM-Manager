#ifndef PPM_MANAGER_HPP
#define PPM_MANAGER_HPP

#include <atomic>
#include <mutex>
#include <thread>

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
     * Initializes the PPM tracking system but does not start the update loop.
     */
    PPMManager();

    /**
     * @brief Destroys the PPMManager instance.
     *
     * Ensures the update loop is properly stopped before destruction.
     */
    ~PPMManager();

    /**
     * @brief Starts the PPM update loop.
     *
     * This function launches a background thread that periodically updates the PPM value
     * by measuring clock drift. The update interval is configurable.
     *
     * @param interval_seconds The interval in seconds between PPM updates (default: 600s).
     */
    void startPPMUpdateLoop(int interval_seconds = 600);

    /**
     * @brief Stops the PPM update loop.
     *
     * Terminates the background thread safely.
     */
    void stop();

    /**
     * @brief Retrieves the latest calculated PPM value.
     *
     * This function provides a thread-safe way to get the most recent PPM measurement.
     *
     * @return The current PPM value.
     */
    double getCurrentPPM();

    /**
     * @brief Checks whether the system time is synchronized.
     *
     * This function determines if the system clock is synchronized with an external time source.
     *
     * @return True if time is synchronized, false otherwise.
     */
    bool isTimeSynchronized();

private:
    std::atomic<double> ppm_value;   ///< Stores the current PPM value.
    std::mutex ppm_mutex;            ///< Ensures thread-safe access to the PPM value.
    std::atomic<bool> running;       ///< Indicates whether the update loop is running.
    std::thread ppm_thread;          ///< Background thread for PPM updates.

    /**
     * @brief Retrieves the initial PPM value from Chrony.
     *
     * If Chrony is unavailable, it returns a fallback value.
     *
     * @return The PPM value from Chrony, or a fallback value on failure.
     */
    double getChronyPPM();

    /**
     * @brief Measures clock drift over a specified duration.
     *
     * This function calculates how much the system clock drifts over a given number of seconds.
     *
     * @param seconds The duration in seconds to measure clock drift (default: 300s).
     * @return The calculated PPM drift value.
     */
    double measureClockDrift(int seconds = 300);

    /**
     * @brief The internal update loop for recalculating PPM.
     *
     * Runs in a background thread, periodically updating the PPM value.
     *
     * @param interval_seconds The interval in seconds between PPM updates.
     */
    void ppmUpdateLoop(int interval_seconds);
};

#endif // PPM_MANAGER_HPP
