/**
 * @file main.cpp
 * @brief Entry point for the PPM Manager demonstration program.
 *
 * This program initializes the PPMManager, starts worker threads,
 * and handles signal-based termination while ensuring clean shutdown.
 */

#include "ppm_manager.hpp"
#include <iostream>
#include <thread>
#include <vector>
#include <atomic>
#include <csignal>

/// Global instance of PPMManager
PPMManager ppmManager;

/// Atomic flag to control program execution
std::atomic<bool> running(true);

/**
 * @brief Signal handler for graceful shutdown.
 *
 * Captures termination signals (e.g., SIGINT) and updates the running flag.
 *
 * @param signum The signal number received.
 */
void signalHandler(int signum)
{
    std::cout << "\nCaught signal " << signum << ", stopping gracefully..." << std::endl;
    running = false;
}

/**
 * @brief Simulated worker thread function.
 *
 * Each worker runs a computational loop while the `running` flag is true.
 *
 * @param id Unique identifier for the worker thread.
 */
void workerThread(int id)
{
    while (running)
    {
        for (volatile int i = 0; i < 1000000; ++i)
            ; // Simulate workload
    }
}

/**
 * @brief Main function of the program.
 *
 * Initializes the PPMManager, starts worker threads, and waits for termination.
 *
 * @return 0 on successful execution, nonzero on failure.
 */
int main()
{
    // Register signal handler for graceful exit
    std::signal(SIGINT, signalHandler);

    // Initialize PPM Manager
    PPMStatus status = ppmManager.initialize();

    // Handle initialization errors
    if (status == PPMStatus::ERROR_UNSYNCHRONIZED_TIME)
    {
        std::cerr << "System time is not synchronized." << std::endl;
        return 1;
    }
    else if (status == PPMStatus::ERROR_CHRONY_NOT_FOUND)
    {
        std::cerr << "Chrony not found. Using clock drift measurement." << std::endl;
    }

    std::cout << "Current PPM: " << ppmManager.getCurrentPPM() << std::endl;

    // Start PPM update loop
    ppmManager.startPPMUpdateLoop();

    std::vector<std::thread> workers;
    const int num_workers = 4;

    // Launch worker threads
    for (int i = 0; i < num_workers; ++i)
    {
        workers.emplace_back(workerThread, i);
    }

    // Keep running until interrupted
    while (running)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // Graceful shutdown
    std::cout << "Stopping PPM Manager..." << std::endl;
    ppmManager.stop();

    std::cout << "Waiting for worker threads to finish..." << std::endl;
    for (auto &worker : workers)
    {
        worker.join();
    }

    std::cout << "All threads stopped. Exiting." << std::endl;
    return 0;
}
