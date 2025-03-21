# PPMManager

## Overview

`PPMManager` is a modern C++ class designed to measure and adjust a system's parts-per-million (PPM) clock drift.  It supports **Chrony** for precise time tracking and automatically falls back to system time measurement if Chrony is unavailable.  The PPM value is updated periodically in a background thread, and users can register a callback to be notified of PPM changes.

## Features

- **Initial PPM Calculation** from Chrony (if installed)
- **Automatic Clock Drift Measurement** using `clock_gettime()`
- **Thread-Safe Background PPM Updates**
- **Graceful Handling of Missing Chrony**
- **Start and Stop Methods** to control background updates
- **Callback Support** for real-time PPM updates
- **Status-Based API** for structured error handling

## Installation

Ensure that **Chrony** is installed for more accurate PPM calculations:

```bash
sudo apt install chrony -y
```

## Compilation

A sample `main.cpp` is included to demonstrate the class's functionality.  If you include this class as a submodule, you should exclude it from compilation or simply delete it if you manually add it to your project.

To compile a functional test using the project with the included `Makefile`:

```bash
make
```

Or manually compile with:

```bash
g++ -std=c++17 -o ppm_manager main.cpp ppm_manager.cpp -lpthread -latomic
```

## Usage

### Basic Example

```cpp
#include "ppm_manager.hpp"
#include <iostream>
#include <thread>
#include <condition_variable>

int main()
{
    PPMManager ppmManager;
    PPMStatus status = ppmManager.initialize();

    if (status != PPMStatus::SUCCESS) {
        std::cerr << "Error: PPM Manager initialization failed." << std::endl;
        return 1;
 }

 // Register a callback lambda to monitor PPM updates
    ppmManager.setPPMCallback([](double ppm) {
        std::cout << "PPM Updated: " << ppm << std::endl;
 });

    ppmManager.startPPMUpdateLoop();

 // Using condition variable to wait for signal-based termination
    std::mutex cv_mutex;
    std::condition_variable cv;
    bool stop_requested = false;

    std::thread signal_thread([&]() {
        std::unique_lock<std::mutex> lock(cv_mutex);
        cv.wait(lock, [&]() { return stop_requested; });
 });

    std::this_thread::sleep_for(std::chrono::minutes(10));
    stop_requested = true;
    cv.notify_all();
    signal_thread.join();

    ppmManager.stop();
    return 0;
}
```

## API Documentation

### Class: `PPMManager`

#### Methods

| Method | Description |
|--------|-------------|
| `PPMManager()` | Constructor, initializes internal values but does not start updates. |
| `PPMStatus initialize()` | Initializes PPMManager, checking Chrony and clock drift. |
| `PPMStatus startPPMUpdateLoop()` | Starts the background PPM update thread. |
| `PPMStatus stop()` | Stops the background PPM update thread. |
| `double getCurrentPPM()` | Returns the current PPM value. |
| `bool isTimeSynchronized()` | Checks if the system time is synchronized. |
| `void setPPMCallback(std::function<void(double)> callback)` | Registers a callback to be invoked when the PPM value changes. |

## Troubleshooting

### Q: I get an error: `Error: PPM Manager initialization failed.`

Make sure **Chrony** is installed:

```sh
sudo apt install chrony -y
```

Or verify that your system clock is synchronized:

```sh
timedatectl status
```

### Q: How do I stop the background thread?

Call `ppmManager.stop();` before exiting the program.

### Q: How do I register a callback to get PPM updates?

Use the `setPPMCallback()` method with a lambda:

```cpp
main()
{

 // ... your code here

    ppmManager.setPPMCallback([](double ppm) {
        std::cout << "New PPM Value: " << ppm << std::endl;
 });

 // ... your code here

}
```

Or in a separate function::

```cpp
double ppm_callback(double new_ppm)
{
    std::cout << "PPM Updated: " << ppm << std::endl;
}

int main()
{

 // ... your code here

    ppmManager.setPPMCallback(ppm_callback);

 // ... your code here

}
```

## License

This project is released under the **[MIT License](./LICENSE.md)**.
