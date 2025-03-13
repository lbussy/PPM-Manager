# PPMManager

## Overview

`PPMManager` is a modern C++ class designed to measure and adjust the parts-per-million (PPM) clock drift of a system. It supports **Chrony** for precise time tracking and automatically falls back to system time measurement if Chrony is unavailable.

## Features

- **Initial PPM Calculation** from Chrony (if installed)
- **Automatic Clock Drift Measurement** using `clock_gettime()`
- **Thread-Safe Background PPM Updates**
- **Graceful Handling of Missing Chrony**
- **Start and Stop Methods** to control background updates
- **Status-Based API** for structured error handling

## Installation

Ensure that **Chrony** is installed for more accurate PPM calculations:

```sh
sudo apt install chrony -y
```

## Compilation

Compile the project using the included `Makefile`:

```sh
make
```

Or manually compile with:

```sh
g++ -std=c++17 -o ppm_manager main.cpp ppm_manager.cpp -lpthread -latomic
```

## Usage

### Basic Example

```cpp
#include "ppm_manager.hpp"
#include <iostream>

int main()
{
    PPMManager ppmManager;
    PPMStatus status = ppmManager.initialize();

    if (status != PPMStatus::SUCCESS) {
        std::cerr << "Error: PPM Manager initialization failed." << std::endl;
        return 1;
    }

    ppmManager.startPPMUpdateLoop();

    std::cout << "Current PPM: " << ppmManager.getCurrentPPM() << std::endl;

    std::this_thread::sleep_for(std::chrono::minutes(10));

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
| `PPMStatus startPPMUpdateLoop(int interval_seconds = 600)` | Starts the background PPM update thread. |
| `PPMStatus stop()` | Stops the background PPM update thread. |
| `double getCurrentPPM()` | Returns the current PPM value. |
| `bool isTimeSynchronized()` | Checks if the system time is synchronized. |

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

## License

This project is released under the **[MIT License](./LICENSE.md)**.
