# PPMManager

## Overview

`PPMManager` is a modern C++ class designed to measure and adjust the parts-per-million (PPM) clock drift of a system. It supports **Chrony** for precise time tracking and automatically falls back to system time measurement if Chrony is unavailable.

## Features

- **Initial PPM Calculation** from Chrony (if installed)
- **Automatic Clock Drift Measurement** using `clock_gettime()`
- **Thread-Safe Background PPM Updates**
- **Fails Gracefully** if Chrony is not found
- **Start and Stop Methods** to control background updates

## Installation

Ensure that **Chrony** is installed for more accurate PPM calculations:

```sh
sudo apt install chrony -y
```

## Compilation

Compile the project with the following command (or use the `Makefile` included):

```sh
g++ -std=c++17 -o ppm_adjust main.cpp PPMManager.cpp -lpthread -latomic
```

## Usage

### Basic Example

```cpp
#include "PPMManager.hpp"
#include <iostream>

int main()
{
    PPMManager ppmManager;
    ppmManager.start();

    std::cout << "Current PPM: " << ppmManager.getPPM() << std::endl;

    // Run for a while...
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
| `PPMManager()` | Constructor, initializes PPM detection. |
| `void start()` | Starts the background PPM update thread. |
| `void stop()` | Stops the background PPM update thread. |
| `double getPPM() const` | Returns the current PPM value. |

## Troubleshooting

### Q: I get an error: `Warning: Chrony not found.`

Make sure **Chrony** is installed:

```sh
sudo apt install chrony -y
```

### Q: How do I stop the background thread?

Call `ppmManager.stop();` before exiting the program.

## License

This project is released under the **[MIT License](./LICENSE.md)**.
