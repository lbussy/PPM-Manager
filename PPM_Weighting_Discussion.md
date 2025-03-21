# PPM Source Weighting and Behavior Overview

This document describes how the **PPMManager** class calculates and updates the system's PPM (Parts Per Million) drift using a combination of Chrony data and measured system clock drift. It is designed to be adaptive, reliable, and capable of functioning in environments with or without active NTP synchronization.

## Purpose

PPM (Parts Per Million) indicates the drift between the system clock and real time. An accurate PPM value is critical for time-sensitive applications, especially on systems like the Raspberry Pi.

## Data Sources

The system uses two potential sources for determining the current PPM:

- **Chrony-derived PPM** from `chronyc tracking` when NTP is available.
- **Measured PPM** using system clock drift calculations via `clock_gettime()` when Chrony is not available or is intentionally bypassed.

## Source Selection Logic

The system uses the following logic to determine the PPM value:

- If **Chrony is available**, its PPM value is used as the authoritative source.
- If **Chrony is not available**, the system uses the result of `measureClockDrift()` as the sole PPM source.

This fallback ensures the system can continue to operate in a degraded but functional mode if network synchronization is unavailable.

## Smoothing for Measured PPM

To improve accuracy and stability of the measured PPM value, the system keeps a **sliding history** of recent measurements using a `std::deque`. This provides the basis for smoothing via a moving average:

- A maximum number of historical samples (`max_history_size`) are retained.
- Each new measurement is added to the deque.
- The system calculates a smoothed average from this history.

This smoothing process helps mitigate anomalies and fluctuations that naturally occur in real-time clock measurements.

## Update Cadence and Behavior

The update loop runs every `interval_seconds` (configurable). Each loop iteration:

1. Checks whether Chrony is available.
2. Measures system clock drift if needed.
3. Calculates a smoothed PPM value (if relying on measured data).
4. Updates the current PPM value if it has changed significantly (by more than 0.01 PPM).
5. Triggers a registered callback function with the new value, if one is set.

## Logging and Debugging

Debug output includes:

- Timestamps for each update.
- Chrony PPM.
- Measured PPM.
- Final selected PPM.

These are logged in a structured format that can be used for auditing or graphing the system’s timekeeping performance over time.

## Chrony as Primary Authority

Chrony is treated as the preferred source when available because it provides:

- Long-term averaging.
- Correction for hardware clock drift.
- More accurate, global time alignment.

The measured PPM fallback is provided only to ensure system continuity when Chrony is unavailable.

## Summary

- The system ensures robust timekeeping by prioritizing Chrony and falling back to measured drift.
- Smooth averaging of measured values improves stability.
- Adaptive to connected and disconnected operation.
- Designed for environments where accurate and reliable clock behavior is important.
