# Satellite Communications API

A C++ library for determining satellite-to-point transmission visibility, accounting for satellite movement, beam direction, and planet occlusion.

## Overview

This library provides tools to determine:
1. Whether a point in 3D space can receive a transmission from a satellite at a given time
2. When the next time is that a point will be able to receive a transmission

The solution accounts for:
- The satellite's position and beam direction changing over time
- Interpolation between known state samples
- Whether the point is within the satellite's beam cone
- Whether the planet blocks the transmission path

## Features

- Linear interpolation of satellite position and beam direction over time
- Efficient cone angle calculation using dot products
- Planet occlusion detection using geometric line-sphere intersection
- Binary search refinement for finding exact transmission times
- Comprehensive unit tests with Google Test
- Clear, well-documented API with meaningful error handling

## Project Structure

```
Readme.md              # This file
SatelliteComms.cpp     # Implementation of the satellite communications system
SatelliteComms.h       # Header declaring the API and key data structures
main.cpp               # Example application demonstrating API usage
tests/
    test.cpp           # Comprehensive unit tests
```

## API Usage

```cpp
// Initialize with planet radius, beam cone angle, and state timeline
SatelliteComms comms(planetRadius, beamConeAngle, stateTimeline);

// Check if a point can receive transmission at a given time
bool canReceive = comms.canReceiveTransmission(pointP, time);

// Find next time transmission is possible (if any)
std::optional<double> nextTime = comms.nextTransmissionTime(pointP, startTime);
```

## Building and Testing

### Prerequisites
- C++17 compatible compiler
- CMake 3.10 or higher

### Build Commands
```bash
mkdir build && cd build
cmake ..
cmake --build .
```

### Run Tests
```bash
ctest
# or directly:
./SatelliteTests
```

### Run Example Application
```bash
./SatelliteApp
```

## Design Considerations

### Key Assumptions
- Planet is centered at origin (0,0,0)
- Beam cone has its apex at the satellite position and extends along the beam direction
- Linear interpolation is adequate for positions between samples
- Timeline is chronologically ordered and contains sufficient samples

### Performance Optimization
- Dot product for angle calculation rather than explicit trigonometry
- Binary search refinement for efficient next transmission time calculation
- Pre-computing beam cone angle in radians

