# Satellite Communications API

Technical assignment for Progranda application as Team Lead in Simulation.

A C++17 library for determining satellite-to-point transmission visibility, accounting for satellite movement, beam direction, and planet occlusion.

## System Architecture Overview

The Satellite Communications System determines when a point in 3D space can receive transmissions from a satellite in orbit. The architecture follows object-oriented design principles with a focus on performance, accuracy, and maintainability.

### Key Capabilities
1. Determine if a point can receive a transmission at a specific time
2. Find the next time a point will be able to receive a transmission

### Core Components

#### Vector3 Class
The foundation of spatial calculations representing a point or direction in 3D space.
- Basic vector operations (subtraction, addition, scaling)
- Magnitude calculation with SIMD optimization
- Normalization and dot product computation
- Distance and cross product operations

#### Quaternion Class
Provides robust rotational interpolation for accurately representing changes in beam direction.
- Construction from vector pairs
- Spherical Linear Interpolation (SLERP)
- Vector rotation with proper great circle paths
- Handling of edge cases (parallel/anti-parallel vectors)

#### Satellite State Representation
- `SatelliteState`: Position and beam direction at a specific moment
- `TimedSatelliteState`: Pairs a state with a timestamp

#### SatelliteComms Class
The main API class implementing transmission visibility determination.
- Timeline management and validation
- State interpolation between samples
- Beam cone visibility checking
- Planet occlusion detection
- Transmission time prediction
- State caching for performance

## Core Algorithms

### State Interpolation
For any given time point:
1. Binary search finds surrounding timeline entries
2. Linear interpolation calculates position
3. Quaternion SLERP computes correctly interpolated beam direction
4. Results are cached with expiration time

### Beam Cone Visibility
Determines if a point is within the satellite's transmission cone:
1. Calculate vector from satellite to point
2. Compute dot product with beam direction
3. Compare against pre-computed cosine of beam cone angle

### Planetary Occlusion
Checks if the planet blocks the transmission path:
1. Project the satellite position onto the line-of-sight
2. Find the closest approach to planet center
3. Compare this distance to the planet radius

### Next Transmission Time
Uses a hybrid search approach:
1. Linear search with adaptive step size
2. Binary search refinement for precision
3. Early exit conditions for optimization

## Performance Optimizations

The system implements several performance enhancements:

### Hardware Acceleration
- SIMD instructions (SSE3/SSE2) for vector operations
- Graceful degradation on systems without SSE support

### Memory Management
- Time-limited caching with TTL mechanism
- Maximum cache size constraints
- Periodic cleanup of expired entries

### Computational Efficiency
- Pre-computed constants (beam angle cosine)
- Early exit conditions in algorithms
- Dot product for angle comparison instead of arc-cosine
- Adaptive step sizing based on orbital characteristics

## API Usage

```cpp
// Initialize with planet radius, beam cone angle, and state timeline
SatelliteComms comms(planetRadius, beamConeAngleDegrees, stateTimeline);

// Check if a point can receive transmission at a given time
bool canReceive = comms.canReceiveTransmission(pointP, time);

// Find next time transmission is possible (if any)
std::optional<double> nextTime = comms.nextTransmissionTime(pointP, startTime);

// For applications with memory constraints or long lifetimes
comms.clearCache();  // Clear the state interpolation cache
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
- Beam cone has its apex at the satellite position
- Timeline is chronologically ordered
- Time is measured from a universal datum

### Error Handling
- Constructor validation for timeline ordering and sample count
- Safe vector normalization with edge case handling
- Range checking for time values
- `std::optional` return for potentially impossible operations

## Extension Possibilities

The architecture supports several potential extensions:
- Multiple satellites tracking
- Moving reception points
- Variable beam parameters
- Extended visibility window forecasting
- Different interpolation strategies for specialized orbits

## Project Structure

- `Vector3.h`: 3D vector implementation with SIMD optimization
- `Quaternion.h`: Quaternion implementation for direction interpolation
- `SatelliteComms.h/.cpp`: Main API implementation
- `main.cpp`: Example application demonstrating API usage
- `tests/`: Comprehensive test suite
  - Basic functionality tests
  - Edge case handling
  - Advanced orbital configurations
  - Performance benchmarking
