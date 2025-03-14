# Satellite Communications API

A C++17 library for determining satellite-to-point transmission visibility, accounting for satellite movement, beam direction, and planet occlusion.

This is production-level code designed for practical use in real-world satellite communication systems.

## System Architecture Overview

The Satellite Communications System determines when a point in 3D space can receive transmissions from a satellite in orbit. The architecture follows object-oriented design principles with a focus on performance, accuracy, and maintainability.

### Key Capabilities
1. Determine if a point can receive a transmission at a specific time
2. Find the next time a point will be able to receive a transmission
3. Efficient caching for improved performance in repeated queries

### Core Components

#### Vector3 Class
The foundation of spatial calculations representing a point or direction in 3D space.
- Basic vector operations (subtraction, addition, scaling)
- Magnitude calculation with SIMD optimization
- Normalization and dot product computation
- Distance and cross-product operations

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
4. Results are cached with a two-level caching strategy

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
- Two-level caching strategy with MRU array + map
- Integer-based cache keys for faster lookups
- Time quantization to improve cache hit rates
- Maximum cache size constraints
- Batch removal of entries when cache is full

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

// Timeline information access
double startTime = comms.getTimelineStart();
double endTime = comms.getTimelineEnd();
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

### Run Performance Tests
Performance tests are enabled by default. To disable them:
```bash
cmake -DENABLE_PERFORMANCE_TESTS=OFF ..
cmake --build .
./SatelliteTests
```

### Run Example Application
```bash
./SatelliteApp
```

## Benchmark System

The project includes a comprehensive benchmarking system to evaluate caching performance. The benchmark tests measure how effectively the state interpolation cache improves performance across different scenarios.

### Running the Benchmark

```bash
mkdir build && cd build
cmake ..
cmake --build .
./Benchmark
```

[SampleBenchmark](/resources/benchmark.txt)

### Benchmark Scenarios

The benchmark includes six test scenarios designed to evaluate different aspects of caching performance:

1. **Basic Orbit - Unique Queries (0% cache hits)**  
   Tests performance with unique time queries, providing a baseline without cache benefits.

2. **Complex Orbit - Repeated Queries (98% cache hits)**  
   Tests a complex orbital scenario with highly repeated queries for maximum cache benefit.

3. **Large Timeline - High Cache Benefit (99% hits)**  
   Uses a detailed timeline (2,000 samples) with 20,000 highly repeated queries.

4. **Very Dense Timeline - Sequential Scan (0% hits)**  
   Tests a high-resolution timeline (5,000 samples) with a sequential access pattern.

5. **Real-world Satellite - Clustered Access (20% hits)**  
   Simulates a geostationary satellite with time queries clustered around specific events.

6. **Extreme Case - Random Access (0% hits)**  
   Stress tests the system with a long timeline (10,000 samples) and random access.

### Performance Metrics

Each scenario measures three key performance metrics:

1. **Interpolation Performance**  
   Direct measurement of state interpolation with and without caching.

2. **Transmission Check Performance**  
   Evaluate the impact of caching on determining if a point can receive a transmission.

3. **Next Transmission Time Performance**  
   Measures efficiency when finding the next time a point will receive a transmission.

### Cache Effectiveness

The caching system shows the following characteristics:

- Provides significant speedup (5-12x) for transmission checks across all scenarios
- Achieves 3-4x speedup for next transmission time calculations
- Shows variable performance for direct interpolation, with best results on repeated queries
- May show slightly reduced performance for interpolation with sequential access patterns

Optimal cache performance is achieved with high repeat rates of time queries, which matches the intended usage pattern in satellite communication systems where the same time points are frequently queried for multiple ground locations.

## Design Considerations

### Key Assumptions
- Planet is centred at origin (0,0,0)
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
- Multiple satellite tracking
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
  - `test.cpp`: Basic functionality tests
  - `additional_tests.cpp`: Edge case and specialized tests
  - `performance_tests.cpp`: Performance benchmarks
- `SimpleBenchmark.cpp`: Dedicated benchmark system for caching performance
- `doc/`: Documentation files
  - `Performance.md`: Detailed information about the computational characteristics
  - `Quaternion.md`: Background on quaternion-based interpolation
  - `Rationale.md`: Design decisions and justifications

## Performance Results

Sample benchmark results on a typical system:

| Test Scenario                            | Interpolation | Transmission | NextTime    | Cache Hit% |
|------------------------------------------|---------------|--------------|-------------|------------|
| Basic Orbit - Unique Queries             | 1.05x         | 5.16x        | 3.94x       | 0.0%       |
| Complex Orbit - Repeated Queries         | 10.65x        | 5.90x        | 4.29x       | 98.0%      |
| Large Timeline - High Cache Benefit      | 7.46x         | 5.52x        | 4.30x       | 99.9%      |
| Very Dense Timeline - Sequential Scan    | 0.64x         | 4.97x        | 3.76x       | 0.0%       |
| Real-world Satellite - Clustered Access  | 0.66x         | 5.06x        | 4.08x       | 19.8%      |
| Extreme Case - Random Access             | 0.72x         | 5.22x        | 4.29x       | 0.0%       |

The benchmark demonstrates that the caching system provides substantial performance improvements for operations that involve multiple interpolations (transmission checks and next-time calculations) across all access patterns, while direct interpolation performance varies based on the access pattern.
