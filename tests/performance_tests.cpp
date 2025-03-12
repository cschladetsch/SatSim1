#include <gtest/gtest.h>
#include "../SatelliteComms.h"
#include <cmath>
#include <chrono>
#include <vector>
#include <random>
#include <iostream>
#include <iomanip>
#include <functional>

#ifdef ENABLE_PERFORMANCE_TESTS

// Helper function to measure execution time
template<typename Func>
double measureExecutionTime(Func&& func, int iterations, const std::string& testName) {
    auto start = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < iterations; i++) {
        func();
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    
    std::cout << std::setw(45) << testName << " | " << std::setw(8) << iterations
              << " iterations | " << std::setw(10) << duration << " æs | "
              << std::setw(8) << std::fixed << std::setprecision(2) << (double)duration / iterations
              << " æs/iter" << std::endl;
    
    return duration;
}

// Test fixture for performance testing
class SatelliteCommsPerformanceTest : public ::testing::Test {
protected:
    // Constants for test configuration
    static constexpr double ORBIT_RADIUS = 10000.0;
    static constexpr double PLANET_RADIUS = 6371.0;
    static constexpr double BEAM_ANGLE = 15.0;
    static constexpr int NUM_SAMPLES = 100;
    static constexpr double TIME_STEP = 100.0;
    static constexpr double TWO_PI = 2.0 * M_PI;
    
    std::vector<TimedSatelliteState> stateTimeline;
    
    void SetUp() override {
        createCircularOrbitTimeline();
    }
    
    // Creates a circular orbit timeline in the XY plane with more samples
    void createCircularOrbitTimeline() {
        stateTimeline.clear();
        
        for (int i = 0; i < NUM_SAMPLES; i++) {
            double angle = (TWO_PI * i) / NUM_SAMPLES;
            double time = i * TIME_STEP;
            
            Vector3 position = {
                ORBIT_RADIUS * cos(angle),
                ORBIT_RADIUS * sin(angle),
                0.0
            };
            
            // Beam pointing outward from planet center
            Vector3 beamDirection = {
                position.x / ORBIT_RADIUS,
                position.y / ORBIT_RADIUS,
                0.0
            };
            
            stateTimeline.push_back({time, {position, beamDirection}});
        }
    }
    
    // Creates a more complex 3D orbit timeline
    void create3DOrbitTimeline() {
        stateTimeline.clear();
        
        // Orbital inclination in radians
        const double inclination = 0.5;
        
        for (int i = 0; i < NUM_SAMPLES; i++) {
            double angle = (TWO_PI * i) / NUM_SAMPLES;
            double time = i * TIME_STEP;
            
            // Create a 3D orbit with inclination
            Vector3 position = {
                ORBIT_RADIUS * cos(angle),
                ORBIT_RADIUS * sin(angle) * cos(inclination),
                ORBIT_RADIUS * sin(angle) * sin(inclination)
            };
            
            // Beam pointing outward from planet center
            Vector3 beamDirection = {
                position.x / ORBIT_RADIUS,
                position.y / ORBIT_RADIUS,
                position.z / ORBIT_RADIUS
            };
            
            stateTimeline.push_back({time, {position, beamDirection}});
        }
    }
    
    // Generate random points for testing
    std::vector<Vector3> generateRandomPoints(int count, double minRadius, double maxRadius) {
        std::vector<Vector3> points;
        std::mt19937 gen(42); // Fixed seed for reproducibility
        std::uniform_real_distribution<> distRadius(minRadius, maxRadius);
        std::uniform_real_distribution<> distAngle(0, TWO_PI);
        std::uniform_real_distribution<> distZ(-maxRadius, maxRadius);
        
        for (int i = 0; i < count; i++) {
            double radius = distRadius(gen);
            double angle = distAngle(gen);
            double z = distZ(gen);
            
            Vector3 point = {
                radius * cos(angle),
                radius * sin(angle),
                z
            };
            
            points.push_back(point);
        }
        
        return points;
    }
};

// Performance comparison tests
TEST_F(SatelliteCommsPerformanceTest, InterpolationPerformance) {
    // Create the satellite communication system
    SatelliteComms comms(PLANET_RADIUS, BEAM_ANGLE, stateTimeline);
    
    // Print header
    std::cout << "\n=== INTERPOLATION PERFORMANCE TEST ===\n";
    std::cout << std::setw(45) << "Test" << " | " << std::setw(8) << "Count" 
              << " | " << std::setw(10) << "Total Time" << " | " 
              << std::setw(11) << "Time/Call" << std::endl;
    std::cout << std::string(80, '-') << std::endl;
    
    // 1. Interpolate at exact timeline points
    auto testExactPoints = [&comms, this]() -> double {
        double result = 0;
        for (const auto& state : stateTimeline) {
            SatelliteState interpolated = comms.interpolateState(state.timestamp);
            // Force the compiler to use the result
            result += interpolated.position.x;
        }
        return result;
    };
    
    measureExecutionTime(testExactPoints, 1000, "Interpolate at exact timeline points");
    
    // 2. Interpolate at midpoints between timeline samples
    auto testMidpoints = [&comms, this]() -> double {
        double result = 0;
        for (size_t i = 0; i < stateTimeline.size() - 1; i++) {
            double midTime = (stateTimeline[i].timestamp + stateTimeline[i+1].timestamp) / 2.0;
            SatelliteState interpolated = comms.interpolateState(midTime);
            // Force the compiler to use the result
            result += interpolated.position.x;
        }
        return result;
    };
    
    measureExecutionTime(testMidpoints, 1000, "Interpolate at midpoints");
    
    // 3. Interpolate at random times
    auto testRandomTimes = [&comms, this]() -> double {
        double result = 0;
        std::mt19937 gen(42);
        std::uniform_real_distribution<> dist(stateTimeline.front().timestamp, 
                                             stateTimeline.back().timestamp);
        
        for (int i = 0; i < 100; i++) {
            double randomTime = dist(gen);
            SatelliteState interpolated = comms.interpolateState(randomTime);
            // Force the compiler to use the result
            result += interpolated.position.x;
        }
        return result;
    };
    
    measureExecutionTime(testRandomTimes, 100, "Interpolate at random times");
    
    // 4. Test cache efficiency by repeating the same queries
    auto testCachedQueries = [&comms, this]() -> double {
        double result = 0;
        std::mt19937 gen(42);
        std::uniform_real_distribution<> dist(stateTimeline.front().timestamp, 
                                             stateTimeline.back().timestamp);
        
        // Generate a small set of times to query repeatedly
        std::vector<double> queryTimes;
        for (int i = 0; i < 10; i++) {
            queryTimes.push_back(dist(gen));
        }
        
        // Query each time multiple times to benefit from caching
        for (int rep = 0; rep < 10; rep++) {
            for (const auto& time : queryTimes) {
                SatelliteState interpolated = comms.interpolateState(time);
                // Force the compiler to use the result
                result += interpolated.position.x;
            }
        }
        return result;
    };
    
    measureExecutionTime(testCachedQueries, 10, "Cached queries");
    
    // Clear the cache between tests
    comms.clearCache();
}

TEST_F(SatelliteCommsPerformanceTest, TransmissionCheckPerformance) {
    // Create the satellite communication system
    SatelliteComms comms(PLANET_RADIUS, BEAM_ANGLE, stateTimeline);
    
    // Generate test points at various distances
    std::vector<Vector3> nearPoints = generateRandomPoints(50, ORBIT_RADIUS, ORBIT_RADIUS * 1.5);
    std::vector<Vector3> farPoints = generateRandomPoints(50, ORBIT_RADIUS * 1.5, ORBIT_RADIUS * 3.0);
    
    // Print header
    std::cout << "\n=== TRANSMISSION CHECK PERFORMANCE TEST ===\n";
    std::cout << std::setw(45) << "Test" << " | " << std::setw(8) << "Count" 
              << " | " << std::setw(10) << "Total Time" << " | " 
              << std::setw(11) << "Time/Call" << std::endl;
    std::cout << std::string(80, '-') << std::endl;
    
    // 1. Check transmission for nearby points
    auto testNearPoints = [&comms, &nearPoints]() -> int {
        int successCount = 0;
        double midTime = (comms.getTimelineStart() + comms.getTimelineEnd()) / 2.0;
        for (const auto& point : nearPoints) {
            if (comms.canReceiveTransmission(point, midTime)) {
                successCount++;
            }
        }
        return successCount;
    };
    
    measureExecutionTime(testNearPoints, 100, "Check transmission for nearby points");
    
    // 2. Check transmission for far points
    auto testFarPoints = [&comms, &farPoints]() -> int {
        int successCount = 0;
        double midTime = (comms.getTimelineStart() + comms.getTimelineEnd()) / 2.0;
        for (const auto& point : farPoints) {
            if (comms.canReceiveTransmission(point, midTime)) {
                successCount++;
            }
        }
        return successCount;
    };
    
    measureExecutionTime(testFarPoints, 100, "Check transmission for far points");
    
    // 3. Check transmission at multiple times for the same point
    auto testMultipleTimes = [&comms, this]() -> int {
        int successCount = 0;
        // Use a fixed point for consistency
        Vector3 testPoint = {ORBIT_RADIUS * 1.5, 0, 0};
        
        for (const auto& state : stateTimeline) {
            if (comms.canReceiveTransmission(testPoint, state.timestamp)) {
                successCount++;
            }
        }
        return successCount;
    };
    
    measureExecutionTime(testMultipleTimes, 10, "Check transmission at multiple times");
}

TEST_F(SatelliteCommsPerformanceTest, NextTransmissionTimePerformance) {
    // Create the satellite communication system
    SatelliteComms comms(PLANET_RADIUS, BEAM_ANGLE, stateTimeline);
    
    // Generate test points
    std::vector<Vector3> testPoints = generateRandomPoints(20, ORBIT_RADIUS, ORBIT_RADIUS * 2.0);
    
    // Print header
    std::cout << "\n=== NEXT TRANSMISSION TIME PERFORMANCE TEST ===\n";
    std::cout << std::setw(45) << "Test" << " | " << std::setw(8) << "Count" 
              << " | " << std::setw(10) << "Total Time" << " | " 
              << std::setw(11) << "Time/Call" << std::endl;
    std::cout << std::string(80, '-') << std::endl;
    
    // 1. Find next transmission time from the beginning of timeline
    auto testFromStart = [&comms, &testPoints, this]() -> int {
        int foundCount = 0;
        double startTime = stateTimeline.front().timestamp;
        
        for (const auto& point : testPoints) {
            auto result = comms.nextTransmissionTime(point, startTime);
            if (result.has_value()) {
                foundCount++;
            }
        }
        return foundCount;
    };
    
    measureExecutionTime(testFromStart, 5, "Find next transmission from start");
    
    // 2. Find next transmission time from the middle of timeline
    auto testFromMiddle = [&comms, &testPoints, this]() -> int {
        int foundCount = 0;
        double midTime = (stateTimeline.front().timestamp + stateTimeline.back().timestamp) / 2.0;
        
        for (const auto& point : testPoints) {
            auto result = comms.nextTransmissionTime(point, midTime);
            if (result.has_value()) {
                foundCount++;
            }
        }
        return foundCount;
    };
    
    measureExecutionTime(testFromMiddle, 5, "Find next transmission from middle");
    
    // 3. Test never-visible points (worst case scenario)
    auto testNeverVisible = [&comms, this]() -> int {
        int foundCount = 0;
        double startTime = stateTimeline.front().timestamp;
        
        // Create points that should never be visible
        std::vector<Vector3> invisiblePoints;
        for (int i = 0; i < 5; i++) {
            // Points perpendicular to orbital plane at extreme distance
            invisiblePoints.push_back({0, 0, ORBIT_RADIUS * 100.0});
        }
        
        for (const auto& point : invisiblePoints) {
            auto result = comms.nextTransmissionTime(point, startTime);
            if (result.has_value()) {
                foundCount++;
            }
        }
        return foundCount;
    };
    
    measureExecutionTime(testNeverVisible, 5, "Test never-visible points");
}

// Vector3 SIMD optimization benchmark
TEST_F(SatelliteCommsPerformanceTest, Vector3Operations) {
    // Generate a large number of random vectors for testing
    const int NUM_VECTORS = 10000;
    std::vector<Vector3> vectors;
    std::mt19937 gen(42);
    std::uniform_real_distribution<> dist(-1000.0, 1000.0);
    
    for (int i = 0; i < NUM_VECTORS; i++) {
        vectors.push_back({dist(gen), dist(gen), dist(gen)});
    }
    
    // Print header
    std::cout << "\n=== VECTOR3 OPERATIONS PERFORMANCE TEST ===\n";
    std::cout << std::setw(45) << "Test" << " | " << std::setw(8) << "Count" 
              << " | " << std::setw(10) << "Total Time" << " | " 
              << std::setw(11) << "Time/Call" << std::endl;
    std::cout << std::string(80, '-') << std::endl;
    
    // 1. Magnitude calculation
    auto testMagnitude = [&vectors]() -> double {
        double result = 0.0;
        for (const auto& v : vectors) {
            result += v.magnitude();
        }
        return result;
    };
    
    measureExecutionTime(testMagnitude, 100, "Vector magnitude calculation");
    
    // 2. Dot product calculation
    auto testDotProduct = [&vectors]() -> double {
        double result = 0.0;
        for (size_t i = 0; i < vectors.size() - 1; i++) {
            result += vectors[i].dot(vectors[i+1]);
        }
        return result;
    };
    
    measureExecutionTime(testDotProduct, 100, "Vector dot product calculation");
    
    // 3. Vector subtraction
    auto testSubtraction = [&vectors]() -> double {
        Vector3 result = {0.0, 0.0, 0.0};
        for (size_t i = 0; i < vectors.size() - 1; i++) {
            Vector3 diff = vectors[i] - vectors[i+1];
            result = {result.x + diff.x, result.y + diff.y, result.z + diff.z};
        }
        return result.magnitude();
    };
    
    measureExecutionTime(testSubtraction, 100, "Vector subtraction");
    
    // 4. Vector normalization
    auto testNormalization = [&vectors]() -> double {
        double result = 0.0;
        for (const auto& v : vectors) {
            try {
                Vector3 normalized = v.normalize();
                result += normalized.magnitude();
            } catch (const std::exception&) {
                // Skip any zero vectors
            }
        }
        return result;
    };
    
    measureExecutionTime(testNormalization, 100, "Vector normalization");
}

// Comparison between non-cached and cached implementations
TEST_F(SatelliteCommsPerformanceTest, CachingEfficiency) {
    // Set up a more complex orbit with more points for a better test
    create3DOrbitTimeline();
    SatelliteComms comms(PLANET_RADIUS, BEAM_ANGLE, stateTimeline);
    
    // Print header
    std::cout << "\n=== CACHING EFFICIENCY TEST ===\n";
    std::cout << std::setw(45) << "Test" << " | " << std::setw(8) << "Count" 
              << " | " << std::setw(10) << "Total Time" << " | " 
              << std::setw(11) << "Time/Call" << std::endl;
    std::cout << std::string(80, '-') << std::endl;
    
    // Generate a set of random times that we'll query multiple times
    std::vector<double> queryTimes;
    std::mt19937 gen(42);
    std::uniform_real_distribution<> dist(stateTimeline.front().timestamp, 
                                         stateTimeline.back().timestamp);
    
    for (int i = 0; i < 100; i++) {
        queryTimes.push_back(dist(gen));
    }
    
    // 1. First pass - no caching benefit
    auto testFirstPass = [&comms, &queryTimes]() -> double {
        double result = 0.0;
        for (const auto& time : queryTimes) {
            SatelliteState state = comms.interpolateState(time);
            result += state.position.magnitude();
        }
        return result;
    };
    
    measureExecutionTime(testFirstPass, 1, "First pass (no cache)");
    
    // 2. Second pass - should benefit from caching
    auto testSecondPass = [&comms, &queryTimes]() -> double {
        double result = 0.0;
        for (const auto& time : queryTimes) {
            SatelliteState state = comms.interpolateState(time);
            result += state.position.magnitude();
        }
        return result;
    };
    
    measureExecutionTime(testSecondPass, 1, "Second pass (with cache)");
    
    // 3. Repeated access - maximum cache benefit
    auto testRepeatedAccess = [&comms, &queryTimes]() -> double {
        double result = 0.0;
        // Just use the first 10 times but query them many times
        for (int i = 0; i < 50; i++) {
            for (int j = 0; j < 10; j++) {
                SatelliteState state = comms.interpolateState(queryTimes[j]);
                result += state.position.magnitude();
            }
        }
        return result;
    };
    
    measureExecutionTime(testRepeatedAccess, 1, "Repeated access (max cache benefit)");
    
    // Clear cache and measure performance difference
    comms.clearCache();
    
    // 4. After cache cleared
    auto testAfterClearCache = [&comms, &queryTimes]() -> double {
        double result = 0.0;
        for (const auto& time : queryTimes) {
            SatelliteState state = comms.interpolateState(time);
            result += state.position.magnitude();
        }
        return result;
    };
    
    measureExecutionTime(testAfterClearCache, 1, "After clearing cache");
}

#endif // ENABLE_PERFORMANCE_TESTS
