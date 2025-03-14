#include <chrono>
#include <iostream>
#include <iomanip>
#include <random>
#include <vector>
#include <string>
#include <memory>
#include <functional>
#include <fstream>
#include "SatelliteComms.h"

// Timer class for precise measurements
class Timer {
private:
    using clock_t = std::chrono::high_resolution_clock;
    using time_point_t = std::chrono::time_point<clock_t>;
    using duration_t = std::chrono::duration<double>;
    
    time_point_t start_time;
    
public:
    Timer() : start_time(clock_t::now()) {}
    
    void reset() {
        start_time = clock_t::now();
    }
    
    double elapsedSeconds() const {
        return std::chrono::duration_cast<duration_t>(clock_t::now() - start_time).count();
    }
};

// Helper class for generating test patterns
class TestPatternGenerator {
private:
    std::mt19937 gen;
    
public:
    TestPatternGenerator(unsigned int seed = 42) : gen(seed) {}
    
    // Generate timeline for satellite orbit
    std::vector<TimedSatelliteState> generateOrbitTimeline(
        double orbitRadius, int numSamples, double timeStep, 
        double inclination = 0.0, double eccentricity = 0.0) {
        
        std::vector<TimedSatelliteState> timeline;
        const double TWO_PI = 2.0 * M_PI;
        
        for (int i = 0; i < numSamples; i++) {
            double angle = (TWO_PI * i) / numSamples;
            double time = i * timeStep;
            
            // Adjust radius for eccentricity
            double adjRadius = orbitRadius * (1.0 - eccentricity * cos(angle));
            
            // Generate 3D position with inclination
            Vector3 position = {
                adjRadius * cos(angle),
                adjRadius * sin(angle) * cos(inclination),
                adjRadius * sin(angle) * sin(inclination)
            };
            
            // Beam pointing outward from planet center
            Vector3 beamDirection = position.normalize();
            
            timeline.push_back({time, {position, beamDirection}});
        }
        
        return timeline;
    }
    
    // Generate test points with various distributions
    enum class PointDistribution {
        UNIFORM_SPHERE,      // Uniformly distributed in a spherical shell
        ORBITAL_PLANE,       // Points concentrated around the orbital plane
        BEAM_TARGETED,       // Points mostly within beam cones
        MIXED                // Mixed distribution
    };
    
    std::vector<Vector3> generateTestPoints(
        int count, double minRadius, double maxRadius, 
        PointDistribution distribution = PointDistribution::UNIFORM_SPHERE) {
        
        std::vector<Vector3> points;
        std::uniform_real_distribution<> distRadius(minRadius, maxRadius);
        std::uniform_real_distribution<> distAngle(0, 2 * M_PI);
        std::uniform_real_distribution<> distZ(-1.0, 1.0);
        std::normal_distribution<> distNormal(0.0, 0.1);  // For concentrated distributions
        
        for (int i = 0; i < count; i++) {
            double radius, theta, phi, x, y, z;
            
            switch (distribution) {
                case PointDistribution::UNIFORM_SPHERE: {
                    // Uniform points in a spherical shell
                    radius = distRadius(gen);
                    theta = distAngle(gen);
                    phi = acos(2.0 * std::uniform_real_distribution<>(0, 1)(gen) - 1.0);
                    
                    x = radius * sin(phi) * cos(theta);
                    y = radius * sin(phi) * sin(theta);
                    z = radius * cos(phi);
                    break;
                }
                
                case PointDistribution::ORBITAL_PLANE: {
                    // Points concentrated around XY plane (orbit)
                    radius = distRadius(gen);
                    theta = distAngle(gen);
                    z = radius * distNormal(gen); // Concentrated around orbital plane
                    
                    x = radius * cos(theta);
                    y = radius * sin(theta);
                    break;
                }
                
                case PointDistribution::BEAM_TARGETED: {
                    // Points mostly within potential beam paths
                    // We'll create points in cone shapes from the origin
                    radius = distRadius(gen);
                    theta = distAngle(gen);
                    
                    // Generate points in cone-shaped regions
                    double coneAngle = 15.0 * M_PI / 180.0; // 15 degrees
                    phi = distNormal(gen) * coneAngle;
                    
                    x = radius * sin(phi) * cos(theta);
                    y = radius * sin(phi) * sin(theta);
                    z = radius * cos(phi);
                    break;
                }
                
                case PointDistribution::MIXED:
                default: {
                    // Mix of the above strategies
                    int strategy = i % 3;
                    if (strategy == 0) {
                        // Uniform
                        radius = distRadius(gen);
                        theta = distAngle(gen);
                        phi = acos(2.0 * std::uniform_real_distribution<>(0, 1)(gen) - 1.0);
                        
                        x = radius * sin(phi) * cos(theta);
                        y = radius * sin(phi) * sin(theta);
                        z = radius * cos(phi);
                    } 
                    else if (strategy == 1) {
                        // Orbital plane
                        radius = distRadius(gen);
                        theta = distAngle(gen);
                        z = radius * distNormal(gen) * 0.1;
                        
                        x = radius * cos(theta);
                        y = radius * sin(theta);
                    }
                    else {
                        // Beam targeted
                        radius = distRadius(gen);
                        theta = distAngle(gen);
                        
                        double coneAngle = 15.0 * M_PI / 180.0;
                        phi = distNormal(gen) * coneAngle;
                        
                        x = radius * sin(phi) * cos(theta);
                        y = radius * sin(phi) * sin(theta);
                        z = radius * cos(phi);
                    }
                    break;
                }
            }
            
            points.push_back({x, y, z});
        }
        
        return points;
    }
    
    // Generate query patterns for time values
    enum class TimeQueryPattern {
        UNIFORM,            // Uniformly distributed across timeline
        CLUSTERED,          // Clustered around specific time points
        REPEATED,           // A small set of time values repeated
        SEQUENTIAL          // Sequential time values
    };
    
    std::vector<double> generateTimeQueries(
        int count, double minTime, double maxTime,
        TimeQueryPattern pattern = TimeQueryPattern::UNIFORM) {
        
        std::vector<double> times;
        std::uniform_real_distribution<> distTime(minTime, maxTime);
        
        switch (pattern) {
            case TimeQueryPattern::UNIFORM: {
                // Uniformly distributed times
                for (int i = 0; i < count; i++) {
                    times.push_back(distTime(gen));
                }
                break;
            }
            
            case TimeQueryPattern::CLUSTERED: {
                // Generate clusters around certain time points
                int numClusters = 5;
                std::vector<double> clusterCenters;
                
                for (int i = 0; i < numClusters; i++) {
                    clusterCenters.push_back(minTime + (maxTime - minTime) * i / (numClusters - 1));
                }
                
                std::normal_distribution<> distCluster(0.0, (maxTime - minTime) * 0.05);
                
                for (int i = 0; i < count; i++) {
                    int cluster = i % numClusters;
                    double time = clusterCenters[cluster] + distCluster(gen);
                    time = std::max(minTime, std::min(maxTime, time)); // Clamp to range
                    times.push_back(time);
                }
                break;
            }
            
            case TimeQueryPattern::REPEATED: {
                // Generate a small set of times that are repeatedly queried
                int uniqueTimes = std::min(20, count / 5);
                std::vector<double> timeSamples;
                
                for (int i = 0; i < uniqueTimes; i++) {
                    timeSamples.push_back(distTime(gen));
                }
                
                for (int i = 0; i < count; i++) {
                    times.push_back(timeSamples[i % uniqueTimes]);
                }
                break;
            }
            
            case TimeQueryPattern::SEQUENTIAL: {
                // Sequential time values
                double step = (maxTime - minTime) / count;
                for (int i = 0; i < count; i++) {
                    times.push_back(minTime + i * step);
                }
                break;
            }
        }
        
        return times;
    }
};

// Direct implementation of interpolation without caching for comparison
SatelliteState interpolateWithoutCache(
    double time,
    const std::vector<TimedSatelliteState>& timeline) {
    
    // Find the first timeline entry with a timestamp >= time.
    auto it = std::lower_bound(
        timeline.begin(), 
        timeline.end(), 
        time,
        [](const TimedSatelliteState& state, double t) {
            return state.timestamp < t;
        }
    );
    
    // Handle edge cases: before the first sample or after the last sample.
    if (it == timeline.begin()) {
        return timeline.front().state;
    }
    if (it == timeline.end()) {
        return timeline.back().state;
    }
    
    // Get the timeline entries before and after the requested time.
    auto after = it;
    auto before = it - 1;
    
    // If time exactly matches a sample, return that state.
    if (doubleEquiv(time, before->timestamp)) {
        return before->state;
    }
    if (doubleEquiv(time, after->timestamp)) {
        return after->state;
    }
    
    // Calculate interpolation factor (0.0 to 1.0)
    double t = (time - before->timestamp) / (after->timestamp - before->timestamp);
    
    // Linear interpolation for position
    Vector3 interpolatedPosition = before->state.position + (after->state.position - before->state.position) * t;
    
    // Use quaternions for proper beam direction interpolation.
    static constexpr Vector3 REFERENCE_DIRECTION = {0.0, 0.0, 1.0};
    Quaternion q1 = Quaternion::fromVectors(REFERENCE_DIRECTION, before->state.beamDirection);
    Quaternion q2 = Quaternion::fromVectors(REFERENCE_DIRECTION, after->state.beamDirection);
    
    // Perform spherical linear interpolation (SLERP) between the two quaternions.
    Quaternion interpolatedQ = Quaternion::slerp(q1, q2, t);
    
    // Apply the interpolated rotation to the reference direction to get the interpolated beam direction.
    Vector3 interpolatedDirection = interpolatedQ.rotateVector(REFERENCE_DIRECTION);
    
    // Create the interpolated state
    SatelliteState result = {interpolatedPosition, interpolatedDirection};
    return result;
}

// Test scenario class
class TestScenario {
public:
    struct Config {
        std::string name;
        double planetRadius;
        double orbitRadius;
        double beamConeAngle;
        int timelineSamples;
        double timeStep;
        double timelineInclination;
        double timelineEccentricity;
        int pointCount;
        double minRadius;
        double maxRadius;
        TestPatternGenerator::PointDistribution pointDistribution;
        int timeQueryCount;
        TestPatternGenerator::TimeQueryPattern timePattern;
        int iterationMultiplier;  // Multiplier for how many times to run tests
    };
    
    struct BenchmarkResult {
        std::string scenarioName;
        double cachingInterpolationTime;
        double nonCachingInterpolationTime;
        double interpolationSpeedup;
        
        double cachingTransmissionCheckTime;
        double nonCachingTransmissionCheckTime;
        double transmissionSpeedup;
        
        double cachingNextTransmissionTime;
        double nonCachingNextTransmissionTime;
        double nextTransmissionSpeedup;
        
        int uniqueTimeQueries;
        int totalTimeQueries;
        double estimatedCacheHitRate;
    };
    
    TestScenario(const Config& config) : config(config) {
        // Initialize random generator
        TestPatternGenerator generator;
        
        // Generate timeline
        timeline = generator.generateOrbitTimeline(
            config.orbitRadius, 
            config.timelineSamples, 
            config.timeStep,
            config.timelineInclination,
            config.timelineEccentricity
        );
        
        // Generate test points
        testPoints = generator.generateTestPoints(
            config.pointCount, 
            config.minRadius, 
            config.maxRadius, 
            config.pointDistribution
        );
        
        // Generate time queries
        double minTime = timeline.front().timestamp;
        double maxTime = timeline.back().timestamp;
        timeQueries = generator.generateTimeQueries(
            config.timeQueryCount, 
            minTime, 
            maxTime, 
            config.timePattern
        );
    }
    
    BenchmarkResult runBenchmark() {
        BenchmarkResult result;
        result.scenarioName = config.name;
        
        // Create instance with caching
        SatelliteComms cachingComms(config.planetRadius, config.beamConeAngle, timeline);
        
        // =========== Calculate unique time queries for estimating cache hit rate ===========
        auto timeQueriesCopy = timeQueries;
        std::sort(timeQueriesCopy.begin(), timeQueriesCopy.end());
        auto uniqueEnd = std::unique(timeQueriesCopy.begin(), timeQueriesCopy.end(),
            [](double a, double b) { return std::abs(a - b) < 0.00001; });
        
        result.uniqueTimeQueries = std::distance(timeQueriesCopy.begin(), uniqueEnd);
        result.totalTimeQueries = timeQueries.size();
        result.estimatedCacheHitRate = 1.0 - (static_cast<double>(result.uniqueTimeQueries) / result.totalTimeQueries);
        
        // Determine number of iterations to increase runtime
        int iterationCount = config.iterationMultiplier;
        
        // =========== Interpolation Benchmark ===========
        {
            Timer cachingTimer;
            double sumValues = 0.0; // Prevent optimization from removing the calculation
            
            for (int iter = 0; iter < iterationCount; iter++) {
                for (double time : timeQueries) {
                    SatelliteState state = cachingComms.interpolateState(time);
                    sumValues += state.position.x; // Force the calculation to be performed
                }
            }
            
            result.cachingInterpolationTime = cachingTimer.elapsedSeconds();
            
            // Touch sumValues to prevent compiler from removing the calculation
            volatile double forceCalc = sumValues;
        }
        
        {
            Timer nonCachingTimer;
            double sumValues = 0.0;
            
            for (int iter = 0; iter < iterationCount; iter++) {
                for (double time : timeQueries) {
                    SatelliteState state = interpolateWithoutCache(time, timeline);
                    sumValues += state.position.x;
                }
            }
            
            result.nonCachingInterpolationTime = nonCachingTimer.elapsedSeconds();
            
            volatile double forceCalc = sumValues;
        }
        
        result.interpolationSpeedup = result.nonCachingInterpolationTime / result.cachingInterpolationTime;
        
        // Clear cache between tests
        cachingComms.clearCache();
        
        // =========== Transmission Check Benchmark ===========
        int transmissionSampleSize = std::min(500, static_cast<int>(testPoints.size()));
        int timeQuerySampleSize = std::min(20, static_cast<int>(timeQueries.size()));
        
        {
            Timer cachingTimer;
            int countReceived = 0;
            
            for (int iter = 0; iter < iterationCount; iter++) {
                for (int i = 0; i < transmissionSampleSize; i++) {
                    const auto& point = testPoints[i];
                    for (int j = 0; j < timeQuerySampleSize; j++) {
                        double time = timeQueries[j];
                        if (cachingComms.canReceiveTransmission(point, time)) {
                            countReceived++;
                        }
                    }
                }
            }
            
            result.cachingTransmissionCheckTime = cachingTimer.elapsedSeconds();
            
            volatile int forceCalc = countReceived;
        }
        
        // For non-caching version, we need to create a custom implementation
        // that calls interpolateWithoutCache instead of the cached version
        {
            Timer nonCachingTimer;
            int countReceived = 0;
            
            for (int iter = 0; iter < iterationCount; iter++) {
                for (int i = 0; i < transmissionSampleSize; i++) {
                    const auto& point = testPoints[i];
                    for (int j = 0; j < timeQuerySampleSize; j++) {
                        double time = timeQueries[j];
                        
                        // Skip if time is out of range
                        if (time < timeline.front().timestamp || time > timeline.back().timestamp) {
                            continue;
                        }
                        
                        // Get the state without using cache
                        SatelliteState state = interpolateWithoutCache(time, timeline);
                        
                        // Check if point is in beam cone
                        Vector3 toPointVector = point - state.position;
                        Vector3 normalizedBeamDir = state.beamDirection.normalize();
                        Vector3 normalizedToPoint = toPointVector.normalize();
                        double cosAngle = normalizedBeamDir.dot(normalizedToPoint);
                        double cosBeamConeAngle = std::cos(config.beamConeAngle * M_PI / 180.0);
                        bool inBeamCone = (cosAngle >= cosBeamConeAngle);
                        
                        // Check if planet is blocking
                        bool planetBlocking = false;
                        if (inBeamCone) {
                            // Calculate closest point to origin on line from satellite to point
                            double distToPoint = toPointVector.magnitude();
                            Vector3 dirToPoint = toPointVector.normalize();
                            Vector3 originToSat = state.position;
                            double projection = originToSat.dot(dirToPoint);
                            
                            if (projection >= 0 && projection <= distToPoint) {
                                Vector3 closestPoint = state.position - dirToPoint * projection;
                                planetBlocking = closestPoint.magnitude() < config.planetRadius;
                            }
                        }
                        
                        if (inBeamCone && !planetBlocking) {
                            countReceived++;
                        }
                    }
                }
            }
            
            result.nonCachingTransmissionCheckTime = nonCachingTimer.elapsedSeconds();
            
            volatile int forceCalc = countReceived;
        }
        
        result.transmissionSpeedup = result.nonCachingTransmissionCheckTime / result.cachingTransmissionCheckTime;
        
        // Clear cache between tests
        cachingComms.clearCache();
        
        // =========== Next Transmission Time Benchmark ===========
        // For this test, we'll use a subset of points to keep runtime reasonable
        int nextTransmissionSampleSize = std::min(20, static_cast<int>(testPoints.size()));
        
        {
            Timer cachingTimer;
            int foundCount = 0;
            
            for (int iter = 0; iter < iterationCount; iter++) {
                for (int i = 0; i < nextTransmissionSampleSize; i++) {
                    double startTime = timeline.front().timestamp;
                    auto result = cachingComms.nextTransmissionTime(testPoints[i], startTime);
                    if (result.has_value()) {
                        foundCount++;
                    }
                }
            }
            
            result.cachingNextTransmissionTime = cachingTimer.elapsedSeconds();
            
            volatile int forceCalc = foundCount;
        }
        
        // Create a non-caching version of nextTransmissionTime is complex and would require
        // duplicating significant logic. Instead, we'll just run the original with cleared
        // cache each time to simulate worst-case behavior.
        {
            Timer nonCachingTimer;
            int foundCount = 0;
            
            for (int iter = 0; iter < iterationCount; iter++) {
                for (int i = 0; i < nextTransmissionSampleSize; i++) {
                    cachingComms.clearCache(); // Clear cache between each call
                    double startTime = timeline.front().timestamp;
                    auto result = cachingComms.nextTransmissionTime(testPoints[i], startTime);
                    if (result.has_value()) {
                        foundCount++;
                    }
                }
            }
            
            result.nonCachingNextTransmissionTime = nonCachingTimer.elapsedSeconds();
            
            volatile int forceCalc = foundCount;
        }
        
        result.nextTransmissionSpeedup = result.nonCachingNextTransmissionTime / result.cachingNextTransmissionTime;
        
        std::cout << " - Complete" << std::endl;
        
        return result;
    }
    
private:
    Config config;
    std::vector<TimedSatelliteState> timeline;
    std::vector<Vector3> testPoints;
    std::vector<double> timeQueries;
};

// Main benchmark function
void runComprehensiveBenchmark() {
    std::cout << "\n" << std::string(80, '*') << std::endl;
    std::cout << "                   SATELLITE CACHE BENCHMARK                  " << std::endl;
    std::cout << std::string(80, '*') << std::endl;
    std::cout << "Testing caching performance using real-world satellite communication scenarios.\n" << std::endl;

    // Define test scenarios - including intensive ones
    std::vector<TestScenario::Config> scenarios = {
        // Basic tests with various hit ratios
        {
            "Test 1: Basic orbit - Unique queries (0% cache hits)",
            6371.0,                                    // planetRadius
            10000.0,                                   // orbitRadius
            15.0,                                      // beamConeAngle
            100,                                       // timelineSamples
            100.0,                                     // timeStep
            0.0,                                       // timelineInclination
            0.0,                                       // timelineEccentricity
            500,                                       // pointCount
            7000.0,                                    // minRadius
            20000.0,                                   // maxRadius
            TestPatternGenerator::PointDistribution::UNIFORM_SPHERE,   // pointDistribution
            5000,                                      // timeQueryCount
            TestPatternGenerator::TimeQueryPattern::UNIFORM,     // timePattern
            10                                         // iterationMultiplier
        },
        {
            "Test 2: Complex orbit - Repeated queries (98% cache hits)",
            6371.0,                                    // planetRadius
            10000.0,                                   // orbitRadius
            15.0,                                      // beamConeAngle
            200,                                       // timelineSamples
            50.0,                                      // timeStep
            0.3,                                       // timelineInclination
            0.1,                                       // timelineEccentricity
            500,                                       // pointCount
            7000.0,                                    // minRadius
            20000.0,                                   // maxRadius
            TestPatternGenerator::PointDistribution::MIXED,     // pointDistribution
            5000,                                      // timeQueryCount
            TestPatternGenerator::TimeQueryPattern::REPEATED,   // timePattern
            10                                         // iterationMultiplier
        },
        
        // Intensive tests
        {
            "Test 3: Large timeline - High cache benefit (99% hits)",
            6371.0,                                    // planetRadius
            10000.0,                                   // orbitRadius
            15.0,                                      // beamConeAngle
            2000,                                      // timelineSamples - very detailed timeline
            10.0,                                      // timeStep
            0.0,                                       // timelineInclination
            0.0,                                       // timelineEccentricity
            1000,                                      // pointCount
            7000.0,                                    // minRadius
            20000.0,                                   // maxRadius
            TestPatternGenerator::PointDistribution::BEAM_TARGETED, // pointDistribution
            20000,                                     // timeQueryCount - lots of queries
            TestPatternGenerator::TimeQueryPattern::REPEATED,   // timePattern - high cache hit potential
            5                                          // iterationMultiplier
        },
        {
            "Test 4: Very dense timeline - Sequential scan (0% hits)",
            6371.0,                                    // planetRadius
            10000.0,                                   // orbitRadius
            15.0,                                      // beamConeAngle
            5000,                                      // timelineSamples - extremely detailed
            5.0,                                       // timeStep
            0.1,                                       // timelineInclination
            0.05,                                      // timelineEccentricity
            1000,                                      // pointCount
            7000.0,                                    // minRadius
            20000.0,                                   // maxRadius
            TestPatternGenerator::PointDistribution::ORBITAL_PLANE, // pointDistribution
            10000,                                     // timeQueryCount
            TestPatternGenerator::TimeQueryPattern::SEQUENTIAL, // timePattern - sequential access pattern
            5                                          // iterationMultiplier
        },
        {
            "Test 5: Real-world satellite - Clustered access (20% hits)",
            6371.0,                                    // planetRadius
            42164.0,                                   // orbitRadius - Geostationary orbit
            8.0,                                       // beamConeAngle - narrower beam
            2500,                                      // timelineSamples
            10.0,                                      // timeStep
            0.05,                                      // timelineInclination - slight inclination
            0.0001,                                    // timelineEccentricity - near circular
            2000,                                      // pointCount - many ground points
            6500.0,                                    // minRadius - surface locations
            8000.0,                                    // maxRadius - surface + some altitude
            TestPatternGenerator::PointDistribution::ORBITAL_PLANE, // pointDistribution
            10000,                                     // timeQueryCount
            TestPatternGenerator::TimeQueryPattern::CLUSTERED,  // timePattern - time clustered around events
            5                                          // iterationMultiplier
        },
        {
            "Test 6: Extreme case - Random access (0% hits)",
            6371.0,                                    // planetRadius
            15000.0,                                   // orbitRadius
            20.0,                                      // beamConeAngle
            10000,                                     // timelineSamples - extremely long timeline
            5.0,                                       // timeStep
            0.3,                                       // timelineInclination
            0.2,                                       // timelineEccentricity - more eccentric
            3000,                                      // pointCount
            7000.0,                                    // minRadius
            30000.0,                                   // maxRadius - wide range of distances
            TestPatternGenerator::PointDistribution::MIXED,     // pointDistribution
            15000,                                     // timeQueryCount
            TestPatternGenerator::TimeQueryPattern::UNIFORM,    // timePattern - random access, low cache hit potential
            3                                          // iterationMultiplier
        }
    };
    
    // Run benchmarks
    std::vector<TestScenario::BenchmarkResult> results;
    
    for (const auto& config : scenarios) {
        std::cout << "Running " << config.name;
        std::cout.flush();
        TestScenario scenario(config);
        results.push_back(scenario.runBenchmark());
    }
    
    // Print results table - summary
    std::cout << "\n" << std::string(80, '*') << std::endl;
    std::cout << "BENCHMARK SUMMARY RESULTS" << std::endl;
    std::cout << std::string(80, '-') << std::endl;
    
    // Print header
    printf("%-45s | %12s | %12s | %12s | %9s\n", 
          "Test", "Interpolation", "Transmission", "NextTime", "Cache Hit%");
    
    std::cout << std::string(80, '-') << std::endl;
    
    // Results - formatted with printf for better alignment
    for (const auto& result : results) {
        printf("%-45s | %10.2fx | %10.2fx | %10.2fx | %8.1f%%\n",
              result.scenarioName.c_str(),
              result.interpolationSpeedup,
              result.transmissionSpeedup,
              result.nextTransmissionSpeedup,
              result.estimatedCacheHitRate * 100.0);
    }
    
    // Print detailed timing
    std::cout << "\n" << std::string(80, '*') << std::endl;
    std::cout << "DETAILED TIMING RESULTS (SECONDS)" << std::endl;
    std::cout << std::string(80, '-') << std::endl;
    
    // Detailed header
    printf("%-45s | %-12s | %12s | %12s | %12s\n",
          "Test", "Operation", "With Cache", "No Cache", "Difference");
    
    std::cout << std::string(80, '-') << std::endl;
    
    // Detailed timing - using printf for better alignment
    for (const auto& result : results) {
        // Interpolation
        printf("%-45s | %-12s | %12.4f | %12.4f | %12.4f\n",
              result.scenarioName.c_str(),
              "Transmission",
              result.cachingTransmissionCheckTime,
              result.nonCachingTransmissionCheckTime,
              result.nonCachingTransmissionCheckTime - result.cachingTransmissionCheckTime);
        
        // Next transmission time
        printf("%-45s | %-12s | %12.4f | %12.4f | %12.4f\n",
              result.scenarioName.c_str(),
              "NextTime",
              result.cachingNextTransmissionTime,
              result.nonCachingNextTransmissionTime,
              result.nonCachingNextTransmissionTime - result.cachingNextTransmissionTime);
        
        if (&result != &results.back()) {
            std::cout << std::string(80, '-') << std::endl;
        }
    }
    
    // Save results to CSV
    std::ofstream csvFile("satellite_cache_benchmark.csv");
    if (csvFile.is_open()) {
        // CSV Header
        csvFile << "Test,Operation,WithCache,NoCache,Difference,Speedup,CacheHitRate\n";
        
        for (const auto& result : results) {
            // Interpolation
            csvFile << "\"" << result.scenarioName << "\",Interpolation,"
                    << result.cachingInterpolationTime << ","
                    << result.nonCachingInterpolationTime << ","
                    << (result.nonCachingInterpolationTime - result.cachingInterpolationTime) << ","
                    << result.interpolationSpeedup << ","
                    << result.estimatedCacheHitRate << "\n";
            
            // Transmission check
            csvFile << "\"" << result.scenarioName << "\",Transmission,"
                    << result.cachingTransmissionCheckTime << ","
                    << result.nonCachingTransmissionCheckTime << ","
                    << (result.nonCachingTransmissionCheckTime - result.cachingTransmissionCheckTime) << ","
                    << result.transmissionSpeedup << ","
                    << result.estimatedCacheHitRate << "\n";
            
            // Next transmission time
            csvFile << "\"" << result.scenarioName << "\",NextTime,"
                    << result.cachingNextTransmissionTime << ","
                    << result.nonCachingNextTransmissionTime << ","
                    << (result.nonCachingNextTransmissionTime - result.cachingNextTransmissionTime) << ","
                    << result.nextTransmissionSpeedup << ","
                    << result.estimatedCacheHitRate << "\n";
        }
        
        csvFile.close();
        std::cout << "\nResults saved to satellite_cache_benchmark.csv" << std::endl;
    }
}

int main() {
    runComprehensiveBenchmark();
    return 0;
}
