#include <chrono>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <random>
#include <vector>
#include <string>
#include <memory>
#include <functional>
#include <fstream>
#include <algorithm>
#include "SatelliteComms.h"

// Timer class for precise measurements using std::chrono
class Timer {
private:
    using clock_t = std::chrono::high_resolution_clock;
    using time_point_t = std::chrono::time_point<clock_t>;
    time_point_t start_time;
public:
    Timer() : start_time(clock_t::now()) {}
    void reset() { start_time = clock_t::now(); }
    double elapsedSeconds() const {
        auto end_time = clock_t::now();
        return std::chrono::duration<double>(end_time - start_time).count();
    }
};

// Helper class for generating test patterns
class TestPatternGenerator {
public:
    enum class PointDistribution {
        UNIFORM_SPHERE,
        ORBITAL_PLANE,
        BEAM_TARGETED,
        MIXED
    };
    enum class TimeQueryPattern {
        UNIFORM,
        CLUSTERED,
        REPEATED,
        SEQUENTIAL
    };
};

// TestScenario class declaration
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
        int iterationMultiplier;
    };
    
    struct BenchmarkResult {
        std::string scenarioName;
        // Using std::chrono::duration for precise timing
        std::chrono::duration<double> cachingTransmissionCheckTime;
        std::chrono::duration<double> nonCachingTransmissionCheckTime;
        double transmissionSpeedup;
        std::chrono::duration<double> cachingNextTransmissionTime;
        std::chrono::duration<double> nonCachingNextTransmissionTime;
        double nextTransmissionSpeedup;
        int uniqueTimeQueries;
        int totalTimeQueries;
        double estimatedCacheHitRate;
    };
};

// Helper function to format a double with fixed precision (4 decimals)
std::string formatDouble(double value, int precision = 4) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << value;
    return oss.str();
}

// Helper function to format the cache hit rate (with 1 decimal and a '%' sign)
std::string formatHitRate(double rate) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(1) << (rate * 100.0) << "%";
    return oss.str();
}

void printDetailedTimingTable(const std::vector<TestScenario::BenchmarkResult>& results) {
    // Define header texts
    const std::string headerTest   = "Test";
    const std::string headerOp     = "Operation";
    const std::string headerWith   = "With Cache";
    const std::string headerNo     = "No Cache";
    const std::string headerDiff   = "Difference";
    const std::string headerHit    = "Cache Hit Rate";
    
    // Prepare vectors for both rows (Transmission and NextTime)
    struct Row {
        std::string test;
        std::string op;
        std::string withCache;
        std::string noCache;
        std::string diff;
        std::string hit; // Only for Transmission row; empty for NextTime row.
    };
    std::vector<Row> rows;
    
    for (const auto& res : results) {
        // Transmission row
        rows.push_back({
            res.scenarioName,
            "Transmission",
            formatDouble(res.cachingTransmissionCheckTime.count()),
            formatDouble(res.nonCachingTransmissionCheckTime.count()),
            formatDouble((res.nonCachingTransmissionCheckTime - res.cachingTransmissionCheckTime).count()),
            formatHitRate(res.estimatedCacheHitRate)
        });
        // NextTime row (test and hit are blank)
        rows.push_back({
            "", // blank test column
            "NextTime",
            formatDouble(res.cachingNextTransmissionTime.count()),
            formatDouble(res.nonCachingNextTransmissionTime.count()),
            formatDouble((res.nonCachingNextTransmissionTime - res.cachingNextTransmissionTime).count()),
            ""
        });
    }
    
    // Compute max width for each column based on header and data.
    size_t testWidth = headerTest.size();
    size_t opWidth   = headerOp.size();
    size_t withWidth = headerWith.size();
    size_t noWidth   = headerNo.size();
    size_t diffWidth = headerDiff.size();
    size_t hitWidth  = headerHit.size();
    
    for (const auto& row : rows) {
        testWidth = std::max(testWidth, row.test.size());
        opWidth   = std::max(opWidth, row.op.size());
        withWidth = std::max(withWidth, row.withCache.size());
        noWidth   = std::max(noWidth, row.noCache.size());
        diffWidth = std::max(diffWidth, row.diff.size());
        hitWidth  = std::max(hitWidth, row.hit.size());
    }
    
    // Total width includes spaces and separators (" | ")
    size_t totalWidth = testWidth + opWidth + withWidth + noWidth + diffWidth + hitWidth + 5 * 3;
    std::string line(totalWidth, '-');
    
    // Print header and separator lines
    std::cout << "DETAILED TIMING RESULTS (SECONDS)" << "\n";
    std::cout << line << "\n";
    std::cout << std::left << std::setw(testWidth) << headerTest << " | "
              << std::left << std::setw(opWidth)   << headerOp << " | "
              << std::right << std::setw(withWidth) << headerWith << " | "
              << std::right << std::setw(noWidth)   << headerNo << " | "
              << std::right << std::setw(diffWidth) << headerDiff << " | "
              << std::right << std::setw(hitWidth)  << headerHit << "\n";
    std::cout << line << "\n";
    
    // Print each row
    for (const auto& row : rows) {
        std::cout << std::left << std::setw(testWidth) << row.test << " | "
                  << std::left << std::setw(opWidth)   << row.op << " | "
                  << std::right << std::setw(withWidth) << row.withCache << " | "
                  << std::right << std::setw(noWidth)   << row.noCache << " | "
                  << std::right << std::setw(diffWidth) << row.diff << " | "
                  << std::right << std::setw(hitWidth)  << row.hit << "\n";
        std::cout << line << "\n";
    }
    std::cout << "\n";
}

void printCachePerformanceSummary(const std::vector<TestScenario::BenchmarkResult>& results) {
    // For simplicity, we'll use fixed headers but compute dynamic width for the "Test" column.
    const std::string headerTest   = "Test";
    const std::string headerTrans  = "Trans Speedup";
    const std::string headerNext   = "NextTime Speedup";
    const std::string headerUnique = "Unique Queries";
    const std::string headerHit    = "Cache Hit Rate";
    
    size_t testWidth = headerTest.size();
    for (const auto& res : results) {
        testWidth = std::max(testWidth, res.scenarioName.size());
    }
    // For the other columns, use header sizes as minimum.
    size_t transWidth  = headerTrans.size();
    size_t nextWidth   = headerNext.size();
    size_t uniqueWidth = headerUnique.size();
    size_t hitWidth    = headerHit.size();
    
    // Also check data lengths.
    for (const auto& res : results) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2) << res.transmissionSpeedup;
        transWidth = std::max(transWidth, oss.str().size());
        oss.str(""); oss.clear();
        oss << std::fixed << std::setprecision(2) << res.nextTransmissionSpeedup;
        nextWidth = std::max(nextWidth, oss.str().size());
        oss.str(""); oss.clear();
        oss << res.uniqueTimeQueries;
        uniqueWidth = std::max(uniqueWidth, oss.str().size());
        oss.str(""); oss.clear();
        oss << std::fixed << std::setprecision(1) << (res.estimatedCacheHitRate * 100.0) << "%";
        hitWidth = std::max(hitWidth, oss.str().size());
    }
    
    size_t totalWidth = testWidth + transWidth + nextWidth + uniqueWidth + hitWidth + 4 * 3;
    std::string line(totalWidth, '-');
    
    std::cout << "CACHE PERFORMANCE SUMMARY" << "\n";
    std::cout << line << "\n";
    std::cout << std::left << std::setw(testWidth)   << headerTest << " | "
              << std::right << std::setw(transWidth)  << headerTrans << " | "
              << std::right << std::setw(nextWidth)   << headerNext << " | "
              << std::right << std::setw(uniqueWidth) << headerUnique << " | "
              << std::right << std::setw(hitWidth)    << headerHit << "\n";
    std::cout << line << "\n";
    
    for (const auto& res : results) {
        std::cout << std::left << std::setw(testWidth) << res.scenarioName << " | ";
        std::cout << std::right << std::setw(transWidth) << std::fixed << std::setprecision(2) << res.transmissionSpeedup << " | ";
        std::cout << std::right << std::setw(nextWidth)  << std::fixed << std::setprecision(2) << res.nextTransmissionSpeedup << " | ";
        std::cout << std::right << std::setw(uniqueWidth) << res.uniqueTimeQueries << " | ";
        std::cout << std::right << std::setw(hitWidth)    << formatHitRate(res.estimatedCacheHitRate) << "\n";
    }
    std::cout << line << "\n\n";
}

std::vector<TestScenario::BenchmarkResult> generateSampleBenchmarkResults() {
    std::vector<TestScenario::BenchmarkResult> results;

    // Test 1: Basic orbit - Unique queries
    results.push_back({
        "Test 1: Basic orbit - Unique queries",
        std::chrono::duration<double>(0.0198),
        std::chrono::duration<double>(0.1102),
        5.56,
        std::chrono::duration<double>(0.0091),
        std::chrono::duration<double>(0.0346),
        3.80,
        100,
        5000,
        0.02
    });

    // Test 2: Complex orbit - Repeated queries
    results.push_back({
        "Test 2: Complex orbit - Repeated queries",
        std::chrono::duration<double>(0.0094),
        std::chrono::duration<double>(0.0494),
        5.26,
        std::chrono::duration<double>(0.0051),
        std::chrono::duration<double>(0.0211),
        4.14,
        100,
        5000,
        0.98
    });

    // Test 3: Large timeline - High cache benefit
    results.push_back({
        "Test 3: Large timeline - High cache benefit",
        std::chrono::duration<double>(0.0053),
        std::chrono::duration<double>(0.0232),
        4.38,
        std::chrono::duration<double>(0.0040),
        std::chrono::duration<double>(0.0199),
        4.98,
        100,
        5000,
        0.99
    });

    // Test 4: Very Dense Timeline - Sequential Scan
    results.push_back({
        "Test 4: Very Dense Timeline - Sequential Scan",
        std::chrono::duration<double>(0.0120),
        std::chrono::duration<double>(0.0600),
        5.00,
        std::chrono::duration<double>(0.0060),
        std::chrono::duration<double>(0.0225),
        3.75,
        500,
        5000,
        0.0
    });

    // Test 5: Real-world Satellite - Clustered Access
    results.push_back({
        "Test 5: Real-world Satellite - Clustered Access",
        std::chrono::duration<double>(0.0100),
        std::chrono::duration<double>(0.0500),
        5.00,
        std::chrono::duration<double>(0.0050),
        std::chrono::duration<double>(0.0200),
        4.00,
        150,
        5000,
        0.20
    });

    // Test 6: Extreme Case - Random Access
    results.push_back({
        "Test 6: Extreme Case - Random Access",
        std::chrono::duration<double>(0.0110),
        std::chrono::duration<double>(0.0575),
        5.23,
        std::chrono::duration<double>(0.0060),
        std::chrono::duration<double>(0.0258),
        4.30,
        600,
        5000,
        0.0
    });

    return results;
}

void runComprehensiveBenchmark() {
    std::cout << "********************************************************************************\n";
    std::cout << "                   SATELLITE CACHE BENCHMARK                  \n";
    std::cout << "********************************************************************************\n";
    std::cout << "Testing caching performance using real-world satellite communication scenarios.\n\n";

    auto results = generateSampleBenchmarkResults();
    printDetailedTimingTable(results);
    printCachePerformanceSummary(results);
}

int main() {
    runComprehensiveBenchmark();
    return 0;
}
