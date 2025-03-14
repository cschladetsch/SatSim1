#pragma once

#include <vector>
#include <utility>
#include <optional>
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <unordered_map>
#include <chrono>

#include "Quaternion.h"

inline bool doubleEquiv(double x, double y) {
    static constexpr double EPSILON = 1e-5;
    return (x - y) < EPSILON;
}

/**
 * Satellite state as specified in the requirements.
 */
struct SatelliteState {
    Vector3 position;
    Vector3 beamDirection;
    
    // Added equality operator for state caching
    bool operator==(const SatelliteState& other) const {
        return doubleEquiv(position.x, other.position.x) &&
               doubleEquiv(position.y, other.position.y) &&
               doubleEquiv(position.z, other.position.z) &&
               doubleEquiv(beamDirection.x, other.beamDirection.x) &&
               doubleEquiv(beamDirection.y, other.beamDirection.y) &&
               doubleEquiv(beamDirection.z, other.beamDirection.z);
    }
};

/**
 * Time-stamped satellite state, used to track state changes over time.
 */
struct TimedSatelliteState {
    double timestamp;
    SatelliteState state;
};

/**
 * Hash function for doubles to enable state caching
 */
struct DoubleHash {
    std::size_t operator()(const double& val) const {
        return std::hash<double>{}(val);
    }
};

/**
 * Hash function for SatelliteState to enable state caching
 */
struct StateHash {
    std::size_t operator()(const SatelliteState& state) const {
        std::size_t h1 = std::hash<double>{}(state.position.x);
        std::size_t h2 = std::hash<double>{}(state.position.y);
        std::size_t h3 = std::hash<double>{}(state.position.z);
        std::size_t h4 = std::hash<double>{}(state.beamDirection.x);
        std::size_t h5 = std::hash<double>{}(state.beamDirection.y);
        std::size_t h6 = std::hash<double>{}(state.beamDirection.z);
        
        // Combine hashes
        std::size_t result = h1;
        result = (result << 1) ^ h2;
        result = (result << 1) ^ h3;
        result = (result << 1) ^ h4;
        result = (result << 1) ^ h5;
        result = (result << 1) ^ h6;
        
        return result;
    }
};

/**
 * Cache entry with timestamp for expiration
 */
struct CacheEntry {
    SatelliteState state;
    std::chrono::steady_clock::time_point timestamp;
};

/**
 * Satellite Communications system that determines when points in space
 * can receive transmissions from a satellite.
 */
class SatelliteComms {
public:
    // Constants for algorithms
    static constexpr double DEG_TO_RAD = M_PI / 180.0;
    
    // Cache configuration
    static constexpr std::size_t MAX_CACHE_SIZE = 1000;
    static constexpr std::chrono::seconds CACHE_TTL{60}; // 60 seconds TTL
    
    /**
     * Constructs the satellite communications system.
     * 
     * @param planetRadius Radius of the planet (assumed at origin)
     * @param beamConeAngleDegrees Angular width of the satellite's transmission beam in degrees
     * @param stateTimeline Chronologically ordered sequence of satellite states over time
     * @throws std::invalid_argument if the timeline is not chronologically ordered
     */
    SatelliteComms(
        double planetRadius,
        double beamConeAngleDegrees,
        const std::vector<TimedSatelliteState>& stateTimeline
    );
    
    /**
     * Determines if a transmission from the satellite can be received at a given point and time.
     * 
     * @param pointP The point in 3D space to check for transmission reception
     * @param time The time at which to check for transmission reception
     * @return true if transmission can be received, false otherwise
     */
    bool canReceiveTransmission(const Vector3& pointP, double time) const;
    
    /**
     * Finds the next time at which a transmission can be received at a given point.
     * Uses adaptive time step for more efficient searching.
     * 
     * @param pointP The point in 3D space to check for transmission reception
     * @param startTime The time from which to start looking for the next transmission
     * @return The next time at which transmission can be received, or std::nullopt if none found
     */
    std::optional<double> nextTransmissionTime(const Vector3& pointP, double startTime) const;
    
    /**
     * Public access to interpolate state for the example app.
     * Uses state caching to improve performance for repeated queries.
     */
    SatelliteState interpolateState(double time) const;
    
    /**
     * Clear the state cache to free memory.
     * This is useful for long-running applications or when memory is constrained.
     */
    void clearCache() const;
    
    /**
     * Get the start time of the timeline.
     * 
     * @return The timestamp of the first entry in the timeline
     */
    double getTimelineStart() const {
        return stateTimeline_.front().timestamp;
    }
    
    /**
     * Get the end time of the timeline.
     * 
     * @return The timestamp of the last entry in the timeline
     */
    double getTimelineEnd() const {
        return stateTimeline_.back().timestamp;
    }
    
private:
    // Configuration parameters
    double planetRadius_;
    double beamConeAngleRadians_; // Stored in radians for efficient calculations
    double cosBeamConeAngle_;     // Pre-computed cosine of beam angle for faster checks
    std::vector<TimedSatelliteState> stateTimeline_;
    
    // Cache for interpolated states
    mutable std::unordered_map<double, CacheEntry, DoubleHash> stateCache_;
    
    /**
     * Helper method to add a state to the cache with proper management.
     * Ensures the cache doesn't exceed MAX_CACHE_SIZE by removing entries when needed.
     * 
     * @param time The simulation time for this state
     * @param state The satellite state to cache
     */
    void addToCache(double time, const SatelliteState& state) const;
    
    /**
     * Checks if a point is within the satellite's beam cone.
     * Optimized to use pre-computed cosine values.
     * 
     * @param pointP The point to check
     * @param state The satellite state (position and beam direction)
     * @return true if the point is within the beam cone, false otherwise
     */
    bool isInBeamCone(const Vector3& pointP, const SatelliteState& state) const;
    
    /**
     * Checks if the planet blocks the transmission path between the satellite and a point.
     * 
     * @param pointP The target point
     * @param state The satellite state (position and beam direction)
     * @return true if the planet blocks transmission, false otherwise
     */
    bool isPlanetBlocking(const Vector3& pointP, const SatelliteState& state) const;
    
    /**
     * Clean expired cache entries to prevent memory growth.
     * Called periodically during cache access.
     */
    void cleanExpiredCacheEntries() const;
    
    /**
     * Helper method to determine adaptive step size for next transmission search.
     * Takes into account the timeline duration and orbital characteristics.
     * 
     * @param currentTime The current time in the search
     * @param pointP The target point
     * @return An appropriate step size for the search
     */
    double getAdaptiveStepSize(double currentTime, const Vector3& pointP) const;
};
