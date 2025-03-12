#pragma once

#include <vector>
#include <utility>
#include <optional>
#include <cmath>
#include <algorithm>
#include <stdexcept>

/**
 * 3D Vector class as specified in the requirements.
 * Extended with common vector operations for clarity and usability.
 */
struct Vector3 {
    double x, y, z;
    
    /**
     * Computes the vector difference (this - other).
     */
    Vector3 operator-(const Vector3& other) const {
        return {x - other.x, y - other.y, z - other.z};
    }
    
    /**
     * Computes the magnitude (length) of the vector.
     */
    double magnitude() const {
        return std::sqrt(x*x + y*y + z*z);
    }
    
    /**
     * Returns a normalized (unit length) version of this vector.
     */
    Vector3 normalize() const {
        double mag = magnitude();
        if (mag < 1e-10) {
            throw std::domain_error("Cannot normalize vector with zero magnitude");
        }
        return {x/mag, y/mag, z/mag};
    }
    
    /**
     * Computes the dot product with another vector.
     */
    double dot(const Vector3& other) const {
        return x * other.x + y * other.y + z * other.z;
    }
};

/**
 * Satellite state as specified in the requirements.
 */
struct SatelliteState {
    Vector3 position;
    Vector3 beamDirection;
};

/**
 * Time-stamped satellite state, used to track state changes over time.
 */
struct TimedSatelliteState {
    double timestamp;
    SatelliteState state;
};

/**
 * Satellite Communications system that determines when points in space
 * can receive transmissions from a satellite.
 */
class SatelliteComms {
public:
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
     * 
     * @param pointP The point in 3D space to check for transmission reception
     * @param startTime The time from which to start looking for the next transmission
     * @return The next time at which transmission can be received, or std::nullopt if none found
     */
    std::optional<double> nextTransmissionTime(const Vector3& pointP, double startTime) const;
    
    /**
     * Interpolates the satellite state at a given time between known timeline points.
     * 
     * @param time The time at which to interpolate the satellite state
     * @return The interpolated satellite state
     */
    SatelliteState interpolateState(double time) const;
    
private:
    // Configuration parameters
    double planetRadius_;
    double beamConeAngleRadians_; // Stored in radians for efficient calculations
    std::vector<TimedSatelliteState> stateTimeline_;
    
    /**
     * Checks if a point is within the satellite's beam cone.
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
};
