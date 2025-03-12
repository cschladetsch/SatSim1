// SatelliteComms.h
#pragma once

#include <vector>
#include <utility>
#include <optional>
#include <cmath>
#include <algorithm>
#include <stdexcept>

// 3D Vector class as specified
struct Vector3 {
    double x, y, z;
    
    // Vector operations
    Vector3 operator-(const Vector3& other) const {
        return {x - other.x, y - other.y, z - other.z};
    }
    
    double magnitude() const {
        return std::sqrt(x*x + y*y + z*z);
    }
    
    Vector3 normalize() const {
        double mag = magnitude();
        return {x/mag, y/mag, z/mag};
    }
    
    double dot(const Vector3& other) const {
        return x * other.x + y * other.y + z * other.z;
    }
};

// Satellite state as specified
struct SatelliteState {
    Vector3 position;
    Vector3 beamDirection;
};

// Time-stamped satellite state
struct TimedSatelliteState {
    double timestamp;
    SatelliteState state;
};

class SatelliteComms {
public:
    // Constructor
    SatelliteComms(
        double planetRadius,
        double beamConeAngleDegrees,
        const std::vector<TimedSatelliteState>& stateTimeline
    );
    
    // Question 1: Can a transmission be received at point P at given time?
    bool canReceiveTransmission(const Vector3& pointP, double time) const;
    
    // Question 2: When is the next time transmission can be received?
    std::optional<double> nextTransmissionTime(const Vector3& pointP, double startTime) const;
    
private:
    double planetRadius_;
    double beamConeAngleRadians_; // Stored in radians for calculations
    std::vector<TimedSatelliteState> stateTimeline_;
    
    // Helper to interpolate satellite state between timeline points
    SatelliteState interpolateState(double time) const;
    
    // Helper to check if point is within beam cone
    bool isInBeamCone(const Vector3& pointP, const SatelliteState& state) const;
    
    // Helper to check if planet blocks transmission
    bool isPlanetBlocking(const Vector3& pointP, const SatelliteState& state) const;
};
