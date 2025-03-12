#include "SatelliteComms.h"

SatelliteComms::SatelliteComms(
    double planetRadius,
    double beamConeAngleDegrees,
    const std::vector<TimedSatelliteState>& stateTimeline
) : 
    planetRadius_(planetRadius),
    beamConeAngleRadians_(beamConeAngleDegrees * M_PI / 180.0),
    stateTimeline_(stateTimeline)
{
    // Validate the timeline is chronologically ordered
    for (size_t i = 1; i < stateTimeline_.size(); ++i) {
        if (stateTimeline_[i].timestamp <= stateTimeline_[i-1].timestamp) {
            throw std::invalid_argument("Satellite state timeline must be chronologically ordered");
        }
    }
    
    // Validate we have at least two points for interpolation
    if (stateTimeline_.size() < 2) {
        throw std::invalid_argument("Satellite state timeline must contain at least two points");
    }
}

bool SatelliteComms::canReceiveTransmission(const Vector3& pointP, double time) const {
    // Check if time is within our timeline range
    if (time < stateTimeline_.front().timestamp || 
        time > stateTimeline_.back().timestamp) {
        return false;
    }
    
    // Get interpolated satellite state at the given time
    SatelliteState state = interpolateState(time);
    
    // Both conditions must be met for transmission to be received:
    // 1. Point must be within beam cone
    // 2. Planet must not block transmission
    return isInBeamCone(pointP, state) && !isPlanetBlocking(pointP, state);
}

std::optional<double> SatelliteComms::nextTransmissionTime(const Vector3& pointP, double startTime) const {
    // If we're already beyond our timeline, we can't predict future states
    if (startTime >= stateTimeline_.back().timestamp) {
        return std::nullopt;
    }
    
    // If transmission can be received now, return current time
    if (canReceiveTransmission(pointP, startTime)) {
        return startTime;
    }
    
    // Define search parameters
    const double timeStep = 1.0;       // Initial time step (1 second)
    const double precision = 1e-6;     // Precision threshold for binary search
    double currentTime = startTime + timeStep;
    
    // Linear search to find approximate next transmission time
    while (currentTime <= stateTimeline_.back().timestamp) {
        if (canReceiveTransmission(pointP, currentTime)) {
            // Found approximate time, now refine with binary search
            double lowerBound = currentTime - timeStep;
            double upperBound = currentTime;
            
            // Binary search to find the exact transition point
            while (upperBound - lowerBound > precision) {
                double midTime = (lowerBound + upperBound) / 2.0;
                if (canReceiveTransmission(pointP, midTime)) {
                    upperBound = midTime;  // Transmission possible at midTime
                } else {
                    lowerBound = midTime;  // Transmission not possible at midTime
                }
            }
            
            return upperBound;  // Return earliest time transmission is possible
        }
        
        currentTime += timeStep;
    }
    
    // No transmission time found within timeline
    return std::nullopt;
}

SatelliteState SatelliteComms::interpolateState(double time) const {
    // Find the timeline entry that is just after the requested time
    auto it = std::lower_bound(
        stateTimeline_.begin(), 
        stateTimeline_.end(), 
        time,
        [](const TimedSatelliteState& state, double t) {
            return state.timestamp < t;
        }
    );
    
    // Handle edge cases
    if (it == stateTimeline_.begin()) {
        return stateTimeline_.front().state;  // Time is before or at first sample
    }
    if (it == stateTimeline_.end()) {
        return stateTimeline_.back().state;   // Time is after last sample
    }
    
    // Get the timeline entries before and after the requested time
    auto after = it;
    auto before = it - 1;
    
    // If time exactly matches a sample, return that sample's state
    if (time == before->timestamp) {
        return before->state;
    }
    if (time == after->timestamp) {
        return after->state;
    }
    
    // Calculate interpolation factor (0.0 to 1.0)
    double t = (time - before->timestamp) / (after->timestamp - before->timestamp);
    
    // Linear interpolation for position
    Vector3 interpolatedPosition = {
        before->state.position.x + t * (after->state.position.x - before->state.position.x),
        before->state.position.y + t * (after->state.position.y - before->state.position.y),
        before->state.position.z + t * (after->state.position.z - before->state.position.z)
    };
    
    // Linear interpolation for direction vector, then normalize
    Vector3 lerpedDirection = {
        before->state.beamDirection.x + t * (after->state.beamDirection.x - before->state.beamDirection.x),
        before->state.beamDirection.y + t * (after->state.beamDirection.y - before->state.beamDirection.y),
        before->state.beamDirection.z + t * (after->state.beamDirection.z - before->state.beamDirection.z)
    };
    
    // Normalize the interpolated direction
    double dirMag = lerpedDirection.magnitude();
    Vector3 interpolatedDirection = {
        lerpedDirection.x / dirMag,
        lerpedDirection.y / dirMag,
        lerpedDirection.z / dirMag
    };
    
    return {interpolatedPosition, interpolatedDirection};
}

bool SatelliteComms::isInBeamCone(const Vector3& pointP, const SatelliteState& state) const {
    // Vector from satellite to point P
    Vector3 toPointVector = pointP - state.position;
    
    // Skip zero-distance check (would be a degenerate case)
    
    // Normalize vectors
    Vector3 normalizedBeamDir = state.beamDirection.normalize();
    Vector3 normalizedToPoint = toPointVector.normalize();
    
    // Dot product gives cosine of angle between vectors
    double cosAngle = normalizedBeamDir.dot(normalizedToPoint);
    
    // Inside cone if cosine of angle is greater than or equal to cosine of cone angle
    // (larger cosine means smaller angle)
    return cosAngle >= std::cos(beamConeAngleRadians_);
}

bool SatelliteComms::isPlanetBlocking(const Vector3& pointP, const SatelliteState& state) const {
    // Vector from satellite to point P
    Vector3 toPointVector = pointP - state.position;
    
    // Distance from satellite to point P
    double distToPoint = toPointVector.magnitude();
    
    // Direction from satellite to point (normalized)
    Vector3 dirToPoint = toPointVector.normalize();
    
    // Vector from origin (planet center) to satellite
    Vector3 originToSat = state.position;
    
    // Project originToSat onto the normalized direction from satellite to point
    // This gives the distance along the line where we find the closest approach to origin
    double projection = originToSat.dot(dirToPoint);
    
    // If projection is negative, closest point is "behind" the satellite (away from point P)
    // If projection > distance to point, closest point is "beyond" point P
    // In both cases, there's no intersection with planet between satellite and point P
    if (projection < 0 || projection > distToPoint) {
        return false;
    }
    
    // Calculate the closest point on the line to origin
    Vector3 closestPoint = {
        state.position.x - projection * dirToPoint.x,
        state.position.y - projection * dirToPoint.y,
        state.position.z - projection * dirToPoint.z
    };
    
    // Distance from closest point to origin
    double closestDist = closestPoint.magnitude();
    
    // Planet blocks if the closest approach is less than the planet radius
    return closestDist < planetRadius_;
}
