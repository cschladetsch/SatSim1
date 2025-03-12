#include "SatelliteComms.h"

// Constructor
SatelliteComms::SatelliteComms(
    double planetRadius,
    double beamConeAngleDegrees,
    const std::vector<TimedSatelliteState>& stateTimeline
) : 
    planetRadius_(planetRadius),
    beamConeAngleRadians_(beamConeAngleDegrees * M_PI / 180.0),
    stateTimeline_(stateTimeline)
{
    // Validate that timeline is chronologically ordered
    for (size_t i = 1; i < stateTimeline_.size(); ++i) {
        if (stateTimeline_[i].timestamp <= stateTimeline_[i-1].timestamp) {
            throw std::invalid_argument("Satellite state timeline must be chronologically ordered");
        }
    }
}

bool SatelliteComms::canReceiveTransmission(const Vector3& pointP, double time) const {
    // Check if time is within our timeline
    if (time < stateTimeline_.front().timestamp || 
        time > stateTimeline_.back().timestamp) {
        return false;
    }
    
    // Get interpolated satellite state at the given time
    SatelliteState state = interpolateState(time);
    
    // Check both conditions:
    // 1. Point must be within beam cone
    // 2. Planet must not block transmission
    return isInBeamCone(pointP, state) && !isPlanetBlocking(pointP, state);
}

std::optional<double> SatelliteComms::nextTransmissionTime(const Vector3& pointP, double startTime) const {
    // If transmission can be received now, return current time
    if (canReceiveTransmission(pointP, startTime)) {
        return startTime;
    }
    
    // If start time is beyond our timeline, we cannot predict
    if (startTime >= stateTimeline_.back().timestamp) {
        return std::nullopt;
    }
    
    // Binary search approach for efficiency
    // This is simplified and would need to be replaced with a more sophisticated algorithm
    // for a production implementation, as satellite visibility could change multiple times
    // between two consecutive samples
    
    // Start with a linear search for simplicity
    double timeStep = 1.0; // 1 second step, can be adjusted for precision
    double currentTime = startTime + timeStep;
    
    while (currentTime <= stateTimeline_.back().timestamp) {
        if (canReceiveTransmission(pointP, currentTime)) {
            // Refine the time by binary search between previous and current
            double lowerBound = currentTime - timeStep;
            double upperBound = currentTime;
            
            while (upperBound - lowerBound > 1e-6) { // Precision threshold
                double midTime = (lowerBound + upperBound) / 2.0;
                if (canReceiveTransmission(pointP, midTime)) {
                    upperBound = midTime;
                } else {
                    lowerBound = midTime;
                }
            }
            
            return upperBound;
        }
        
        currentTime += timeStep;
    }
    
    // If we get here, no transmission time was found
    return std::nullopt;
}

SatelliteState SatelliteComms::interpolateState(double time) const {
    // Find the two timeline points that surround the given time
    auto it = std::lower_bound(
        stateTimeline_.begin(), 
        stateTimeline_.end(), 
        time,
        [](const TimedSatelliteState& state, double t) {
            return state.timestamp < t;
        }
    );
    
    // If time is exactly on a timeline point
    if (it != stateTimeline_.end() && it->timestamp == time) {
        return it->state;
    }
    
    // If time is before the first point or after the last point
    if (it == stateTimeline_.begin()) {
        return stateTimeline_.front().state;
    }
    if (it == stateTimeline_.end()) {
        return stateTimeline_.back().state;
    }
    
    // Otherwise, interpolate between the two surrounding points
    auto after = it;
    auto before = it - 1;
    
    double t = (time - before->timestamp) / (after->timestamp - before->timestamp);
    
    // Linear interpolation for position
    Vector3 interpolatedPosition = {
        before->state.position.x + t * (after->state.position.x - before->state.position.x),
        before->state.position.y + t * (after->state.position.y - before->state.position.y),
        before->state.position.z + t * (after->state.position.z - before->state.position.z)
    };
    
    // SLERP (Spherical Linear Interpolation) would be ideal for direction vectors
    // but we'll use a simpler approach for brevity: normalize(lerp(v1, v2, t))
    Vector3 lerpedDirection = {
        before->state.beamDirection.x + t * (after->state.beamDirection.x - before->state.beamDirection.x),
        before->state.beamDirection.y + t * (after->state.beamDirection.y - before->state.beamDirection.y),
        before->state.beamDirection.z + t * (after->state.beamDirection.z - before->state.beamDirection.z)
    };
    Vector3 interpolatedDirection = {
        lerpedDirection.x / lerpedDirection.magnitude(),
        lerpedDirection.y / lerpedDirection.magnitude(),
        lerpedDirection.z / lerpedDirection.magnitude()
    };
    
    return {interpolatedPosition, interpolatedDirection};
}

bool SatelliteComms::isInBeamCone(const Vector3& pointP, const SatelliteState& state) const {
    // Vector from satellite to point P
    Vector3 toPointVector = {
        pointP.x - state.position.x,
        pointP.y - state.position.y,
        pointP.z - state.position.z
    };
    
    // Normalize both vectors
    Vector3 normalizedBeamDir = state.beamDirection.normalize();
    Vector3 normalizedToPoint = toPointVector.normalize();
    
    // Calculate dot product (cosine of angle between vectors)
    double cosAngle = normalizedBeamDir.dot(normalizedToPoint);
    
    // If cosine of angle is greater than cosine of cone angle, point is inside cone
    return cosAngle >= cos(beamConeAngleRadians_);
}

bool SatelliteComms::isPlanetBlocking(const Vector3& pointP, const SatelliteState& state) const {
    // Vector from satellite to point P
    Vector3 toPointVector = {
        pointP.x - state.position.x,
        pointP.y - state.position.y,
        pointP.z - state.position.z
    };
    
    // Distance from satellite to point P
    double distToPoint = toPointVector.magnitude();
    
    // Calculate closest distance from line (satellite to P) to planet center (origin)
    // We're assuming planet is at (0,0,0) as it wasn't specified otherwise
    
    // Vector from origin to satellite
    Vector3 originToSat = state.position;
    
    // Project originToSat onto the normalized direction from satellite to point
    Vector3 dirToPoint = toPointVector.normalize();
    double projection = originToSat.dot(dirToPoint);
    
    // Calculate closest point on the line to origin
    Vector3 closestPoint = {
        state.position.x - projection * dirToPoint.x,
        state.position.y - projection * dirToPoint.y,
        state.position.z - projection * dirToPoint.z
    };
    
    // Distance from closest point to origin
    double closestDist = closestPoint.magnitude();
    
    // If this distance is less than planet radius, the planet is blocking
    if (closestDist < planetRadius_) {
        // Additional check: is the closest point between satellite and point P?
        // If projection is negative, closest point is before satellite
        // If projection > distance to point, closest point is beyond point P
        if (projection > 0 && projection < distToPoint) {
            return true;
        }
    }
    
    return false;
}
