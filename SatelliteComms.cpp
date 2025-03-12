#include "SatelliteComms.h"
#include "Quaternion.h"

SatelliteComms::SatelliteComms(
    double planetRadius,
    double beamConeAngleDegrees,
    const std::vector<TimedSatelliteState>& stateTimeline
) : 
    planetRadius_(planetRadius),
    beamConeAngleRadians_(beamConeAngleDegrees * DEG_TO_RAD),
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
    // Search algorithm constants
    static constexpr double TIME_STEP = 1.0;    // Initial time step (1 second)
    static constexpr double PRECISION = 1e-6;   // Precision threshold for binary search
    
    // If we're already beyond our timeline, we can't predict future states
    if (startTime >= stateTimeline_.back().timestamp) {
        return std::nullopt;
    }
    
    // If transmission can be received now, return current time
    if (canReceiveTransmission(pointP, startTime)) {
        return startTime;
    }
    
    // Step 1: Find the first time where transmission is possible
    double approximateTime = startTime + TIME_STEP;
    bool foundTransition = false;
    while (approximateTime <= stateTimeline_.back().timestamp) {
        if (canReceiveTransmission(pointP, approximateTime)) {
            foundTransition = true;
            break;
        }
        approximateTime += TIME_STEP;
    }
    
    if (!foundTransition) {
        return std::nullopt;
    }
    
    // Step 2: Expand the search window backward to find the earliest time transmission becomes possible
    double lowerBound = approximateTime;
    while (lowerBound - TIME_STEP >= stateTimeline_.front().timestamp &&
           canReceiveTransmission(pointP, lowerBound - TIME_STEP)) {
        lowerBound -= TIME_STEP;
    }
    
    // Step 3: Refine the transition time with binary search between lowerBound and approximateTime
    double refinedTime = approximateTime;
    while (refinedTime - lowerBound > PRECISION) {
        double midTime = (lowerBound + refinedTime) / 2.0;
        if (canReceiveTransmission(pointP, midTime)) {
            refinedTime = midTime;  // Transmission possible at midTime
        } else {
            lowerBound = midTime;  // Transmission not possible at midTime
        }
    }
    
    return refinedTime;  // Return the earliest time transmission is possible
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
    
    // Use quaternions for proper direction interpolation
    // Create quaternion that rotates from reference direction (e.g., {0,0,1}) to actual directions
    static constexpr Vector3 REFERENCE_DIRECTION = {0.0, 0.0, 1.0};
    
    Quaternion q1 = Quaternion::fromVectors(REFERENCE_DIRECTION, before->state.beamDirection);
    Quaternion q2 = Quaternion::fromVectors(REFERENCE_DIRECTION, after->state.beamDirection);
    
    // Perform spherical linear interpolation (SLERP) between the two quaternions
    Quaternion interpolatedQ = Quaternion::slerp(q1, q2, t);
    
    // Apply the interpolated rotation to the reference direction to get the interpolated beam direction
    Vector3 interpolatedDirection = interpolatedQ.rotateVector(REFERENCE_DIRECTION);
    
    return {interpolatedPosition, interpolatedDirection};
}

bool SatelliteComms::isInBeamCone(const Vector3& pointP, const SatelliteState& state) const {
    // Vector from satellite to point P
    Vector3 toPointVector = pointP - state.position;
    
    // Normalize vectors
    Vector3 normalizedBeamDir = state.beamDirection.normalize();
    Vector3 normalizedToPoint = toPointVector.normalize();
    
    // Dot product gives cosine of angle between vectors
    double cosAngle = normalizedBeamDir.dot(normalizedToPoint);
    
    // Inside cone if cosine of angle is greater than or equal to cosine of cone angle
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
    double projection = originToSat.dot(dirToPoint);
    
    // If projection is negative or beyond pointP, planet doesn't block
    if (projection < 0 || projection > distToPoint) {
        return false;
    }
    
    // Calculate the closest point on the line from satellite to pointP to the origin
    Vector3 closestPoint = {
        state.position.x - projection * dirToPoint.x,
        state.position.y - projection * dirToPoint.y,
        state.position.z - projection * dirToPoint.z
    };
    
    // Distance from the closest point to the origin
    double closestDist = closestPoint.magnitude();
    
    // Planet blocks if the closest approach is less than the planet radius
    return closestDist < planetRadius_;
}
