#include "SatelliteComms.h"
#include "Quaternion.h"

SatelliteComms::SatelliteComms(
    double planetRadius,
    double beamConeAngleDegrees,
    const std::vector<TimedSatelliteState>& stateTimeline
) : 
    planetRadius_(planetRadius),
    beamConeAngleRadians_(beamConeAngleDegrees * DEG_TO_RAD),
    cosBeamConeAngle_(std::cos(beamConeAngleDegrees * DEG_TO_RAD)),
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
    
    // Reserve space for the cache to avoid rehashing
    stateCache_.reserve(MAX_CACHE_SIZE);
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
    // 1. The point must be within the satellite's beam cone.
    // 2. The planet must not block the transmission.
    return isInBeamCone(pointP, state) && !isPlanetBlocking(pointP, state);
}

std::optional<double> SatelliteComms::nextTransmissionTime(const Vector3& pointP, double startTime) const {
    // Constants for binary search refinement
    static constexpr double PRECISION = 1e-6;       // precision for binary search refinement
    static constexpr double MAX_STEP = 500.0;       // maximum time step to avoid too-large jumps
    static constexpr double MIN_STEP = 0.1;         // minimum time step to ensure progress

    // If startTime is beyond our timeline, no transmission is possible
    if (startTime >= stateTimeline_.back().timestamp) {
        return std::nullopt;
    }
    
    // Adjust startTime to be within the timeline if it's before the start
    double adjustedStartTime = std::max(startTime, stateTimeline_.front().timestamp);
    
    // If transmission is possible at adjusted startTime, return it immediately
    if (canReceiveTransmission(pointP, adjustedStartTime)) {
        return adjustedStartTime;
    }
    
    // Adaptive search for the first time transmission becomes possible
    double t_prev = adjustedStartTime;
    double t = t_prev + MIN_STEP;  // Start with a small step to ensure progress
    
    // Linear search phase - use fixed or adaptive steps depending on distance to target
    while (t <= stateTimeline_.back().timestamp) {
        if (canReceiveTransmission(pointP, t)) {
            break; // Found a time when transmission is possible
        }
        
        // Update search parameters
        t_prev = t;
        
        // Calculate next step size - either adaptive or linear
        double dt = std::min(MAX_STEP, (stateTimeline_.back().timestamp - t) / 20.0);
        
        // Ensure we make progress with at least the minimum step
        dt = std::max(dt, MIN_STEP);
        
        t = t_prev + dt;
        
        // Make sure we don't overshoot the timeline end
        if (t > stateTimeline_.back().timestamp) {
            // Make one final check at the last possible time
            if (canReceiveTransmission(pointP, stateTimeline_.back().timestamp)) {
                return stateTimeline_.back().timestamp;
            }
            return std::nullopt;
        }
    }
    
    // If we've reached the end without finding a transmission time
    if (t > stateTimeline_.back().timestamp) {
        return std::nullopt;
    }
    
    // If we've reached here, we found a transmission time at t
    // Binary search refinement between t_prev and t to find the exact transition point
    double lowerBound = t_prev;
    double upperBound = t;
    
    while (upperBound - lowerBound > PRECISION) {
        double midTime = (lowerBound + upperBound) / 2.0;
        if (canReceiveTransmission(pointP, midTime)) {
            upperBound = midTime;
        } else {
            lowerBound = midTime;
        }
    }
    
    return upperBound;
}

SatelliteState SatelliteComms::interpolateState(double time) const {
    // Check if the state is in the cache first
    auto cacheIt = stateCache_.find(time);
    if (cacheIt != stateCache_.end()) {
        // Check if the cache entry is still valid (not expired)
        auto now = std::chrono::steady_clock::now();
        if (now - cacheIt->second.timestamp < CACHE_TTL) {
            return cacheIt->second.state;
        }
        // If expired, remove it from the cache
        stateCache_.erase(cacheIt);
    }
    
    // Clean expired cache entries occasionally to prevent memory growth
    if (stateCache_.size() > MAX_CACHE_SIZE / 2) {
        cleanExpiredCacheEntries();
    }
    
    // Find the first timeline entry with a timestamp >= time.
    auto it = std::lower_bound(
        stateTimeline_.begin(), 
        stateTimeline_.end(), 
        time,
        [](const TimedSatelliteState& state, double t) {
            return state.timestamp < t;
        }
    );
    
    // Handle edge cases: before the first sample or after the last sample.
    if (it == stateTimeline_.begin()) {
        SatelliteState result = stateTimeline_.front().state;
        
        // Cache the result
        if (stateCache_.size() < MAX_CACHE_SIZE) {
            stateCache_[time] = {result, std::chrono::steady_clock::now()};
        }
        
        return result;
    }
    if (it == stateTimeline_.end()) {
        SatelliteState result = stateTimeline_.back().state;
        
        // Cache the result
        if (stateCache_.size() < MAX_CACHE_SIZE) {
            stateCache_[time] = {result, std::chrono::steady_clock::now()};
        }
        
        return result;
    }
    
    // Get the timeline entries before and after the requested time.
    auto after = it;
    auto before = it - 1;
    
    // If time exactly matches a sample, return that state.
    if (time == before->timestamp) {
        SatelliteState result = before->state;
        
        // Cache the result
        if (stateCache_.size() < MAX_CACHE_SIZE) {
            stateCache_[time] = {result, std::chrono::steady_clock::now()};
        }
        
        return result;
    }
    if (time == after->timestamp) {
        SatelliteState result = after->state;
        
        // Cache the result
        if (stateCache_.size() < MAX_CACHE_SIZE) {
            stateCache_[time] = {result, std::chrono::steady_clock::now()};
        }
        
        return result;
    }
    
    // Calculate interpolation factor (0.0 to 1.0)
    double t = (time - before->timestamp) / (after->timestamp - before->timestamp);
    
    // Linear interpolation for position
    Vector3 interpolatedPosition = {
        before->state.position.x + t * (after->state.position.x - before->state.position.x),
        before->state.position.y + t * (after->state.position.y - before->state.position.y),
        before->state.position.z + t * (after->state.position.z - before->state.position.z)
    };
    
    // Use quaternions for proper beam direction interpolation.
    // Create quaternions representing the rotation from a reference direction to the actual beam directions.
    static constexpr Vector3 REFERENCE_DIRECTION = {0.0, 0.0, 1.0};
    Quaternion q1 = Quaternion::fromVectors(REFERENCE_DIRECTION, before->state.beamDirection);
    Quaternion q2 = Quaternion::fromVectors(REFERENCE_DIRECTION, after->state.beamDirection);
    
    // Perform spherical linear interpolation (SLERP) between the two quaternions.
    Quaternion interpolatedQ = Quaternion::slerp(q1, q2, t);
    
    // Apply the interpolated rotation to the reference direction to get the interpolated beam direction.
    Vector3 interpolatedDirection = interpolatedQ.rotateVector(REFERENCE_DIRECTION);
    
    // Create the interpolated state
    SatelliteState result = {interpolatedPosition, interpolatedDirection};
    
    // Cache the result
    if (stateCache_.size() < MAX_CACHE_SIZE) {
        stateCache_[time] = {result, std::chrono::steady_clock::now()};
    }
    
    return result;
}

bool SatelliteComms::isInBeamCone(const Vector3& pointP, const SatelliteState& state) const {
    // Calculate the vector from the satellite to point P.
    Vector3 toPointVector = pointP - state.position;
    
    // Normalize the beam direction and the vector to the point.
    Vector3 normalizedBeamDir = state.beamDirection.normalize();
    Vector3 normalizedToPoint = toPointVector.normalize();
    
    // Compute the cosine of the angle between the beam and the point.
    double cosAngle = normalizedBeamDir.dot(normalizedToPoint);
    
    // The point is inside the beam cone if the cosine of the angle is greater than or equal
    // to the cosine of the beam cone's half-angle (which is pre-computed).
    return cosAngle >= cosBeamConeAngle_;
}

bool SatelliteComms::isPlanetBlocking(const Vector3& pointP, const SatelliteState& state) const {
    // Calculate the vector from the satellite to point P.
    Vector3 toPointVector = pointP - state.position;
    
    // Compute the distance from the satellite to point P.
    double distToPoint = toPointVector.magnitude();
    
    // Normalize the direction from the satellite to the point.
    Vector3 dirToPoint = toPointVector.normalize();
    
    // Vector from planet center (origin) to satellite.
    Vector3 originToSat = state.position;
    
    // Project originToSat onto the direction from the satellite to the point.
    double projection = originToSat.dot(dirToPoint);
    
    // If the projection is negative or greater than the distance to point P, the planet is not between.
    if (projection < 0 || projection > distToPoint) {
        return false;
    }
    
    // Compute the closest point on the line from satellite to point P to the origin.
    Vector3 closestPoint = {
        state.position.x - projection * dirToPoint.x,
        state.position.y - projection * dirToPoint.y,
        state.position.z - projection * dirToPoint.z
    };
    
    // The planet blocks transmission if the closest approach is less than the planet's radius.
    return closestPoint.magnitude() < planetRadius_;
}

void SatelliteComms::clearCache() const {
    stateCache_.clear();
}

void SatelliteComms::cleanExpiredCacheEntries() const {
    auto now = std::chrono::steady_clock::now();
    for (auto it = stateCache_.begin(); it != stateCache_.end();) {
        if (now - it->second.timestamp >= CACHE_TTL) {
            it = stateCache_.erase(it);
        } else {
            ++it;
        }
    }
}

double SatelliteComms::getAdaptiveStepSize(double currentTime, const Vector3& pointP) const {
    // Constants for adaptive stepping
    static constexpr double MIN_STEP = 0.1;         // minimum time step
    static constexpr double MAX_STEP = 500.0;       // maximum time step
    static constexpr double DEFAULT_STEP = 100.0;   // default step size
    
    // Get the current satellite state
    SatelliteState currentState;
    try {
        currentState = interpolateState(currentTime);
    } catch (const std::exception&) {
        // If interpolation fails, use a conservative default step
        return DEFAULT_STEP;
    }
    
    // Calculate the factor based on where we are in the timeline
    double timelineProgress = (currentTime - stateTimeline_.front().timestamp) / 
                              (stateTimeline_.back().timestamp - stateTimeline_.front().timestamp);
    
    // Calculate step based on the distance to the point
    // Further points can use larger steps
    double distToPoint = (pointP - currentState.position).magnitude();
    double orbitRadius = currentState.position.magnitude();
    
    // Normalize by orbit radius to get a relative distance
    double relativeDistance = distToPoint / orbitRadius;
    
    // Calculate angular velocity based on adjacent timeline points
    double angularVelocity = 0.1; // Default value
    
    // Find adjacent timeline points to calculate actual angular velocity
    auto it = std::lower_bound(
        stateTimeline_.begin(), 
        stateTimeline_.end(), 
        currentTime,
        [](const TimedSatelliteState& state, double t) {
            return state.timestamp < t;
        }
    );
    
    if (it != stateTimeline_.begin() && it != stateTimeline_.end()) {
        auto after = it;
        auto before = it - 1;
        
        // Calculate angular change between samples
        Vector3 positionBefore = before->state.position;
        Vector3 positionAfter = after->state.position;
        
        // Calculate unit vectors from origin to positions
        Vector3 dirBefore = positionBefore;
        Vector3 dirAfter = positionAfter;
        double magBefore = dirBefore.magnitude();
        double magAfter = dirAfter.magnitude();
        
        if (magBefore > 0 && magAfter > 0) {
            dirBefore = {
                dirBefore.x / magBefore,
                dirBefore.y / magBefore,
                dirBefore.z / magBefore
            };
            
            dirAfter = {
                dirAfter.x / magAfter,
                dirAfter.y / magAfter,
                dirAfter.z / magAfter
            };
            
            // Calculate angle between directions
            double cosAngle = dirBefore.dot(dirAfter);
            cosAngle = std::min(std::max(cosAngle, -1.0), 1.0); // Clamp to [-1, 1]
            double angle = std::acos(cosAngle);
            
            // Calculate time between samples
            double timeDelta = after->timestamp - before->timestamp;
            
            // Angular velocity in radians per second
            if (timeDelta > 0) {
                angularVelocity = angle / timeDelta;
            }
        }
    }
    
    // Calculate adaptive step size based on all factors
    double stepSize = DEFAULT_STEP;
    
    // Adjust for angular velocity - faster rotation requires smaller steps
    if (angularVelocity > 0) {
        // We want smaller steps for higher angular velocities
        double angularFactor = 0.1 / angularVelocity;
        stepSize *= std::min(std::max(angularFactor, 0.1), 10.0);
    }
    
    // Adjust for distance - larger distances can use larger steps
    stepSize *= std::min(std::max(relativeDistance, 0.5), 2.0);
    
    // Adjust for timeline progress - use smaller steps near the end of timeline
    if (timelineProgress > 0.9) {
        stepSize *= 0.5; // Reduce step size near the end
    }
    
    // Clamp to reasonable bounds
    return std::min(std::max(stepSize, MIN_STEP), MAX_STEP);
}
