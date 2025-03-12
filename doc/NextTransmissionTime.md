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
