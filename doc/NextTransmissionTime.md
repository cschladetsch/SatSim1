# Explanation of the `nextTransmissionTime` Method

The `nextTransmissionTime` method is a core part of the satellite communications API that answers the second question in the requirements: "If the transmission cannot be received at a given time, provide the next time at which it can be received."

Here's a detailed explanation of how this method works:

## Purpose

This method determines the earliest time, starting from a specified point in time, when a satellite's transmission can be received at a given 3D location.

## Method Signature

```cpp
std::optional<double> nextTransmissionTime(const Vector3& pointP, double startTime) const;
```

- **Parameters**:
  - `pointP`: The 3D coordinates where we want to receive the transmission
  - `startTime`: The time from which to start looking for the next transmission opportunity
  
- **Return Value**:
  - An `std::optional<double>` that contains the next transmission time when one exists
  - Returns `std::nullopt` if no transmission time is found within the available timeline

## Implementation Details

The implementation uses a hybrid approach combining linear search and binary search:

1. **Early Checks**:
   - If `startTime` is beyond our timeline end, return `std::nullopt` (we can't predict future states)
   - If transmission can be received at `startTime`, return that time immediately

2. **Linear Search Phase**:
   - Start with a time step of 1.0 second (configurable)
   - Incrementally check each time step until finding a time when transmission is possible
   - This provides an approximate time when reception becomes possible

3. **Binary Search Refinement**:
   - Once a viable time is found, use binary search between the last known non-receiving time and the first known receiving time
   - This efficiently narrows down to find the exact transition point where reception first becomes possible
   - The search continues until the precision threshold (1e-6 seconds) is reached

4. **Return Value**:
   - Returns the earliest time at which transmission can be received
   - If no such time is found within the timeline, returns `std::nullopt`

## How It Determines Reception

For each time checked, it calls `canReceiveTransmission()`, which verifies two conditions:
1. The point is within the satellite's beam cone
2. The planet is not blocking the transmission path

## Code Example from Implementation

```cpp
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
```

## Potential Optimizations and Considerations

1. **Adaptive Time Step**: For larger timelines, an adaptive time step that grows based on distance from current time could improve performance.

2. **Multiple Visibility Windows**: The current implementation finds the first visibility window. A more advanced version might find all visibility windows within a time range.

3. **Handling Timeline Gaps**: The implementation assumes the timeline covers all times of interest. In a production system, care should be taken with gaps in the timeline data.

4. **Precision vs. Performance**: The precision threshold (1e-6) balances accuracy against computational cost. This could be adjusted based on specific requirements.

This method efficiently solves the problem of finding when a point will next receive a satellite transmission, which is crucial for planning communication windows in satellite operations.
