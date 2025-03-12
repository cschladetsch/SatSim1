#include <iostream>
#include <vector>
#include <iomanip>
#include <cmath>
#include <string>
#include <optional>
#include "SatelliteComms.h"

/**
 * Reports test case results with consistent formatting
 * 
 * @param caseNumber The test case number
 * @param caseDescription Description of the test case
 * @param pointP The test point location
 * @param time The test time
 * @param comms Reference to the SatelliteComms instance
 * @param showSatPosition Whether to show satellite position at next time (default: false)
 */
void reportTestCase(
    int caseNumber, 
    const std::string& caseDescription, 
    const Vector3& pointP, 
    double time, 
    const SatelliteComms& comms,
    bool showSatPosition = false
) {
    // Print header with case number and description
    std::cout << (caseNumber > 1 ? "\n" : "") << "Case " << caseNumber << ": " << caseDescription << std::endl;
    
    // Print test point and time
    std::cout << "  - Location: (" << pointP.x << ", " << pointP.y << ", " << pointP.z << ")" << std::endl;
    std::cout << "  - Time: " << time << std::endl;
    
    // Check if transmission is possible
    bool canReceive = comms.canReceiveTransmission(pointP, time);
    std::cout << "  - Can receive? " << (canReceive ? "Yes" : "No") << std::endl;
    
    // If can't receive at this time, check for next available time
    if (!canReceive) {
        auto nextTime = comms.nextTransmissionTime(pointP, time);
        if (nextTime) {
            std::cout << "  - Next available time: " << *nextTime << std::endl;
            
            // Verify transmission is possible at the returned time
            bool verifyReceive = comms.canReceiveTransmission(pointP, *nextTime);
            std::cout << "  - Verified reception at next time: " << (verifyReceive ? "Yes" : "No") << std::endl;
            
            // Optionally show satellite position at next time
            if (showSatPosition) {
                SatelliteState state = comms.interpolateState(*nextTime);
                std::cout << "  - Satellite position at next time: ("
                        << state.position.x << ", " 
                        << state.position.y << ", "
                        << state.position.z << ")" << std::endl;
            }
        } else {
            std::cout << "  - No transmission possible within timeline" << std::endl;
        }
    }
}

/**
 * Example application demonstrating the use of the SatelliteComms API.
 * This creates a satellite in circular orbit with a beam pointing outward,
 * then tests reception at various points in space.
 */
int main() {
    // Configuration parameters
    std::vector<TimedSatelliteState> stateTimeline;
    
    // Define orbital and beam parameters
    constexpr double orbitRadius = 10000.0;  // Units (e.g., km)
    constexpr double planetRadius = 6371.0;  // Earth radius in km
    constexpr double beamConeAngle = 15.0;   // 15 degrees cone angle
    
    // Create timeline with 10 samples over one full orbit
    constexpr int numSamples = 10;
    constexpr double timeStep = 1000.0;      // Each sample 1000 seconds apart
    constexpr double TWO_PI = 2.0 * M_PI;    // Full circle in radians
    
    for (int i = 0; i < numSamples; ++i) {
        double time = i * timeStep;
        double angle = (TWO_PI * i) / numSamples;
        
        // Calculate position in circular orbit (XY plane)
        Vector3 position = {
            orbitRadius * std::cos(angle),
            orbitRadius * std::sin(angle),
            0.0
        };
        
        // Beam direction pointing radially outward from planet center
        Vector3 beamDirection = position.normalize();
        
        // Add the state to our timeline
        stateTimeline.push_back({time, {position, beamDirection}});
    }
    
    // Initialize the satellite communications system
    SatelliteComms comms(planetRadius, beamConeAngle, stateTimeline);
    
    // Set output formatting for clarity
    std::cout << std::fixed << std::setprecision(2);
    
    // Test Case 1: Point directly in satellite's beam path
    {
        constexpr double pointDistance = 15000.0;  // Distance from origin for test points
        Vector3 pointP = {pointDistance, 0.0, 0.0};  // Further out along X-axis at t=0
        double time = 0.0;
        
        reportTestCase(1, "Point in beam path", pointP, time, comms);
    }
    
    // Test Case 2: Point outside beam cone
    {
        Vector3 pointP = {15000.0, 5000.0, 0.0};  // Off to the side, outside 15 degree cone
        double time = 0.0;
        
        reportTestCase(2, "Point outside beam cone", pointP, time, comms);
    }
    
    // Test Case 3: Point behind planet (occlusion test)
    {
        constexpr double behindPlanetDistance = -8000.0;  // Negative to place behind planet
        Vector3 pointP = {behindPlanetDistance, 0.0, 0.0};  // Behind planet relative to satellite at t=0
        double time = 0.0;
        
        reportTestCase(3, "Point behind planet", pointP, time, comms, true);
    }
    
    return 0;
}
