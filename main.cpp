#include <iostream>
#include <vector>
#include <iomanip>
#include "SatelliteComms.h"

int main() {
    // Create a sample orbital path for the satellite
    std::vector<TimedSatelliteState> stateTimeline;
    
    // Simulate a satellite in circular orbit, with beam pointing outward
    double orbitRadius = 10000.0;  // Units (e.g., km)
    double planetRadius = 6371.0;  // Earth radius in km
    double beamConeAngle = 15.0;   // 15 degrees cone angle
    
    // Create timeline with 10 samples over one full orbit
    for (int i = 0; i < 10; ++i) {
        double time = i * 1000.0;            // Each sample 1000 seconds apart
        double angle = (2 * M_PI * i) / 10;  // Position in circular orbit
        
        // Calculate position
        Vector3 position = {
            orbitRadius * std::cos(angle),
            orbitRadius * std::sin(angle),
            0.0  // Orbit in XY plane for simplicity
        };
        
        // Beam direction pointing outward from planet
        Vector3 beamDirection = {
            position.x,
            position.y,
            position.z
        };
        
        // Normalize beam direction
        double mag = std::sqrt(beamDirection.x*beamDirection.x + 
                           beamDirection.y*beamDirection.y + 
                           beamDirection.z*beamDirection.z);
        beamDirection.x /= mag;
        beamDirection.y /= mag;
        beamDirection.z /= mag;
        
        stateTimeline.push_back({time, {position, beamDirection}});
    }
    
    // Initialize the satellite communications system
    SatelliteComms comms(planetRadius, beamConeAngle, stateTimeline);
    
    // Example 1: Point directly in front of the satellite at t=0
    Vector3 pointP1 = {15000.0, 0.0, 0.0};  // Further out along X-axis
    double time1 = 0.0;
    
    bool canReceive1 = comms.canReceiveTransmission(pointP1, time1);
    std::cout << "Example 1 - Point at (" << pointP1.x << ", " << pointP1.y << ", " << pointP1.z << ")" << std::endl;
    std::cout << "Can receive transmission at t=" << time1 << "? " << (canReceive1 ? "Yes" : "No") << std::endl;
    
    // Example 2: Point outside beam cone at t=0
    Vector3 pointP2 = {15000.0, 5000.0, 0.0};  // Off to the side, outside 15 degree cone
    double time2 = 0.0;
    
    bool canReceive2 = comms.canReceiveTransmission(pointP2, time2);
    std::cout << "\nExample 2 - Point at (" << pointP2.x << ", " << pointP2.y << ", " << pointP2.z << ")" << std::endl;
    std::cout << "Can receive transmission at t=" << time2 << "? " << (canReceive2 ? "Yes" : "No") << std::endl;
    
    // Find next available time for transmission
    auto nextTime2 = comms.nextTransmissionTime(pointP2, time2);
    if (nextTime2) {
        std::cout << "Next available transmission time: " << *nextTime2 << std::endl;
    } else {
        std::cout << "No transmission time available within timeline" << std::endl;
    }
    
    // Example 3: Point behind planet at t=0
    Vector3 pointP3 = {-8000.0, 0.0, 0.0};  // Behind planet relative to satellite
    double time3 = 0.0;
    
    bool canReceive3 = comms.canReceiveTransmission(pointP3, time3);
    std::cout << "\nExample 3 - Point at (" << pointP3.x << ", " << pointP3.y << ", " << pointP3.z << ")" << std::endl;
    std::cout << "Can receive transmission at t=" << time3 << "? " << (canReceive3 ? "Yes" : "No") << std::endl;
    
    // Find next available time for transmission
    auto nextTime3 = comms.nextTransmissionTime(pointP3, time3);
    if (nextTime3) {
        std::cout << "Next available transmission time: " << *nextTime3 << std::endl;
    } else {
        std::cout << "No transmission time available within timeline" << std::endl;
    }
    
    return 0;
}
