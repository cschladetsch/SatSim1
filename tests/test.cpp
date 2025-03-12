#include <gtest/gtest.h>
#include "../SatelliteComms.h"
#include <cmath>

// Test fixture for SatelliteComms tests
class SatelliteCommsTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Setup default timeline with satellite moving in a circular orbit
        // and beam pointing outward from the planet
        stateTimeline.clear();
        
        // Simulating a circular orbit at 10,000 units
        double orbitRadius = 10000.0;
        int numSamples = 10;
        
        for (int i = 0; i < numSamples; i++) {
            double angle = (2 * M_PI * i) / numSamples;
            double time = i * 1000.0; // 1000 seconds between samples
            
            // Position in circular orbit
            Vector3 position = {
                orbitRadius * cos(angle),
                orbitRadius * sin(angle),
                0.0 // Orbit in XY plane for simplicity
            };
            
            // Beam direction points outward from planet center
            Vector3 beamDirection = {
                position.x,
                position.y,
                position.z
            };
            
            // Normalize beam direction
            double mag = sqrt(beamDirection.x*beamDirection.x + 
                             beamDirection.y*beamDirection.y + 
                             beamDirection.z*beamDirection.z);
            beamDirection.x /= mag;
            beamDirection.y /= mag;
            beamDirection.z /= mag;
            
            stateTimeline.push_back({time, {position, beamDirection}});
        }
        
        // Default parameters
        planetRadius = 6371.0; // Earth radius in km
        beamConeAngle = 15.0;  // 15 degrees cone angle
    }
    
    std::vector<TimedSatelliteState> stateTimeline;
    double planetRadius;
    double beamConeAngle;
};

// Test canReceiveTransmission when point is within beam cone and not blocked
TEST_F(SatelliteCommsTest, CanReceiveWhenInConeAndNotBlocked) {
    SatelliteComms comms(planetRadius, beamConeAngle, stateTimeline);
    
    // First position in timeline (t=0)
    // Position: (10000, 0, 0)
    // Beam direction: (1, 0, 0)
    
    // Point 1000 units further out, same direction
    Vector3 pointP = {11000.0, 0.0, 0.0};
    
    EXPECT_TRUE(comms.canReceiveTransmission(pointP, 0.0));
}

// Test canReceiveTransmission when point is outside beam cone
TEST_F(SatelliteCommsTest, CannotReceiveWhenOutsideCone) {
    SatelliteComms comms(planetRadius, beamConeAngle, stateTimeline);
    
    // First position in timeline (t=0)
    // Position: (10000, 0, 0)
    // Beam direction: (1, 0, 0)
    
    // Point off to the side, outside 15 degree cone
    Vector3 pointP = {11000.0, 3000.0, 0.0};
    
    EXPECT_FALSE(comms.canReceiveTransmission(pointP, 0.0));
}

// Test canReceiveTransmission when planet is blocking
TEST_F(SatelliteCommsTest, CannotReceiveWhenPlanetBlocks) {
    SatelliteComms comms(planetRadius, beamConeAngle, stateTimeline);
    
    // First position in timeline (t=0)
    // Position: (10000, 0, 0)
    // Beam direction: (1, 0, 0)
    
    // Point in opposite direction, behind planet
    Vector3 pointP = {-10000.0, 0.0, 0.0};
    
    EXPECT_FALSE(comms.canReceiveTransmission(pointP, 0.0));
}

// Test interpolation between timeline points
TEST_F(SatelliteCommsTest, InterpolatesBetweenTimelinePoints) {
    SatelliteComms comms(planetRadius, beamConeAngle, stateTimeline);
    
    // Between first and second timeline points
    double time = 500.0; // Halfway between t=0 and t=1000
    
    // At t=0: Position (10000, 0, 0), Direction (1, 0, 0)
    // At t=1000: Position approx (9511, 3090, 0), Direction normalized of that
    
    // Point along the interpolated beam direction
    // Based on interpolated position (9045, 2939, 0) and direction (0.951, 0.309, 0)
    Vector3 pointP = {10947.0, 3557.0, 0.0};
    
    EXPECT_TRUE(comms.canReceiveTransmission(pointP, time));
}

// Test nextTransmissionTime when transmission is possible immediately
TEST_F(SatelliteCommsTest, NextTransmissionTimeReturnsCurrentTimeWhenAvailable) {
    SatelliteComms comms(planetRadius, beamConeAngle, stateTimeline);
    
    // Point within beam at t=0
    Vector3 pointP = {11000.0, 0.0, 0.0};
    double startTime = 0.0;
    
    auto result = comms.nextTransmissionTime(pointP, startTime);
    EXPECT_TRUE(result.has_value());
    EXPECT_DOUBLE_EQ(startTime, result.value());
}

// Test nextTransmissionTime when transmission becomes available later
TEST_F(SatelliteCommsTest, FindsNextTransmissionTimeWhenAvailableLater) {
    SatelliteComms comms(planetRadius, beamConeAngle, stateTimeline);
    
    // Point that will enter beam later in orbit
    // This point is visible around t=2000 (when satellite is at 120 degrees)
    Vector3 pointP = {5000.0, 8660.0, 0.0}; // 60 degrees from positive X axis
    double startTime = 0.0;
    
    auto result = comms.nextTransmissionTime(pointP, startTime);
    EXPECT_TRUE(result.has_value());
    
    // Check that this time is around t=2000 (exact value depends on interpolation)
    // Adjust expected range based on actual algorithm behavior
    EXPECT_GE(result.value(), 1600.0);
    EXPECT_LE(result.value(), 2400.0);
    
    // Verify that transmission is actually possible at the returned time
    EXPECT_TRUE(comms.canReceiveTransmission(pointP, result.value()));
}

// Test nextTransmissionTime when no transmission is possible in timeline
TEST_F(SatelliteCommsTest, ReturnsNulloptWhenNoTransmissionPossible) {
    SatelliteComms comms(planetRadius, beamConeAngle, stateTimeline);
    
    // Point that will never be in any beam throughout the orbit
    // For example, high above the orbital plane
    Vector3 pointP = {0.0, 0.0, 20000.0};
    double startTime = 0.0;
    
    auto result = comms.nextTransmissionTime(pointP, startTime);
    EXPECT_FALSE(result.has_value());
}

// Test timeline validation - should throw if not chronologically ordered
TEST_F(SatelliteCommsTest, ThrowsWhenTimelineNotChronological) {
    std::vector<TimedSatelliteState> badTimeline = stateTimeline;
    // Swap timestamps to make non-chronological
    std::swap(badTimeline[1].timestamp, badTimeline[2].timestamp);
    
    EXPECT_THROW({
        SatelliteComms comms(planetRadius, beamConeAngle, badTimeline);
    }, std::invalid_argument);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
