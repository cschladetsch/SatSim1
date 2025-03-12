#include <gtest/gtest.h>
#include "../SatelliteComms.h"
#include <cmath>

// Test fixture for SatelliteComms tests
class SatelliteCommsTest : public ::testing::Test {
protected:
    // Constants for test configuration
    static constexpr double ORBIT_RADIUS = 10000.0;
    static constexpr int NUM_SAMPLES = 10;
    static constexpr double TIME_STEP = 1000.0;
    static constexpr double TWO_PI = 2.0 * M_PI;
    
    // Test result validation constants
    static constexpr double TIME_LOWER_BOUND = 1600.0;
    static constexpr double TIME_UPPER_BOUND = 2400.0;
    
    // Default parameters
    double planetRadius = 6371.0; // Earth radius in km
    double beamConeAngle = 15.0;  // 15 degrees cone angle
    std::vector<TimedSatelliteState> stateTimeline;
    
    void SetUp() override {
        // Setup default timeline with satellite moving in a circular orbit
        // and beam pointing outward from the planet
        stateTimeline.clear();
        
        for (int i = 0; i < NUM_SAMPLES; i++) {
            double angle = (TWO_PI * i) / NUM_SAMPLES;
            double time = i * TIME_STEP;
            
            // Position in circular orbit
            Vector3 position = {
                ORBIT_RADIUS * cos(angle),
                ORBIT_RADIUS * sin(angle),
                0.0 // Orbit in XY plane for simplicity
            };
            
            // Beam direction points outward from planet center
            Vector3 beamDirection = position.normalize();
            
            stateTimeline.push_back({time, {position, beamDirection}});
        }
    }
};

// Test canReceiveTransmission when point is within beam cone and not blocked
TEST_F(SatelliteCommsTest, CanReceiveWhenInConeAndNotBlocked) {
    SatelliteComms comms(planetRadius, beamConeAngle, stateTimeline);
    
    // First position in timeline (t=0)
    // Position: (10000, 0, 0)
    // Beam direction: (1, 0, 0)
    
    // Point 1000 units further out, same direction
    constexpr double pointDistance = 11000.0;
    Vector3 pointP = {pointDistance, 0.0, 0.0};
    
    EXPECT_TRUE(comms.canReceiveTransmission(pointP, 0.0));
}

// Test canReceiveTransmission when point is outside beam cone
TEST_F(SatelliteCommsTest, CannotReceiveWhenOutsideCone) {
    SatelliteComms comms(planetRadius, beamConeAngle, stateTimeline);
    
    // First position in timeline (t=0)
    // Position: (10000, 0, 0)
    // Beam direction: (1, 0, 0)
    
    // Point off to the side, outside 15 degree cone
    constexpr double xDistance = 11000.0;
    constexpr double yDistance = 3000.0;
    Vector3 pointP = {xDistance, yDistance, 0.0};
    
    EXPECT_FALSE(comms.canReceiveTransmission(pointP, 0.0));
}

// Test canReceiveTransmission when planet is blocking
TEST_F(SatelliteCommsTest, CannotReceiveWhenPlanetBlocks) {
    SatelliteComms comms(planetRadius, beamConeAngle, stateTimeline);
    
    // First position in timeline (t=0)
    // Position: (10000, 0, 0)
    // Beam direction: (1, 0, 0)
    
    // Point in opposite direction, behind planet
    constexpr double behindPlanetDistance = -10000.0;
    Vector3 pointP = {behindPlanetDistance, 0.0, 0.0};
    
    EXPECT_FALSE(comms.canReceiveTransmission(pointP, 0.0));
}

// Test interpolation between timeline points
TEST_F(SatelliteCommsTest, InterpolatesBetweenTimelinePoints) {
    SatelliteComms comms(planetRadius, beamConeAngle, stateTimeline);
    
    // Between first and second timeline points
    constexpr double halfwayTime = 500.0; // Halfway between t=0 and t=1000
    
    // At t=0: Position (10000, 0, 0), Direction (1, 0, 0)
    // At t=1000: Position approx (9511, 3090, 0), Direction normalized of that
    
    // Point along the interpolated beam direction
    Vector3 pointP = {10947.0, 3557.0, 0.0};
    
    EXPECT_TRUE(comms.canReceiveTransmission(pointP, halfwayTime));
}

// Test nextTransmissionTime when transmission is possible immediately
TEST_F(SatelliteCommsTest, NextTransmissionTimeReturnsCurrentTimeWhenAvailable) {
    SatelliteComms comms(planetRadius, beamConeAngle, stateTimeline);
    
    // Point within beam at t=0
    constexpr double pointDistance = 11000.0;
    Vector3 pointP = {pointDistance, 0.0, 0.0};
    constexpr double startTime = 0.0;
    
    auto result = comms.nextTransmissionTime(pointP, startTime);
    EXPECT_TRUE(result.has_value());
    EXPECT_DOUBLE_EQ(startTime, result.value());
}

// Test nextTransmissionTime when transmission becomes available later
TEST_F(SatelliteCommsTest, FindsNextTransmissionTimeWhenAvailableLater) {
    SatelliteComms comms(planetRadius, beamConeAngle, stateTimeline);
    
    // Point that will enter beam later in orbit
    // This point is visible around t=2000 (when satellite is at 120 degrees)
    constexpr double xCoord = 5000.0;
    constexpr double yCoord = 8660.0; // Approximately 60 degrees from positive X axis
    Vector3 pointP = {xCoord, yCoord, 0.0};
    constexpr double startTime = 0.0;
    
    auto result = comms.nextTransmissionTime(pointP, startTime);
    EXPECT_TRUE(result.has_value());
    
    // Check that this time is around t=2000 (exact value depends on interpolation)
    // Adjust expected range based on actual algorithm behavior
    EXPECT_GE(result.value(), TIME_LOWER_BOUND);
    EXPECT_LE(result.value(), TIME_UPPER_BOUND);
    
    // Verify that transmission is actually possible at the returned time
    EXPECT_TRUE(comms.canReceiveTransmission(pointP, result.value()));
}

// Test nextTransmissionTime when no transmission is possible in timeline
TEST_F(SatelliteCommsTest, ReturnsNulloptWhenNoTransmissionPossible) {
    SatelliteComms comms(planetRadius, beamConeAngle, stateTimeline);
    
    // Point that will never be in any beam throughout the orbit
    // For example, high above the orbital plane
    constexpr double highZCoord = 20000.0;
    Vector3 pointP = {0.0, 0.0, highZCoord};
    constexpr double startTime = 0.0;
    
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
