#include <gtest/gtest.h>
#include "../SatelliteComms.h"
#include <cmath>
#include <limits>
#include <chrono>

// Extended test fixture with a different name to avoid collision
class SatelliteCommsAdvancedTest : public ::testing::Test {
protected:
    // Constants for test configuration
    static constexpr double ORBIT_RADIUS = 10000.0;
    static constexpr double PLANET_RADIUS = 6371.0;
    static constexpr double BEAM_ANGLE = 15.0;
    static constexpr int NUM_SAMPLES = 10;
    static constexpr double TIME_STEP = 1000.0;
    static constexpr double TWO_PI = 2.0 * M_PI;
    
    std::vector<TimedSatelliteState> stateTimeline;
    
    void SetUp() override {
        createCircularOrbitTimeline();
    }
    
    // Creates a circular orbit timeline in the XY plane
    void createCircularOrbitTimeline() {
        stateTimeline.clear();
        
        for (int i = 0; i < NUM_SAMPLES; i++) {
            double angle = (TWO_PI * i) / NUM_SAMPLES;
            double time = i * TIME_STEP;
            
            Vector3 position = {
                ORBIT_RADIUS * cos(angle),
                ORBIT_RADIUS * sin(angle),
                0.0
            };
            
            // Beam pointing outward from planet center
            Vector3 beamDirection = {
                position.x / ORBIT_RADIUS,
                position.y / ORBIT_RADIUS,
                0.0
            };
            
            stateTimeline.push_back({time, {position, beamDirection}});
        }
    }
    
    // Creates a polar orbit timeline (in XZ plane)
    void createPolarOrbitTimeline() {
        stateTimeline.clear();
        
        for (int i = 0; i < NUM_SAMPLES; i++) {
            double angle = (TWO_PI * i) / NUM_SAMPLES;
            double time = i * TIME_STEP;
            
            Vector3 position = {
                ORBIT_RADIUS * cos(angle),
                0.0,
                ORBIT_RADIUS * sin(angle)
            };
            
            // Beam pointing outward from planet center
            Vector3 beamDirection = {
                position.x / ORBIT_RADIUS,
                0.0,
                position.z / ORBIT_RADIUS
            };
            
            stateTimeline.push_back({time, {position, beamDirection}});
        }
    }
    
    // Creates a more complex timeline with varying time steps
    void createVariableTimeStepTimeline() {
        stateTimeline.clear();
        
        // Non-uniform time sampling
        std::vector<double> timePoints = {0.0, 100.0, 350.0, 800.0, 950.0, 1500.0, 2300.0, 3000.0, 4200.0, 5000.0};
        
        for (int i = 0; i < NUM_SAMPLES; i++) {
            double angle = (TWO_PI * i) / NUM_SAMPLES;
            double time = timePoints[i];
            
            Vector3 position = {
                ORBIT_RADIUS * cos(angle),
                ORBIT_RADIUS * sin(angle),
                0.0
            };
            
            // Beam pointing outward from planet center
            Vector3 beamDirection = {
                position.x / ORBIT_RADIUS,
                position.y / ORBIT_RADIUS,
                0.0
            };
            
            stateTimeline.push_back({time, {position, beamDirection}});
        }
    }
    
    // Creates a timeline with the beam not pointing outward
    void createOffsetBeamTimeline() {
        stateTimeline.clear();
        
        // Beam offset angle in radians
        constexpr double beamOffsetAngle = 30.0 * M_PI / 180.0;
        
        for (int i = 0; i < NUM_SAMPLES; i++) {
            double angle = (TWO_PI * i) / NUM_SAMPLES;
            double time = i * TIME_STEP;
            
            Vector3 position = {
                ORBIT_RADIUS * cos(angle),
                ORBIT_RADIUS * sin(angle),
                0.0
            };
            
            // Beam pointing with an offset from radial direction
            Vector3 beamDirection = {
                cos(angle + beamOffsetAngle),
                sin(angle + beamOffsetAngle),
                0.0
            };
            
            stateTimeline.push_back({time, {position, beamDirection}});
        }
    }
    
    // Calculate the angle between two vectors in degrees
    double angleBetweenVectors(const Vector3& v1, const Vector3& v2) {
        double dot = v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
        double mag1 = std::sqrt(v1.x * v1.x + v1.y * v1.y + v1.z * v1.z);
        double mag2 = std::sqrt(v2.x * v2.x + v2.y * v2.y + v2.z * v2.z);
        
        double cosAngle = dot / (mag1 * mag2);
        // Clamp to handle floating point errors
        cosAngle = std::min(std::max(cosAngle, -1.0), 1.0);
        
        return std::acos(cosAngle) * 180.0 / M_PI;
    }
};

// Test with varying beam cone angles
TEST_F(SatelliteCommsAdvancedTest, VaryingBeamConeAngles) {
    // At t=0, satellite is at (10000, 0, 0) with beam pointing along X-axis
    double time = 0.0;
    
    // Create a point we know is definitely in the standard 15-degree beam
    Vector3 inBeamPoint = {15000.0, 0.0, 0.0};  // Directly on beam axis
    
    // Should be inside 15ø, 20ø and 30ø cones
    SatelliteComms comms15(PLANET_RADIUS, 15.0, stateTimeline);
    SatelliteComms comms20(PLANET_RADIUS, 20.0, stateTimeline);
    SatelliteComms comms30(PLANET_RADIUS, 30.0, stateTimeline);
    
    EXPECT_TRUE(comms15.canReceiveTransmission(inBeamPoint, time));
    EXPECT_TRUE(comms20.canReceiveTransmission(inBeamPoint, time));
    EXPECT_TRUE(comms30.canReceiveTransmission(inBeamPoint, time));
    
    // Now test with a point that is outside the 15ø beam but still inside 20ø beam
    // We'll calculate the exact position for a point at approx 17ø from beam center
    double distanceFromSat = 5000.0;
    double angle17Deg = 17.0 * M_PI / 180.0;
    Vector3 point17Deg = {
        10000.0 + distanceFromSat * std::cos(angle17Deg),
        distanceFromSat * std::sin(angle17Deg),
        0.0
    };
    
    // Verify angle for debugging
    Vector3 beamDir = {1.0, 0.0, 0.0};
    Vector3 pointDir = {
        point17Deg.x - 10000.0,
        point17Deg.y,
        point17Deg.z
    };
    double actualAngle = angleBetweenVectors(beamDir, pointDir);
    std::cout << "Point angle from beam center: " << actualAngle << " degrees" << std::endl;
    
    // Point should be outside 15ø beam but inside 20ø beam
    EXPECT_FALSE(comms15.canReceiveTransmission(point17Deg, time));
    EXPECT_TRUE(comms20.canReceiveTransmission(point17Deg, time));
}

// Test edge case: requesting time exactly at timeline sample points
TEST_F(SatelliteCommsAdvancedTest, ExactTimelinePoints) {
    SatelliteComms comms(PLANET_RADIUS, BEAM_ANGLE, stateTimeline);
    
    // Test at each exact timeline point
    for (size_t i = 0; i < stateTimeline.size(); ++i) {
        double time = stateTimeline[i].timestamp;
        
        // Point directly in beam path for this time
        Vector3 pointP = {
            stateTimeline[i].state.position.x * 1.5, // 50% further out
            stateTimeline[i].state.position.y * 1.5,
            stateTimeline[i].state.position.z * 1.5
        };
        
        EXPECT_TRUE(comms.canReceiveTransmission(pointP, time))
            << "Failed at timeline point " << i << " (time = " << time << ")";
    }
}

// Test with a polar orbit (satellite orbiting over the poles)
TEST_F(SatelliteCommsAdvancedTest, PolarOrbit) {
    createPolarOrbitTimeline();
    SatelliteComms comms(PLANET_RADIUS, BEAM_ANGLE, stateTimeline);
    
    // Point in XZ plane (polar orbit plane)
    Vector3 pointP = {15000.0, 0.0, 0.0};
    EXPECT_TRUE(comms.canReceiveTransmission(pointP, 0.0));
    
    // Point off orbital plane
    Vector3 pointOffPlane = {15000.0, 3000.0, 0.0};
    EXPECT_FALSE(comms.canReceiveTransmission(pointOffPlane, 0.0));
    
    // Test a point that should be visible at North Pole position
    Vector3 northPoint = {0.0, 0.0, 15000.0};
    double northPoleTime = 2500.0; // Time when satellite is near north pole
    EXPECT_TRUE(comms.canReceiveTransmission(northPoint, northPoleTime));
}

// Test with variable time step timeline
TEST_F(SatelliteCommsAdvancedTest, VariableTimeSteps) {
    createVariableTimeStepTimeline();
    SatelliteComms comms(PLANET_RADIUS, BEAM_ANGLE, stateTimeline);
    
    // Test point at wide time gap
    double timeInWideGap = 3500.0; // Between 3000 and 4200
    
    // Find satellite position at this time through interpolation
    SatelliteState state = comms.interpolateState(timeInWideGap);
    
    // Create a point directly in the beam path at the interpolated position
    Vector3 pointP = {
        state.position.x + state.beamDirection.x * 5000.0,
        state.position.y + state.beamDirection.y * 5000.0,
        state.position.z + state.beamDirection.z * 5000.0
    };
    
    EXPECT_TRUE(comms.canReceiveTransmission(pointP, timeInWideGap));
    
    // Test nextTransmissionTime with variable time steps
    Vector3 futurePoint = {-15000.0, 0.0, 0.0}; // Will be visible much later
    double startTime = 100.0;
    
    auto nextTime = comms.nextTransmissionTime(futurePoint, startTime);
    if (nextTime.has_value()) {
        EXPECT_TRUE(comms.canReceiveTransmission(futurePoint, nextTime.value()));
    } else {
        // If no time found, check if it's truly never visible
        bool everVisible = false;
        for (double t = startTime; t <= stateTimeline.back().timestamp; t += 100.0) {
            if (comms.canReceiveTransmission(futurePoint, t)) {
                everVisible = true;
                break;
            }
        }
        EXPECT_FALSE(everVisible) << "Point should be visible but nextTransmissionTime returned nullopt";
    }
}

// Test with beam not pointing radially outward
TEST_F(SatelliteCommsAdvancedTest, OffsetBeamDirection) {
    createOffsetBeamTimeline();
    SatelliteComms comms(PLANET_RADIUS, BEAM_ANGLE, stateTimeline);
    
    // Point that would be in beam if pointing radially but isn't with offset
    double time = 0.0;
    Vector3 radialPoint = {15000.0, 0.0, 0.0};
    EXPECT_FALSE(comms.canReceiveTransmission(radialPoint, time));
    
    // Get the actual beam direction at t=0
    Vector3 beamDir = stateTimeline[0].state.beamDirection;
    Vector3 satPos = stateTimeline[0].state.position;
    
    // Create a point directly along the beam path
    Vector3 offsetPoint = {
        satPos.x + beamDir.x * 5000.0,
        satPos.y + beamDir.y * 5000.0,
        satPos.z + beamDir.z * 5000.0
    };
    
    EXPECT_TRUE(comms.canReceiveTransmission(offsetPoint, time));
}

// Test boundary conditions for planetary occlusion
TEST_F(SatelliteCommsAdvancedTest, PlanetaryOcclusionBoundary) {
    SatelliteComms comms(PLANET_RADIUS, BEAM_ANGLE, stateTimeline);
    
    // At t=0, satellite is at (10000, 0, 0)
    double time = 0.0;
    
    // Create points that are just barely occluded vs. just barely visible
    
    // Point definitely behind planet (should be blocked)
    Vector3 definitelyBlockedPoint = {-8000.0, 0.0, 0.0};
    EXPECT_FALSE(comms.canReceiveTransmission(definitelyBlockedPoint, time)) 
        << "Point should be occluded by planet";
    
    // Point definitely not behind planet (should be visible if in beam)
    Vector3 definitelyVisiblePoint = {-8000.0, 7000.0, 0.0};
    
    // Check if this point is in the beam
    Vector3 satPos = stateTimeline[0].state.position;
    Vector3 beamDir = stateTimeline[0].state.beamDirection;
    
    Vector3 toPoint = {
        definitelyVisiblePoint.x - satPos.x,
        definitelyVisiblePoint.y - satPos.y,
        definitelyVisiblePoint.z - satPos.z
    };
    
    double angle = angleBetweenVectors(beamDir, toPoint);
    
    if (angle <= BEAM_ANGLE) {
        EXPECT_TRUE(comms.canReceiveTransmission(definitelyVisiblePoint, time)) 
            << "Point should be visible (in beam and not occluded)";
    } else {
        std::cout << "Test point isn't in beam cone (angle = " << angle 
                  << " degrees), skipping visibility check" << std::endl;
    }
}

// Test nextTransmissionTime with a point that is never visible
TEST_F(SatelliteCommsAdvancedTest, NeverVisiblePoint) {
    SatelliteComms comms(PLANET_RADIUS, BEAM_ANGLE, stateTimeline);
    
    // Create a point that's far away in Z-direction (perpendicular to orbital plane)
    // and also behind planet - it should never be visible
    Vector3 neverVisiblePoint = {-5000.0, 0.0, 50000.0};
    double startTime = 0.0;
    
    auto nextTime = comms.nextTransmissionTime(neverVisiblePoint, startTime);
    EXPECT_FALSE(nextTime.has_value()) << "Point should never be visible";
}

// Performance test for nextTransmissionTime
TEST_F(SatelliteCommsAdvancedTest, PerformanceTest) {
    SatelliteComms comms(PLANET_RADIUS, BEAM_ANGLE, stateTimeline);
    
    // Create a point that will be visible but only after searching
    Vector3 eventuallyVisiblePoint = {0.0, -15000.0, 0.0};  // Visible at ~t=5000
    double startTime = 0.0;
    
    auto startClock = std::chrono::high_resolution_clock::now();
    
    auto nextTime = comms.nextTransmissionTime(eventuallyVisiblePoint, startTime);
    
    auto endClock = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endClock - startClock);
    
    std::cout << "nextTransmissionTime performance: " 
              << duration.count() << " microseconds" << std::endl;
    
    if (nextTime.has_value()) {
        EXPECT_TRUE(comms.canReceiveTransmission(eventuallyVisiblePoint, nextTime.value()));
    } else {
        // If no time found, verify it's truly never visible
        bool everVisible = false;
        for (double t = startTime; t <= stateTimeline.back().timestamp; t += 100.0) {
            if (comms.canReceiveTransmission(eventuallyVisiblePoint, t)) {
                everVisible = true;
                break;
            }
        }
        EXPECT_FALSE(everVisible) << "Point should be visible but nextTransmissionTime returned nullopt";
    }
}

// Test with extremely small beam angle - CORRECTED
TEST_F(SatelliteCommsAdvancedTest, VeryNarrowBeam) {
    constexpr double narrowBeamAngle = 1.0;  // 1 degree beam
    SatelliteComms comms(PLANET_RADIUS, narrowBeamAngle, stateTimeline);
    
    // At t=0, satellite is at (10000, 0, 0) with beam pointing along positive X
    
    // For a 1-degree beam, we need to be extremely precise with point placement
    // Let's verify with a simpler test of beam behavior
    
    // Test with standard beam first to confirm base case is working
    SatelliteComms standardComms(PLANET_RADIUS, BEAM_ANGLE, stateTimeline);
    Vector3 standardPoint = {15000.0, 0.0, 0.0};  // Directly on X-axis
    EXPECT_TRUE(standardComms.canReceiveTransmission(standardPoint, 0.0));
    
    // For narrow beam test, create a point that is definitely within 1 degree
    // When checking a 1ø beam, we need to be more careful with numerical issues
    // Let's use a point with extremely small offset to guarantee it's in beam
    Vector3 precisePoint = {15000.0, 0.1, 0.0};  // Practically on beam axis
    
    // We expect this point to be within beam
    EXPECT_TRUE(comms.canReceiveTransmission(precisePoint, 0.0));
    
    // Create a point that is definitely outside a 1ø beam 
    // Tangent of 1.5 degrees is ~0.026  at 5000 units distance, that's ~130 units offset
    Vector3 outsidePoint = {15000.0, 500.0, 0.0};  // Well outside 1-degree cone
    EXPECT_FALSE(comms.canReceiveTransmission(outsidePoint, 0.0));
}

// Test with extremely wide beam angle
TEST_F(SatelliteCommsAdvancedTest, VeryWideBeam) {
    // Let's use 90 degrees (hemisphere) instead of 175
    // This avoids ambiguity about what happens at angles close to 180 degrees
    constexpr double wideBeamAngle = 90.0;  // Hemisphere
    SatelliteComms comms(PLANET_RADIUS, wideBeamAngle, stateTimeline);
    
    // At t=0, satellite is at (10000, 0, 0)
    // Test with a point at a 45-degree angle from beam center
    double angle45 = 45.0 * M_PI / 180.0;
    Vector3 point45deg = {
        10000.0 + 5000.0 * std::cos(angle45),
        5000.0 * std::sin(angle45),
        0.0
    };
    
    // This point should be visible with a 90-degree beam
    EXPECT_TRUE(comms.canReceiveTransmission(point45deg, 0.0));
    
    // Point directly behind satellite should not be visible (blocked by planet)
    Vector3 directlyBehindPoint = {-10000.0, 0.0, 0.0};
    EXPECT_FALSE(comms.canReceiveTransmission(directlyBehindPoint, 0.0));
}

// Test constructing with invalid timeline (too few points)
TEST_F(SatelliteCommsAdvancedTest, TooFewTimelinePoints) {
    std::vector<TimedSatelliteState> shortTimeline;
    shortTimeline.push_back(stateTimeline[0]);  // Just one point
    
    EXPECT_THROW({
        SatelliteComms comms(PLANET_RADIUS, BEAM_ANGLE, shortTimeline);
    }, std::invalid_argument);
}

// Test with edge case times
TEST_F(SatelliteCommsAdvancedTest, EdgeCaseTimes) {
    SatelliteComms comms(PLANET_RADIUS, BEAM_ANGLE, stateTimeline);
    Vector3 pointP = {15000.0, 0.0, 0.0};
    
    // Time slightly before first sample
    EXPECT_FALSE(comms.canReceiveTransmission(pointP, -0.1));
    
    // Time exactly at first sample
    EXPECT_TRUE(comms.canReceiveTransmission(pointP, 0.0));
    
    // Time slightly after last sample
    double lastTime = stateTimeline.back().timestamp;
    EXPECT_FALSE(comms.canReceiveTransmission(pointP, lastTime + 0.1));
    
    // Time exactly at last sample
    Vector3 lastPoint = {
        stateTimeline.back().state.position.x * 1.5,
        stateTimeline.back().state.position.y * 1.5,
        stateTimeline.back().state.position.z * 1.5
    };
    EXPECT_TRUE(comms.canReceiveTransmission(lastPoint, lastTime));
}

