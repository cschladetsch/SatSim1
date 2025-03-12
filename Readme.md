# Satellite Communications API - Design Rationale and Details

## Overview

The API was designed to determine whether a point in 3D space can receive a transmission from a satellite at a given time, and if not, when the next available transmission time would be.

### Core Requirements

1. Determine if a point can receive a transmission based on:
   - Whether the point is within the satellite's beam cone
   - Whether the planet is blocking the transmission
2. Find the next time at which transmission is possible if currently not possible

## Key Design Decisions

### 1. Vector3 Class Extensions

I extended the provided `Vector3` struct with common vector operations:
- Subtraction (operator-)
- Magnitude calculation
- Normalization
- Dot product

These operations are foundational for the geometric calculations needed throughout the solution.

### 2. State Interpolation

A key challenge was accurately representing satellite state between discrete timeline samples. I implemented linear interpolation for position and direction vectors:

- Position: Standard linear interpolation between two points
- Direction: Normalized linear interpolation (a simplified approximation of SLERP)

This approach ensures smooth transitions between known states while maintaining unit-length beam direction vectors.

### 3. Beam Cone Determination

To check if a point is within the beam cone, I:
1. Calculate the vector from satellite to point
2. Normalize both this vector and the beam direction
3. Compute the dot product, which gives the cosine of the angle between them
4. Compare with cosine of the beam cone angle

This is more efficient than explicitly calculating angles using trigonometric functions.

### 4. Planet Occlusion Detection

To determine if the planet blocks transmission, I calculate:
1. The closest point on the satellite-to-point line to the planet center
2. Whether this closest point is between the satellite and the target point
3. Whether the distance from this point to planet center is less than the planet radius

This geometric approach handles all cases where the planet might block transmission.

### 5. Next Transmission Time Algorithm

For finding the next transmission time, I implemented:
1. An initial linear search with fixed t
