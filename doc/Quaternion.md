# Quaternions

## Rationale

Quaternions provide superior direction interpolation compared to simple vector methods because they preserve the geometry of rotations on a sphere. When linearly interpolating vectors and then normalizing, the path doesn't follow a great circle and produces non-uniform angular velocity. This can lead to the "candy wrapper effect" where the motion appears to accelerate and decelerate unnaturally. Quaternions, by contrast, represent rotations directly and allow for Spherical Linear Interpolation (SLERP), which maintains constant angular velocity along the shortest arc between directions. This mathematically correct approach is especially important for satellite beam direction modeling, where accurate representation of antenna orientation during orbital movement is essential for precise transmission predictions.

## Usage

The use of `{0,0,1}` as the reference direction in the quaternion creation is an important detail.

When we call `Quaternion::fromVectors({0,0,1}, before->state.beamDirection)`, we're creating a quaternion that represents the rotation from a reference direction `{0,0,1}` (the positive z-axis) to the actual beam direction.

Here's why we use `{0,0,1}` as the reference:

1. **Common Reference Frame**: By choosing a consistent reference direction for both quaternions, we ensure that the interpolation is performed in the same rotational context. This gives us a proper relative rotation between the two beam directions.

2. **Mathematical Convenience**: The positive z-axis `{0,0,1}` is often used as a standard reference direction in 3D mathematics because it aligns with the conventional "up" direction in many coordinate systems.

3. **Quaternion Operation**: What we're doing here is finding the rotation that would take the z-axis and align it with the beam direction. We do this for both the starting and ending beam directions, then interpolate between these rotations using SLERP.

4. **Consistency in Interpolation**: By using the same reference vector for both source and target quaternions, we're ensuring that the SLERP operation meaningfully represents the rotation path between the two beam directions.

The actual choice of `{0,0,1}` is somewhat arbitrary - we could have used `{1,0,0}` or `{0,1,0}` as well. What's important is:

1. The reference direction is the same for both quaternion conversions
2. The reference direction is not collinear with either of the beam directions (which would create a degenerate case)

At the end of the interpolation, we apply the interpolated quaternion to the same reference vector `{0,0,1}` to get the interpolated beam direction. This completes the rotational transformation from reference space to world space.

This approach ensures that the interpolation follows the shortest great circle arc between the two direction vectors, which is the path a physical satellite antenna would typically follow when rotating.
