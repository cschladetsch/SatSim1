# Design Decisions in the Satellite Communications API

## Vector3 Class Design

1. **Enhanced Vector Functionality**: 
   - **Decision**: Added methods for subtraction, magnitude, normalization, and dot product.
   - **Rationale**: These operations are fundamental to geometric calculations and spatial reasoning. Implementing them as class methods creates a more cohesive API, reduces code duplication, and makes calling code clearer. For example, `vector1.dot(vector2)` is more readable than a standalone function.

2. **Error Handling in Normalization**:
   - **Decision**: Added checks for division by zero in the `normalize()` method.
   - **Rationale**: Prevents silent failure in edge cases, making the code more robust. This follows the principle of "fail fast" – detecting errors early rather than propagating invalid states.

## Timeline and State Representation

1. **Chronologically Ordered Timeline**:
   - **Decision**: Required timeline entries to be strictly ordered by timestamp.
   - **Rationale**: Enables efficient binary search for state lookup. This requirement simplifies the interpolation logic and enables O(log n) lookup performance rather than O(n).

2. **Timeline Validation in Constructor**:
   - **Decision**: Validate timeline ordering at initialization time.
   - **Rationale**: Early detection of invalid inputs prevents subtle bugs later on. This defensive programming approach catches integration errors immediately.

3. **TimedSatelliteState Structure**:
   - **Decision**: Created a dedicated structure to pair timestamps with satellite states.
   - **Rationale**: Provides a clear, type-safe way to associate time with state data, improving code readability and reducing errors.

## Interpolation Strategy

1. **Linear Position Interpolation**:
   - **Decision**: Used simple linear interpolation for positions between samples.
   - **Rationale**: Provides a good balance between accuracy and computational efficiency. For most orbital paths, linear interpolation over small time steps is a reasonable approximation.

2. **Normalized Linear Direction Interpolation**:
   - **Decision**: Used normalized linear interpolation for beam directions rather than more complex SLERP.
   - **Rationale**: Simpler to implement and sufficient for most applications. While SLERP (Spherical Linear Interpolation) would be mathematically more correct for rotating directions, the normalized approach provides good results with lower computational cost.

3. **Edge Case Handling**:
   - **Decision**: Added special handling for times exactly matching timeline points.
   - **Rationale**: Avoids unnecessary interpolation calculations and potential floating-point precision issues.

## Beam Cone Detection

1. **Dot Product for Angle Comparison**:
   - **Decision**: Used dot product to determine if a point is within the beam cone.
   - **Rationale**: Avoids expensive trigonometric functions (acos, asin) by directly comparing cosines. Since cosine is monotonically decreasing for angles 0-180°, we can compare cosines directly rather than converting to angles.

2. **Pre-computation of Cone Angle**:
   - **Decision**: Convert beam cone angle to radians once during construction.
   - **Rationale**: Avoids repeated conversion during each visibility check, improving performance.

## Planet Occlusion Detection

1. **Line-Sphere Intersection Approach**:
   - **Decision**: Used geometric approach calculating closest point on line to planet center.
   - **Rationale**: This is an efficient way to determine if the line of sight between satellite and target intersects with the planet.

2. **Early-Exit Conditions**:
   - **Decision**: Added checks to exit early when closest approach is not between satellite and target.
   - **Rationale**: Reduces unnecessary calculations for cases where the planet is not between the satellite and target point.

## Next Transmission Time Algorithm

1. **Hybrid Search Strategy**:
   - **Decision**: Combined linear search with binary search refinement.
   - **Rationale**: This approach balances broad coverage (finding an approximate time when transmission becomes possible) with precision (refining to the exact transition point). Linear search alone would be inefficient for precision, while binary search alone would require knowledge of transition points that we don't have a priori.

2. **Configurable Time Step and Precision**:
   - **Decision**: Used constants for time step and precision threshold.
   - **Rationale**: Makes the algorithm adaptable to different scenarios through simple configuration changes. These parameters control the tradeoff between speed and accuracy.

3. **Returning Optional Type**:
   - **Decision**: Used `std::optional<double>` for the return value.
   - **Rationale**: Clearly communicates that a transmission time might not exist within our timeline. This is more type-safe and expressive than returning a sentinel value or throwing an exception.

## API Design

1. **Public vs. Private Methods**:
   - **Decision**: Made core functions public while keeping helper functions private.
   - **Rationale**: Presents a clean interface to users while hiding implementation details. The exception was `interpolateState()` which was made public for demonstration purposes in the example application.

2. **Const Correctness**:
   - **Decision**: Marked methods that don't modify state as `const`.
   - **Rationale**: Enables compiler optimization and documents that these methods won't modify the object's state. This also allows using the API with const objects and references.

3. **Comprehensive Documentation**:
   - **Decision**: Added detailed comments for all public methods.
   - **Rationale**: Makes the API self-explanatory and easier to use correctly. Good documentation reduces the learning curve and integration errors.

These design decisions were made to balance correctness, efficiency, readability, and maintainability – the key criteria mentioned in the problem statement. The resulting API provides a clear, efficient way to solve satellite transmission visibility problems while remaining flexible for different use cases.
