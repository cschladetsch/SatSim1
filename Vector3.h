#pragma once

#include <cmath>
#include <stdexcept>

/**
 * 3D Vector class as specified in the requirements.
 * Extended with common vector operations for clarity and usability.
 */
struct Vector3 {
    double x, y, z;
    
    /**
     * Computes the vector difference (this - other).
     */
    constexpr Vector3 operator-(const Vector3& other) const {
        return {x - other.x, y - other.y, z - other.z};
    }
    
    /**
     * Computes the magnitude (length) of the vector.
     */
    double magnitude() const {
        return std::sqrt(x*x + y*y + z*z);
    }
    
    /**
     * Returns a normalized (unit length) version of this vector.
     */
    Vector3 normalize() const {
        static constexpr double EPSILON = 1e-10;
        double mag = magnitude();
        if (mag < EPSILON) {
            throw std::domain_error("Cannot normalize vector with zero magnitude");
        }
        return {x/mag, y/mag, z/mag};
    }
    
    /**
     * Computes the dot product with another vector.
     */
    constexpr double dot(const Vector3& other) const {
        return x * other.x + y * other.y + z * other.z;
    }
};
