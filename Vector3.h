#pragma once

#include <cmath>
#include <stdexcept>

#if defined(__SSE3__) || defined(__SSSE3__)
#include <pmmintrin.h> // For SSE3 including _mm_hadd_pd
#elif defined(__SSE2__) 
#include <emmintrin.h> // For SSE2
#elif defined(__SSE__)
#include <xmmintrin.h> // For SSE
#endif

/**
 * 3D Vector class as specified in the requirements.
 * Extended with common vector operations for clarity and usability.
 * Optimized with optional SIMD support.
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
     * Computes the vector sum (this + other).
     */
    constexpr Vector3 operator+(const Vector3& other) const {
        return {x + other.x, y + other.y, z + other.z};
    }
    
    /**
     * Scales the vector by a scalar value.
     */
    constexpr Vector3 operator*(double scalar) const {
        return {x * scalar, y * scalar, z * scalar};
    }
    
    /**
     * Computes the magnitude (length) of the vector.
     * Uses SIMD operations when available.
     */
    double magnitude() const {
#if defined(__SSE3__) || defined(__SSSE3__)
        // Use SSE3 for faster calculation with horizontal add
        __m128d vec_xy = _mm_set_pd(y, x);
        __m128d vec_zz = _mm_set_pd(0, z);
        __m128d squared_xy = _mm_mul_pd(vec_xy, vec_xy);
        __m128d squared_zz = _mm_mul_pd(vec_zz, vec_zz);
        
        // Horizontal add for xý + yý
        __m128d sum_xy = _mm_hadd_pd(squared_xy, squared_xy);
        
        // Extract sum_xy[0] + zý
        double sum_xy_val = _mm_cvtsd_f64(sum_xy);
        double squared_z = _mm_cvtsd_f64(squared_zz);
        
        return std::sqrt(sum_xy_val + squared_z);
#elif defined(__SSE2__)
        // Use SSE2 without horizontal add
        __m128d vec_xy = _mm_set_pd(y, x);
        __m128d vec_z = _mm_set_sd(z);
        __m128d squared_xy = _mm_mul_pd(vec_xy, vec_xy);
        __m128d squared_z = _mm_mul_sd(vec_z, vec_z);
        
        // Extract and add manually without _mm_hadd_pd
        double x_squared = _mm_cvtsd_f64(squared_xy);
        squared_xy = _mm_shuffle_pd(squared_xy, squared_xy, 1);
        double y_squared = _mm_cvtsd_f64(squared_xy);
        double z_squared = _mm_cvtsd_f64(squared_z);
        
        return std::sqrt(x_squared + y_squared + z_squared);
#else
        // Fallback to standard calculation
        return std::sqrt(x*x + y*y + z*z);
#endif
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
     * Uses SIMD operations when available.
     */
    double dot(const Vector3& other) const {
#if defined(__SSE3__) || defined(__SSSE3__)
        // Use SSE3 for faster calculation with horizontal add
        __m128d vec1_xy = _mm_set_pd(y, x);
        __m128d vec2_xy = _mm_set_pd(other.y, other.x);
        __m128d vec1_zz = _mm_set_pd(0, z);
        __m128d vec2_zz = _mm_set_pd(0, other.z);
        
        // Multiply components
        __m128d mul_xy = _mm_mul_pd(vec1_xy, vec2_xy);
        __m128d mul_zz = _mm_mul_pd(vec1_zz, vec2_zz);
        
        // Sum x*other.x + y*other.y
        __m128d sum_xy = _mm_hadd_pd(mul_xy, mul_xy);
        
        // Extract and add z component
        double sum_xy_val = _mm_cvtsd_f64(sum_xy);
        double mul_z = _mm_cvtsd_f64(mul_zz);
        
        return sum_xy_val + mul_z;
#elif defined(__SSE2__)
        // Use SSE2 without horizontal add
        __m128d vec1_xy = _mm_set_pd(y, x);
        __m128d vec2_xy = _mm_set_pd(other.y, other.x);
        __m128d vec1_z = _mm_set_sd(z);
        __m128d vec2_z = _mm_set_sd(other.z);
        
        // Multiply components
        __m128d mul_xy = _mm_mul_pd(vec1_xy, vec2_xy);
        __m128d mul_z = _mm_mul_sd(vec1_z, vec2_z);
        
        // Extract and add manually
        double x_mul = _mm_cvtsd_f64(mul_xy);
        mul_xy = _mm_shuffle_pd(mul_xy, mul_xy, 1);
        double y_mul = _mm_cvtsd_f64(mul_xy);
        double z_mul = _mm_cvtsd_f64(mul_z);
        
        return x_mul + y_mul + z_mul;
#else
        // Fallback to standard calculation
        return x * other.x + y * other.y + z * other.z;
#endif
    }
    
    /**
     * Computes the squared distance to another vector.
     * Useful for comparisons without the expensive sqrt operation.
     */
    double distanceSquared(const Vector3& other) const {
        double dx = x - other.x;
        double dy = y - other.y;
        double dz = z - other.z;
        return dx*dx + dy*dy + dz*dz;
    }
    
    /**
     * Computes the distance to another vector.
     */
    double distance(const Vector3& other) const {
        return std::sqrt(distanceSquared(other));
    }
    
    /**
     * Computes the cross product with another vector.
     */
    Vector3 cross(const Vector3& other) const {
        return {
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        };
    }
};
