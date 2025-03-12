// Quaternion.h
#pragma once

#include <cmath>
#include "Vector3.h"

/**
 * Quaternion class for representing 3D rotations.
 * Used for correct interpolation of direction vectors.
 */
class Quaternion {
public:
    double w, x, y, z;
    
    /**
     * Default constructor - creates identity quaternion
     */
    Quaternion() : w(1.0), x(0.0), y(0.0), z(0.0) {}
    
    /**
     * Construct quaternion from components
     */
    Quaternion(double w, double x, double y, double z) : w(w), x(x), y(y), z(z) {}
    
    /**
     * Create a quaternion that rotates from vector 'from' to vector 'to'
     */
    static Quaternion fromVectors(const Vector3& from, const Vector3& to) {
        // Normalize input vectors
        Vector3 fromNorm = from.normalize();
        Vector3 toNorm = to.normalize();
        
        // Get the dot product
        double dot = fromNorm.dot(toNorm);
        
        // If vectors are very close, return identity quaternion
        if (dot > 0.99999) {
            return Quaternion(1.0, 0.0, 0.0, 0.0);
        }
        
        // If vectors are opposite, create a 180-degree rotation around an arbitrary perpendicular axis
        if (dot < -0.99999) {
            // Find an axis perpendicular to fromNorm
            Vector3 axis = {1.0, 0.0, 0.0};
            if (std::abs(fromNorm.x) > 0.99999) {
                axis = {0.0, 1.0, 0.0};
            }
            
            // Create perpendicular vector
            Vector3 perpAxis;
            perpAxis.x = axis.y * fromNorm.z - axis.z * fromNorm.y;
            perpAxis.y = axis.z * fromNorm.x - axis.x * fromNorm.z;
            perpAxis.z = axis.x * fromNorm.y - axis.y * fromNorm.x;
            
            // Normalize the perpendicular axis
            double magPerp = std::sqrt(perpAxis.x*perpAxis.x + perpAxis.y*perpAxis.y + perpAxis.z*perpAxis.z);
            perpAxis.x /= magPerp;
            perpAxis.y /= magPerp;
            perpAxis.z /= magPerp;
            
            // Create 180-degree rotation quaternion around this axis
            return Quaternion(0.0, perpAxis.x, perpAxis.y, perpAxis.z);
        }
        
        // Calculate axis of rotation (cross product)
        Vector3 axis;
        axis.x = fromNorm.y * toNorm.z - fromNorm.z * toNorm.y;
        axis.y = fromNorm.z * toNorm.x - fromNorm.x * toNorm.z;
        axis.z = fromNorm.x * toNorm.y - fromNorm.y * toNorm.x;
        
        // Calculate magnitude of axis
        double axisMag = std::sqrt(axis.x*axis.x + axis.y*axis.y + axis.z*axis.z);
        
        // Normalize axis
        if (axisMag > 1e-10) {
            axis.x /= axisMag;
            axis.y /= axisMag;
            axis.z /= axisMag;
        }
        
        // Calculate angle and quaternion components
        double angle = std::acos(dot);
        double sinHalfAngle = std::sin(angle / 2.0);
        
        return Quaternion(
            std::cos(angle / 2.0),
            axis.x * sinHalfAngle,
            axis.y * sinHalfAngle,
            axis.z * sinHalfAngle
        );
    }
    
    /**
     * Normalize the quaternion to unit length
     */
    Quaternion normalize() const {
        double mag = std::sqrt(w*w + x*x + y*y + z*z);
        if (mag < 1e-10) {
            return Quaternion(1.0, 0.0, 0.0, 0.0);  // Return identity if magnitude is zero
        }
        return Quaternion(w/mag, x/mag, y/mag, z/mag);
    }
    
    /**
     * Spherical linear interpolation between two quaternions
     */
    static Quaternion slerp(const Quaternion& q1, const Quaternion& q2, double t) {
        // Normalize inputs
        Quaternion qa = q1.normalize();
        Quaternion qb = q2.normalize();
        
        // Calculate cosine of angle between quaternions
        double dot = qa.w * qb.w + qa.x * qb.x + qa.y * qb.y + qa.z * qb.z;
        
        // If dot is negative, negate one quaternion to take the shorter path
        if (dot < 0.0) {
            qb.w = -qb.w;
            qb.x = -qb.x;
            qb.y = -qb.y;
            qb.z = -qb.z;
            dot = -dot;
        }
        
        // Clamp dot to valid range
        dot = std::min(std::max(dot, -1.0), 1.0);
        
        // Set default interpolation parameters
        double theta = std::acos(dot);
        double sinTheta = std::sin(theta);
        
        // If angle is very small, use linear interpolation
        if (sinTheta < 1e-10) {
            return Quaternion(
                qa.w * (1.0 - t) + qb.w * t,
                qa.x * (1.0 - t) + qb.x * t,
                qa.y * (1.0 - t) + qb.y * t,
                qa.z * (1.0 - t) + qb.z * t
            ).normalize();
        }
        
        // Compute interpolation factors
        double scale0 = std::sin((1.0 - t) * theta) / sinTheta;
        double scale1 = std::sin(t * theta) / sinTheta;
        
        // Perform spherical interpolation
        return Quaternion(
            scale0 * qa.w + scale1 * qb.w,
            scale0 * qa.x + scale1 * qb.x,
            scale0 * qa.y + scale1 * qb.y,
            scale0 * qa.z + scale1 * qb.z
        );
    }
    
    /**
     * Rotate a vector by this quaternion
     */
    Vector3 rotateVector(const Vector3& v) const {
        // Perform quaternion multiplication: q * v * q^-1
        
        // Convert vector to pure quaternion
        Quaternion qv(0.0, v.x, v.y, v.z);
        
        // Calculate q * v
        Quaternion temp(
            -x * qv.x - y * qv.y - z * qv.z,
            w * qv.x + y * qv.z - z * qv.y,
            w * qv.y + z * qv.x - x * qv.z,
            w * qv.z + x * qv.y - y * qv.x
        );
        
        // Calculate (q * v) * q^-1
        // Note: For unit quaternions, q^-1 = (w, -x, -y, -z)
        Quaternion result(
            -temp.x * x - temp.y * y - temp.z * z,
            temp.x * w - temp.y * z + temp.z * y,
            temp.y * w - temp.z * x + temp.x * z,
            temp.z * w - temp.x * y + temp.y * x
        );
        
        return {result.x, result.y, result.z};
    }
};
