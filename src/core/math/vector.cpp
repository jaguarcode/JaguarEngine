/**
 * @file vector.cpp
 * @brief Vector math implementation
 */

#include "jaguar/core/types.h"
#include <cmath>
#include <algorithm>

namespace jaguar {

// ============================================================================
// Vec3 Member Function Implementations
// ============================================================================

Real Vec3::length() const noexcept {
    return std::sqrt(x*x + y*y + z*z);
}

Vec3 Vec3::normalized() const noexcept {
    Real len = length();
    if (len > 1e-10) {
        return {x / len, y / len, z / len};
    }
    return *this;
}

// ============================================================================
// Free Function Utilities
// ============================================================================

namespace math {

Real length(const Vec3& v) {
    return v.length();
}

Vec3 normalize(const Vec3& v) {
    return v.normalized();
}

Real dot(const Vec3& a, const Vec3& b) {
    return a.dot(b);
}

Vec3 cross(const Vec3& a, const Vec3& b) {
    return a.cross(b);
}

Real distance(const Vec3& a, const Vec3& b) {
    return (a - b).length();
}

Real distance_squared(const Vec3& a, const Vec3& b) {
    return (a - b).length_squared();
}

Vec3 lerp(const Vec3& a, const Vec3& b, Real t) {
    return a + (b - a) * t;
}

Vec3 project(const Vec3& v, const Vec3& onto) {
    Real denom = onto.length_squared();
    if (denom < 1e-10) {
        return Vec3::Zero();
    }
    return onto * (v.dot(onto) / denom);
}

Vec3 reject(const Vec3& v, const Vec3& from) {
    return v - project(v, from);
}

Vec3 reflect(const Vec3& v, const Vec3& normal) {
    return v - normal * (2.0 * v.dot(normal));
}

Real angle_between(const Vec3& a, const Vec3& b) {
    Real len_a = a.length();
    Real len_b = b.length();
    if (len_a < 1e-10 || len_b < 1e-10) {
        return 0.0;
    }
    Real cos_angle = a.dot(b) / (len_a * len_b);
    // Clamp to [-1, 1] to handle numerical errors
    cos_angle = std::max(-1.0, std::min(1.0, cos_angle));
    return std::acos(cos_angle);
}

bool is_nearly_zero(const Vec3& v, Real epsilon) {
    return v.length_squared() < epsilon * epsilon;
}

bool are_nearly_equal(const Vec3& a, const Vec3& b, Real epsilon) {
    return is_nearly_zero(a - b, epsilon);
}

} // namespace math

} // namespace jaguar
