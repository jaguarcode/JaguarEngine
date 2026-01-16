/**
 * @file quaternion.cpp
 * @brief Quaternion math implementation
 */

#include "jaguar/core/types.h"
#include <cmath>
#include <algorithm>

namespace jaguar {

// ============================================================================
// Quat Member Function Implementations
// ============================================================================

Vec3 Quat::rotate(const Vec3& v) const noexcept {
    // Optimized quaternion rotation: q * v * q^-1
    // Using Rodrigues' rotation formula optimization
    Vec3 qv{x, y, z};
    Vec3 uv = qv.cross(v);
    Vec3 uuv = qv.cross(uv);
    uv = uv * (2.0 * w);
    uuv = uuv * 2.0;
    return v + uv + uuv;
}

Real Quat::norm() const noexcept {
    return std::sqrt(w*w + x*x + y*y + z*z);
}

Quat Quat::normalized() const noexcept {
    Real len = norm();
    if (len > 1e-10) {
        return {w/len, x/len, y/len, z/len};
    }
    return Identity();
}

Quat Quat::inverse() const noexcept {
    Real n2 = norm_squared();
    if (n2 > 1e-10) {
        Real inv_n2 = 1.0 / n2;
        return {w * inv_n2, -x * inv_n2, -y * inv_n2, -z * inv_n2};
    }
    return Identity();
}

Quat Quat::from_axis_angle(const Vec3& axis, Real angle) noexcept {
    Real half_angle = angle * 0.5;
    Real s = std::sin(half_angle);
    Vec3 n = axis.normalized();
    return {std::cos(half_angle), n.x * s, n.y * s, n.z * s};
}

Quat Quat::from_euler(Real roll, Real pitch, Real yaw) noexcept {
    Real cr = std::cos(roll * 0.5);
    Real sr = std::sin(roll * 0.5);
    Real cp = std::cos(pitch * 0.5);
    Real sp = std::sin(pitch * 0.5);
    Real cy = std::cos(yaw * 0.5);
    Real sy = std::sin(yaw * 0.5);

    return {
        cr * cp * cy + sr * sp * sy,  // w
        sr * cp * cy - cr * sp * sy,  // x
        cr * sp * cy + sr * cp * sy,  // y
        cr * cp * sy - sr * sp * cy   // z
    };
}

void Quat::to_euler(Real& roll, Real& pitch, Real& yaw) const noexcept {
    // Roll (x-axis rotation)
    Real sinr_cosp = 2.0 * (w * x + y * z);
    Real cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    Real sinp = 2.0 * (w * y - z * x);
    if (std::abs(sinp) >= 1.0) {
        pitch = std::copysign(constants::PI / 2.0, sinp);  // Gimbal lock
    } else {
        pitch = std::asin(sinp);
    }

    // Yaw (z-axis rotation)
    Real siny_cosp = 2.0 * (w * z + x * y);
    Real cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

Mat3x3 Quat::to_rotation_matrix() const noexcept {
    Mat3x3 m;

    Real xx = x * x;
    Real yy = y * y;
    Real zz = z * z;
    Real xy = x * y;
    Real xz = x * z;
    Real yz = y * z;
    Real wx = w * x;
    Real wy = w * y;
    Real wz = w * z;

    m(0, 0) = 1.0 - 2.0 * (yy + zz);
    m(0, 1) = 2.0 * (xy - wz);
    m(0, 2) = 2.0 * (xz + wy);
    m(1, 0) = 2.0 * (xy + wz);
    m(1, 1) = 1.0 - 2.0 * (xx + zz);
    m(1, 2) = 2.0 * (yz - wx);
    m(2, 0) = 2.0 * (xz - wy);
    m(2, 1) = 2.0 * (yz + wx);
    m(2, 2) = 1.0 - 2.0 * (xx + yy);

    return m;
}

// ============================================================================
// Quaternion Utilities Namespace
// ============================================================================

namespace math {

Quat normalize(const Quat& q) {
    return q.normalized();
}

Quat conjugate(const Quat& q) {
    return q.conjugate();
}

Quat inverse(const Quat& q) {
    return q.inverse();
}

Real dot(const Quat& a, const Quat& b) {
    return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
}

Quat slerp(const Quat& a, const Quat& b, Real t) {
    // Spherical linear interpolation
    Real cos_half_theta = dot(a, b);

    // If a and b are nearly the same, use linear interpolation
    if (std::abs(cos_half_theta) >= 1.0 - 1e-10) {
        return Quat{
            a.w + t * (b.w - a.w),
            a.x + t * (b.x - a.x),
            a.y + t * (b.y - a.y),
            a.z + t * (b.z - a.z)
        }.normalized();
    }

    // If the dot product is negative, negate one quaternion
    // to take the shorter path
    Quat b_adj = b;
    if (cos_half_theta < 0.0) {
        b_adj = Quat{-b.w, -b.x, -b.y, -b.z};
        cos_half_theta = -cos_half_theta;
    }

    Real half_theta = std::acos(cos_half_theta);
    Real sin_half_theta = std::sqrt(1.0 - cos_half_theta * cos_half_theta);

    Real ratio_a = std::sin((1.0 - t) * half_theta) / sin_half_theta;
    Real ratio_b = std::sin(t * half_theta) / sin_half_theta;

    return Quat{
        a.w * ratio_a + b_adj.w * ratio_b,
        a.x * ratio_a + b_adj.x * ratio_b,
        a.y * ratio_a + b_adj.y * ratio_b,
        a.z * ratio_a + b_adj.z * ratio_b
    };
}

Quat nlerp(const Quat& a, const Quat& b, Real t) {
    // Normalized linear interpolation (faster than slerp, good enough for small angles)
    Real dot_val = dot(a, b);
    Quat b_adj = (dot_val < 0.0) ? Quat{-b.w, -b.x, -b.y, -b.z} : b;

    return Quat{
        a.w + t * (b_adj.w - a.w),
        a.x + t * (b_adj.x - a.x),
        a.y + t * (b_adj.y - a.y),
        a.z + t * (b_adj.z - a.z)
    }.normalized();
}

Real angle_between(const Quat& a, const Quat& b) {
    Real dot_val = std::abs(dot(a, b));
    dot_val = std::min(1.0, dot_val);  // Clamp for numerical safety
    return 2.0 * std::acos(dot_val);
}

bool are_nearly_equal(const Quat& a, const Quat& b, Real epsilon) {
    // Quaternions q and -q represent the same rotation
    Real d1 = (a.w - b.w) * (a.w - b.w) +
              (a.x - b.x) * (a.x - b.x) +
              (a.y - b.y) * (a.y - b.y) +
              (a.z - b.z) * (a.z - b.z);

    Real d2 = (a.w + b.w) * (a.w + b.w) +
              (a.x + b.x) * (a.x + b.x) +
              (a.y + b.y) * (a.y + b.y) +
              (a.z + b.z) * (a.z + b.z);

    return std::min(d1, d2) < epsilon * epsilon;
}

} // namespace math

} // namespace jaguar
