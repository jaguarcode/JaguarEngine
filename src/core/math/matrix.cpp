/**
 * @file matrix.cpp
 * @brief Matrix math implementation
 */

#include "jaguar/core/types.h"
#include <cmath>

namespace jaguar {

// ============================================================================
// Mat3x3 Member Function Implementations
// ============================================================================

Mat3x3 Mat3x3::inverse() const noexcept {
    Real det = determinant();
    if (std::abs(det) < 1e-10) {
        return Identity();  // Return identity if not invertible
    }

    Real inv_det = 1.0 / det;
    Mat3x3 result;

    // Compute cofactor matrix transpose (adjugate) divided by determinant
    result(0, 0) = (m[4]*m[8] - m[5]*m[7]) * inv_det;
    result(0, 1) = (m[2]*m[7] - m[1]*m[8]) * inv_det;
    result(0, 2) = (m[1]*m[5] - m[2]*m[4]) * inv_det;

    result(1, 0) = (m[5]*m[6] - m[3]*m[8]) * inv_det;
    result(1, 1) = (m[0]*m[8] - m[2]*m[6]) * inv_det;
    result(1, 2) = (m[2]*m[3] - m[0]*m[5]) * inv_det;

    result(2, 0) = (m[3]*m[7] - m[4]*m[6]) * inv_det;
    result(2, 1) = (m[1]*m[6] - m[0]*m[7]) * inv_det;
    result(2, 2) = (m[0]*m[4] - m[1]*m[3]) * inv_det;

    return result;
}

// ============================================================================
// Matrix Utilities Namespace
// ============================================================================

namespace math {

Mat3x3 transpose(const Mat3x3& m) {
    return m.transpose();
}

Mat3x3 inverse(const Mat3x3& m) {
    return m.inverse();
}

Real determinant(const Mat3x3& m) {
    return m.determinant();
}

Real trace(const Mat3x3& m) {
    return m(0, 0) + m(1, 1) + m(2, 2);
}

bool is_orthogonal(const Mat3x3& m, Real epsilon) {
    // A matrix is orthogonal if M * M^T = I
    Mat3x3 mt = m.transpose();
    Mat3x3 product = m * mt;
    Mat3x3 identity = Mat3x3::Identity();

    for (SizeT i = 0; i < 9; ++i) {
        if (std::abs(product.m[i] - identity.m[i]) > epsilon) {
            return false;
        }
    }
    return true;
}

bool is_symmetric(const Mat3x3& m, Real epsilon) {
    return std::abs(m(0, 1) - m(1, 0)) < epsilon &&
           std::abs(m(0, 2) - m(2, 0)) < epsilon &&
           std::abs(m(1, 2) - m(2, 1)) < epsilon;
}

bool are_nearly_equal(const Mat3x3& a, const Mat3x3& b, Real epsilon) {
    for (SizeT i = 0; i < 9; ++i) {
        if (std::abs(a.m[i] - b.m[i]) > epsilon) {
            return false;
        }
    }
    return true;
}

Mat3x3 outer_product(const Vec3& a, const Vec3& b) {
    Mat3x3 result;
    result(0, 0) = a.x * b.x; result(0, 1) = a.x * b.y; result(0, 2) = a.x * b.z;
    result(1, 0) = a.y * b.x; result(1, 1) = a.y * b.y; result(1, 2) = a.y * b.z;
    result(2, 0) = a.z * b.x; result(2, 1) = a.z * b.y; result(2, 2) = a.z * b.z;
    return result;
}

Mat3x3 skew_symmetric(const Vec3& v) {
    // Creates skew-symmetric (cross-product) matrix
    // [v]_x such that [v]_x * u = v × u
    Mat3x3 result = Mat3x3::Zero();
    result(0, 1) = -v.z;
    result(0, 2) = v.y;
    result(1, 0) = v.z;
    result(1, 2) = -v.x;
    result(2, 0) = -v.y;
    result(2, 1) = v.x;
    return result;
}

Mat3x3 rotation_x(Real angle) {
    Real c = std::cos(angle);
    Real s = std::sin(angle);
    Mat3x3 result = Mat3x3::Identity();
    result(1, 1) = c;  result(1, 2) = -s;
    result(2, 1) = s;  result(2, 2) = c;
    return result;
}

Mat3x3 rotation_y(Real angle) {
    Real c = std::cos(angle);
    Real s = std::sin(angle);
    Mat3x3 result = Mat3x3::Identity();
    result(0, 0) = c;  result(0, 2) = s;
    result(2, 0) = -s; result(2, 2) = c;
    return result;
}

Mat3x3 rotation_z(Real angle) {
    Real c = std::cos(angle);
    Real s = std::sin(angle);
    Mat3x3 result = Mat3x3::Identity();
    result(0, 0) = c;  result(0, 1) = -s;
    result(1, 0) = s;  result(1, 1) = c;
    return result;
}

Mat3x3 rotation_axis_angle(const Vec3& axis, Real angle) {
    // Rodrigues' rotation formula
    Vec3 k = axis.normalized();
    Real c = std::cos(angle);
    Real s = std::sin(angle);
    Real one_minus_c = 1.0 - c;

    Mat3x3 K = skew_symmetric(k);
    Mat3x3 K2 = K * K;

    // R = I + sin(θ)*K + (1-cos(θ))*K²
    Mat3x3 result = Mat3x3::Identity();
    for (SizeT i = 0; i < 9; ++i) {
        result.m[i] += s * K.m[i] + one_minus_c * K2.m[i];
    }
    return result;
}

Mat3x3 from_euler_xyz(Real roll, Real pitch, Real yaw) {
    // Rotation order: X (roll) -> Y (pitch) -> Z (yaw)
    return rotation_z(yaw) * rotation_y(pitch) * rotation_x(roll);
}

void to_euler_xyz(const Mat3x3& m, Real& roll, Real& pitch, Real& yaw) {
    // Extract Euler angles from rotation matrix (XYZ order)
    pitch = std::asin(-m(2, 0));

    if (std::abs(std::cos(pitch)) > 1e-6) {
        roll = std::atan2(m(2, 1), m(2, 2));
        yaw = std::atan2(m(1, 0), m(0, 0));
    } else {
        // Gimbal lock
        roll = std::atan2(-m(1, 2), m(1, 1));
        yaw = 0.0;
    }
}

} // namespace math

} // namespace jaguar
