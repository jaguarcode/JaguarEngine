/**
 * @file test_math.cpp
 * @brief Comprehensive unit tests for JaguarEngine math library
 */

#include <gtest/gtest.h>
#include "jaguar/core/types.h"
#include <cmath>

using namespace jaguar;

// ============================================================================
// Vec3 Tests
// ============================================================================

class Vec3Test : public ::testing::Test {
protected:
    static constexpr Real EPSILON = 1e-10;

    bool nearly_equal(Real a, Real b, Real eps = EPSILON) {
        return std::abs(a - b) < eps;
    }

    bool nearly_equal(const Vec3& a, const Vec3& b, Real eps = EPSILON) {
        return nearly_equal(a.x, b.x, eps) &&
               nearly_equal(a.y, b.y, eps) &&
               nearly_equal(a.z, b.z, eps);
    }
};

TEST_F(Vec3Test, DefaultConstruction) {
    Vec3 v;
    EXPECT_EQ(v.x, 0.0);
    EXPECT_EQ(v.y, 0.0);
    EXPECT_EQ(v.z, 0.0);
}

TEST_F(Vec3Test, ValueConstruction) {
    Vec3 v{1.0, 2.0, 3.0};
    EXPECT_EQ(v.x, 1.0);
    EXPECT_EQ(v.y, 2.0);
    EXPECT_EQ(v.z, 3.0);
}

TEST_F(Vec3Test, Addition) {
    Vec3 a{1.0, 2.0, 3.0};
    Vec3 b{4.0, 5.0, 6.0};
    Vec3 c = a + b;
    EXPECT_TRUE(nearly_equal(c, Vec3{5.0, 7.0, 9.0}));
}

TEST_F(Vec3Test, Subtraction) {
    Vec3 a{5.0, 7.0, 9.0};
    Vec3 b{1.0, 2.0, 3.0};
    Vec3 c = a - b;
    EXPECT_TRUE(nearly_equal(c, Vec3{4.0, 5.0, 6.0}));
}

TEST_F(Vec3Test, ScalarMultiplication) {
    Vec3 v{1.0, 2.0, 3.0};
    Vec3 result = v * 2.0;
    EXPECT_TRUE(nearly_equal(result, Vec3{2.0, 4.0, 6.0}));

    // Test scalar on left
    Vec3 result2 = 3.0 * v;
    EXPECT_TRUE(nearly_equal(result2, Vec3{3.0, 6.0, 9.0}));
}

TEST_F(Vec3Test, ScalarDivision) {
    Vec3 v{2.0, 4.0, 6.0};
    Vec3 result = v / 2.0;
    EXPECT_TRUE(nearly_equal(result, Vec3{1.0, 2.0, 3.0}));
}

TEST_F(Vec3Test, UnaryMinus) {
    Vec3 v{1.0, -2.0, 3.0};
    Vec3 result = -v;
    EXPECT_TRUE(nearly_equal(result, Vec3{-1.0, 2.0, -3.0}));
}

TEST_F(Vec3Test, DotProduct) {
    Vec3 a{1.0, 0.0, 0.0};
    Vec3 b{0.0, 1.0, 0.0};
    EXPECT_NEAR(a.dot(b), 0.0, EPSILON);  // Perpendicular

    Vec3 c{1.0, 2.0, 3.0};
    Vec3 d{4.0, 5.0, 6.0};
    EXPECT_NEAR(c.dot(d), 32.0, EPSILON);  // 1*4 + 2*5 + 3*6 = 32
}

TEST_F(Vec3Test, CrossProduct) {
    Vec3 i = Vec3::UnitX();
    Vec3 j = Vec3::UnitY();
    Vec3 k = i.cross(j);
    EXPECT_TRUE(nearly_equal(k, Vec3::UnitZ()));

    // i × j = k, j × k = i, k × i = j
    EXPECT_TRUE(nearly_equal(j.cross(k), Vec3::UnitX()));
    EXPECT_TRUE(nearly_equal(k.cross(i), Vec3::UnitY()));
}

TEST_F(Vec3Test, LengthSquared) {
    Vec3 v{3.0, 4.0, 0.0};
    EXPECT_NEAR(v.length_squared(), 25.0, EPSILON);
}

TEST_F(Vec3Test, Length) {
    Vec3 v{3.0, 4.0, 0.0};
    EXPECT_NEAR(v.length(), 5.0, EPSILON);

    Vec3 v2{1.0, 1.0, 1.0};
    EXPECT_NEAR(v2.length(), std::sqrt(3.0), EPSILON);
}

TEST_F(Vec3Test, Normalize) {
    Vec3 v{3.0, 4.0, 0.0};
    Vec3 n = v.normalized();
    EXPECT_NEAR(n.length(), 1.0, EPSILON);
    EXPECT_TRUE(nearly_equal(n, Vec3{0.6, 0.8, 0.0}));
}

TEST_F(Vec3Test, NormalizeZeroVector) {
    Vec3 zero = Vec3::Zero();
    Vec3 n = zero.normalized();
    // Should return original (zero) without crashing
    EXPECT_TRUE(nearly_equal(n, Vec3::Zero()));
}

TEST_F(Vec3Test, CompoundAssignment) {
    Vec3 v{1.0, 2.0, 3.0};

    v += Vec3{1.0, 1.0, 1.0};
    EXPECT_TRUE(nearly_equal(v, Vec3{2.0, 3.0, 4.0}));

    v -= Vec3{1.0, 1.0, 1.0};
    EXPECT_TRUE(nearly_equal(v, Vec3{1.0, 2.0, 3.0}));

    v *= 2.0;
    EXPECT_TRUE(nearly_equal(v, Vec3{2.0, 4.0, 6.0}));

    v /= 2.0;
    EXPECT_TRUE(nearly_equal(v, Vec3{1.0, 2.0, 3.0}));
}

TEST_F(Vec3Test, StaticVectors) {
    EXPECT_TRUE(nearly_equal(Vec3::Zero(), Vec3{0.0, 0.0, 0.0}));
    EXPECT_TRUE(nearly_equal(Vec3::UnitX(), Vec3{1.0, 0.0, 0.0}));
    EXPECT_TRUE(nearly_equal(Vec3::UnitY(), Vec3{0.0, 1.0, 0.0}));
    EXPECT_TRUE(nearly_equal(Vec3::UnitZ(), Vec3{0.0, 0.0, 1.0}));
}

// ============================================================================
// Quaternion Tests
// ============================================================================

class QuatTest : public ::testing::Test {
protected:
    static constexpr Real EPSILON = 1e-10;
    static constexpr Real PI = constants::PI;

    bool nearly_equal(Real a, Real b, Real eps = EPSILON) {
        return std::abs(a - b) < eps;
    }

    bool nearly_equal(const Vec3& a, const Vec3& b, Real eps = EPSILON) {
        return nearly_equal(a.x, b.x, eps) &&
               nearly_equal(a.y, b.y, eps) &&
               nearly_equal(a.z, b.z, eps);
    }

    bool nearly_equal_quat(const Quat& a, const Quat& b, Real eps = EPSILON) {
        // Quaternions q and -q represent the same rotation
        Real d1 = std::abs(a.w - b.w) + std::abs(a.x - b.x) +
                  std::abs(a.y - b.y) + std::abs(a.z - b.z);
        Real d2 = std::abs(a.w + b.w) + std::abs(a.x + b.x) +
                  std::abs(a.y + b.y) + std::abs(a.z + b.z);
        return std::min(d1, d2) < eps;
    }
};

TEST_F(QuatTest, DefaultConstruction) {
    Quat q;
    EXPECT_EQ(q.w, 1.0);
    EXPECT_EQ(q.x, 0.0);
    EXPECT_EQ(q.y, 0.0);
    EXPECT_EQ(q.z, 0.0);
}

TEST_F(QuatTest, Identity) {
    Quat q = Quat::Identity();
    EXPECT_EQ(q.w, 1.0);
    EXPECT_EQ(q.x, 0.0);
    EXPECT_EQ(q.y, 0.0);
    EXPECT_EQ(q.z, 0.0);
}

TEST_F(QuatTest, Conjugate) {
    Quat q{0.5, 0.5, 0.5, 0.5};
    Quat conj = q.conjugate();
    EXPECT_NEAR(conj.w, 0.5, EPSILON);
    EXPECT_NEAR(conj.x, -0.5, EPSILON);
    EXPECT_NEAR(conj.y, -0.5, EPSILON);
    EXPECT_NEAR(conj.z, -0.5, EPSILON);
}

TEST_F(QuatTest, NormSquared) {
    Quat q{1.0, 2.0, 3.0, 4.0};
    EXPECT_NEAR(q.norm_squared(), 30.0, EPSILON);  // 1 + 4 + 9 + 16 = 30
}

TEST_F(QuatTest, Norm) {
    Quat q{0.5, 0.5, 0.5, 0.5};
    EXPECT_NEAR(q.norm(), 1.0, EPSILON);
}

TEST_F(QuatTest, Normalize) {
    Quat q{1.0, 1.0, 1.0, 1.0};
    Quat n = q.normalized();
    EXPECT_NEAR(n.norm(), 1.0, EPSILON);
    EXPECT_NEAR(n.w, 0.5, EPSILON);
    EXPECT_NEAR(n.x, 0.5, EPSILON);
    EXPECT_NEAR(n.y, 0.5, EPSILON);
    EXPECT_NEAR(n.z, 0.5, EPSILON);
}

TEST_F(QuatTest, Multiplication) {
    // Identity multiplication
    Quat identity = Quat::Identity();
    Quat q{0.5, 0.5, 0.5, 0.5};
    Quat result = identity * q;
    EXPECT_TRUE(nearly_equal_quat(result, q));

    // Reverse order should also give same result with identity
    result = q * identity;
    EXPECT_TRUE(nearly_equal_quat(result, q));
}

TEST_F(QuatTest, MultiplicationNonCommutative) {
    Quat a = Quat::from_axis_angle(Vec3::UnitX(), PI / 2);
    Quat b = Quat::from_axis_angle(Vec3::UnitY(), PI / 2);

    Quat ab = a * b;
    Quat ba = b * a;

    // Quaternion multiplication is NOT commutative
    EXPECT_FALSE(nearly_equal_quat(ab, ba, 1e-6));
}

TEST_F(QuatTest, FromAxisAngle) {
    // 90° rotation around Z axis
    Quat q = Quat::from_axis_angle(Vec3::UnitZ(), PI / 2);

    // Expected: w = cos(45°), z = sin(45°)
    Real expected_w = std::cos(PI / 4);
    Real expected_z = std::sin(PI / 4);

    EXPECT_NEAR(q.w, expected_w, EPSILON);
    EXPECT_NEAR(q.x, 0.0, EPSILON);
    EXPECT_NEAR(q.y, 0.0, EPSILON);
    EXPECT_NEAR(q.z, expected_z, EPSILON);
}

TEST_F(QuatTest, RotateVector) {
    // 90° rotation around Z axis should rotate X to Y
    Quat q = Quat::from_axis_angle(Vec3::UnitZ(), PI / 2);
    Vec3 v = Vec3::UnitX();
    Vec3 rotated = q.rotate(v);

    EXPECT_NEAR(rotated.x, 0.0, 1e-9);
    EXPECT_NEAR(rotated.y, 1.0, 1e-9);
    EXPECT_NEAR(rotated.z, 0.0, 1e-9);
}

TEST_F(QuatTest, RotateVectorIdentity) {
    Quat identity = Quat::Identity();
    Vec3 v{1.0, 2.0, 3.0};
    Vec3 rotated = identity.rotate(v);
    EXPECT_TRUE(nearly_equal(rotated, v));
}

TEST_F(QuatTest, RotateVector180Degrees) {
    // 180° rotation around Z axis should rotate X to -X
    Quat q = Quat::from_axis_angle(Vec3::UnitZ(), PI);
    Vec3 v = Vec3::UnitX();
    Vec3 rotated = q.rotate(v);

    EXPECT_NEAR(rotated.x, -1.0, 1e-9);
    EXPECT_NEAR(rotated.y, 0.0, 1e-9);
    EXPECT_NEAR(rotated.z, 0.0, 1e-9);
}

TEST_F(QuatTest, Inverse) {
    Quat q = Quat::from_axis_angle(Vec3::UnitZ(), PI / 4);
    Quat inv = q.inverse();

    // q * q^-1 should equal identity
    Quat result = q * inv;
    EXPECT_TRUE(nearly_equal_quat(result, Quat::Identity(), 1e-9));
}

TEST_F(QuatTest, EulerRoundTrip) {
    Real roll = 0.3;
    Real pitch = 0.2;
    Real yaw = 0.1;

    Quat q = Quat::from_euler(roll, pitch, yaw);

    Real r2, p2, y2;
    q.to_euler(r2, p2, y2);

    EXPECT_NEAR(roll, r2, 1e-9);
    EXPECT_NEAR(pitch, p2, 1e-9);
    EXPECT_NEAR(yaw, y2, 1e-9);
}

TEST_F(QuatTest, ToRotationMatrix) {
    // Identity quaternion should give identity matrix
    Quat identity = Quat::Identity();
    Mat3x3 m = identity.to_rotation_matrix();

    EXPECT_NEAR(m(0, 0), 1.0, EPSILON);
    EXPECT_NEAR(m(1, 1), 1.0, EPSILON);
    EXPECT_NEAR(m(2, 2), 1.0, EPSILON);
    EXPECT_NEAR(m(0, 1), 0.0, EPSILON);
    EXPECT_NEAR(m(0, 2), 0.0, EPSILON);
}

// ============================================================================
// Mat3x3 Tests
// ============================================================================

class Mat3x3Test : public ::testing::Test {
protected:
    static constexpr Real EPSILON = 1e-10;

    bool nearly_equal(Real a, Real b, Real eps = EPSILON) {
        return std::abs(a - b) < eps;
    }

    bool nearly_equal_mat(const Mat3x3& a, const Mat3x3& b, Real eps = EPSILON) {
        for (SizeT i = 0; i < 9; ++i) {
            if (!nearly_equal(a.m[i], b.m[i], eps)) return false;
        }
        return true;
    }
};

TEST_F(Mat3x3Test, DefaultConstruction) {
    Mat3x3 m;
    // Default is identity
    EXPECT_NEAR(m(0, 0), 1.0, EPSILON);
    EXPECT_NEAR(m(1, 1), 1.0, EPSILON);
    EXPECT_NEAR(m(2, 2), 1.0, EPSILON);
    EXPECT_NEAR(m(0, 1), 0.0, EPSILON);
}

TEST_F(Mat3x3Test, Identity) {
    Mat3x3 m = Mat3x3::Identity();
    EXPECT_NEAR(m(0, 0), 1.0, EPSILON);
    EXPECT_NEAR(m(1, 1), 1.0, EPSILON);
    EXPECT_NEAR(m(2, 2), 1.0, EPSILON);
}

TEST_F(Mat3x3Test, Zero) {
    Mat3x3 m = Mat3x3::Zero();
    for (SizeT i = 0; i < 9; ++i) {
        EXPECT_NEAR(m.m[i], 0.0, EPSILON);
    }
}

TEST_F(Mat3x3Test, Diagonal) {
    Mat3x3 m = Mat3x3::Diagonal(2.0, 3.0, 4.0);
    EXPECT_NEAR(m(0, 0), 2.0, EPSILON);
    EXPECT_NEAR(m(1, 1), 3.0, EPSILON);
    EXPECT_NEAR(m(2, 2), 4.0, EPSILON);
    EXPECT_NEAR(m(0, 1), 0.0, EPSILON);
}

TEST_F(Mat3x3Test, MatrixVectorMultiply) {
    Mat3x3 m = Mat3x3::Identity();
    Vec3 v{1.0, 2.0, 3.0};
    Vec3 result = m * v;
    EXPECT_NEAR(result.x, 1.0, EPSILON);
    EXPECT_NEAR(result.y, 2.0, EPSILON);
    EXPECT_NEAR(result.z, 3.0, EPSILON);
}

TEST_F(Mat3x3Test, MatrixMatrixMultiply) {
    Mat3x3 a = Mat3x3::Identity();
    Mat3x3 b = Mat3x3::Diagonal(2.0, 3.0, 4.0);
    Mat3x3 c = a * b;
    EXPECT_TRUE(nearly_equal_mat(c, b));
}

TEST_F(Mat3x3Test, Transpose) {
    Mat3x3 m;
    m(0, 1) = 1.0;
    m(0, 2) = 2.0;
    m(1, 2) = 3.0;

    Mat3x3 t = m.transpose();
    EXPECT_NEAR(t(1, 0), 1.0, EPSILON);
    EXPECT_NEAR(t(2, 0), 2.0, EPSILON);
    EXPECT_NEAR(t(2, 1), 3.0, EPSILON);
}

TEST_F(Mat3x3Test, TransposeSymmetric) {
    // Transpose of transpose should equal original
    Mat3x3 m;
    m(0, 1) = 1.0;
    m(1, 2) = 2.0;

    Mat3x3 tt = m.transpose().transpose();
    EXPECT_TRUE(nearly_equal_mat(tt, m));
}

TEST_F(Mat3x3Test, DeterminantIdentity) {
    Mat3x3 m = Mat3x3::Identity();
    EXPECT_NEAR(m.determinant(), 1.0, EPSILON);
}

TEST_F(Mat3x3Test, DeterminantDiagonal) {
    Mat3x3 m = Mat3x3::Diagonal(2.0, 3.0, 4.0);
    EXPECT_NEAR(m.determinant(), 24.0, EPSILON);  // 2 * 3 * 4 = 24
}

TEST_F(Mat3x3Test, InverseIdentity) {
    Mat3x3 m = Mat3x3::Identity();
    Mat3x3 inv = m.inverse();
    EXPECT_TRUE(nearly_equal_mat(inv, Mat3x3::Identity()));
}

TEST_F(Mat3x3Test, InverseDiagonal) {
    Mat3x3 m = Mat3x3::Diagonal(2.0, 4.0, 8.0);
    Mat3x3 inv = m.inverse();
    Mat3x3 expected = Mat3x3::Diagonal(0.5, 0.25, 0.125);
    EXPECT_TRUE(nearly_equal_mat(inv, expected));
}

TEST_F(Mat3x3Test, InverseProduct) {
    Mat3x3 m = Mat3x3::Diagonal(2.0, 3.0, 4.0);
    Mat3x3 inv = m.inverse();
    Mat3x3 product = m * inv;
    EXPECT_TRUE(nearly_equal_mat(product, Mat3x3::Identity(), 1e-9));
}

TEST_F(Mat3x3Test, ScalarMultiply) {
    Mat3x3 m = Mat3x3::Identity();
    Mat3x3 result = m * 2.0;
    EXPECT_NEAR(result(0, 0), 2.0, EPSILON);
    EXPECT_NEAR(result(1, 1), 2.0, EPSILON);
    EXPECT_NEAR(result(2, 2), 2.0, EPSILON);
}

TEST_F(Mat3x3Test, Addition) {
    Mat3x3 a = Mat3x3::Identity();
    Mat3x3 b = Mat3x3::Identity();
    Mat3x3 c = a + b;
    EXPECT_NEAR(c(0, 0), 2.0, EPSILON);
    EXPECT_NEAR(c(1, 1), 2.0, EPSILON);
    EXPECT_NEAR(c(2, 2), 2.0, EPSILON);
}

TEST_F(Mat3x3Test, InertiaMatrix) {
    Mat3x3 I = Mat3x3::Inertia(100.0, 200.0, 300.0, 10.0, 20.0, 30.0);
    EXPECT_NEAR(I(0, 0), 100.0, EPSILON);  // Ixx
    EXPECT_NEAR(I(1, 1), 200.0, EPSILON);  // Iyy
    EXPECT_NEAR(I(2, 2), 300.0, EPSILON);  // Izz
    EXPECT_NEAR(I(0, 1), -10.0, EPSILON);  // -Ixy
    EXPECT_NEAR(I(1, 0), -10.0, EPSILON);  // -Ixy (symmetric)
}

// ============================================================================
// Physical Constants Tests
// ============================================================================

TEST(ConstantsTest, GravitationalConstant) {
    EXPECT_NEAR(constants::G, 6.67430e-11, 1e-15);
}

TEST(ConstantsTest, StandardGravity) {
    EXPECT_NEAR(constants::G0, 9.80665, 1e-5);
}

TEST(ConstantsTest, EarthParameters) {
    EXPECT_NEAR(constants::R_EARTH_EQUATOR, 6378137.0, 1.0);
    EXPECT_NEAR(constants::MU_EARTH, 3.986004418e14, 1e9);
}

TEST(ConstantsTest, AtmosphericConstants) {
    EXPECT_NEAR(constants::P0, 101325.0, 1.0);
    EXPECT_NEAR(constants::T0, 288.15, 0.01);
    EXPECT_NEAR(constants::RHO0, 1.225, 0.001);
}

TEST(ConstantsTest, ConversionFactors) {
    // Degrees to radians
    EXPECT_NEAR(180.0 * constants::DEG_TO_RAD, constants::PI, 1e-10);

    // Feet to meters
    EXPECT_NEAR(1.0 * constants::FT_TO_M, 0.3048, 1e-6);

    // Round trip conversions
    EXPECT_NEAR(1.0 * constants::FT_TO_M * constants::M_TO_FT, 1.0, 1e-10);
}
