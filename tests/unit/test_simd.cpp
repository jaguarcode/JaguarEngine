/**
 * @file test_simd.cpp
 * @brief Unit tests for SIMD batch operations
 */

#include <gtest/gtest.h>
#include "jaguar/core/simd.h"
#include <vector>
#include <cmath>

using namespace jaguar;
using namespace jaguar::simd;

// ============================================================================
// Test Fixture
// ============================================================================

class SIMDTest : public ::testing::Test {
protected:
    static constexpr Real EPSILON = 1e-10;
    static constexpr SizeT TEST_SIZE = 100;  // Large enough to test SIMD + scalar tail

    std::vector<Real> px, py, pz;
    std::vector<Real> vx, vy, vz;
    std::vector<Real> ax, ay, az;
    std::vector<Real> fx, fy, fz;
    std::vector<Real> masses;
    std::vector<Real> output;

    void SetUp() override {
        // Initialize vectors with test data
        px.resize(TEST_SIZE);
        py.resize(TEST_SIZE);
        pz.resize(TEST_SIZE);
        vx.resize(TEST_SIZE);
        vy.resize(TEST_SIZE);
        vz.resize(TEST_SIZE);
        ax.resize(TEST_SIZE);
        ay.resize(TEST_SIZE);
        az.resize(TEST_SIZE);
        fx.resize(TEST_SIZE);
        fy.resize(TEST_SIZE);
        fz.resize(TEST_SIZE);
        masses.resize(TEST_SIZE);
        output.resize(TEST_SIZE);

        // Fill with deterministic test data
        for (SizeT i = 0; i < TEST_SIZE; ++i) {
            Real idx = static_cast<Real>(i);
            px[i] = idx * 1.0;
            py[i] = idx * 2.0;
            pz[i] = idx * 3.0;
            vx[i] = idx * 0.1;
            vy[i] = idx * 0.2;
            vz[i] = idx * 0.3;
            ax[i] = idx * 0.01;
            ay[i] = idx * 0.02;
            az[i] = idx * 0.03;
            fx[i] = idx * 10.0;
            fy[i] = idx * 20.0;
            fz[i] = idx * 30.0;
            masses[i] = 1.0 + idx * 0.1;  // Avoid division by zero
        }
    }

    bool nearly_equal(Real a, Real b, Real eps = EPSILON) {
        return std::abs(a - b) < eps;
    }
};

// ============================================================================
// Utility Tests
// ============================================================================

TEST_F(SIMDTest, SIMDAvailability) {
    // Just check these functions work - actual value depends on build
    bool available = simd_available();
    const char* instruction_set = simd_instruction_set();

    EXPECT_NE(instruction_set, nullptr);

    // The implementation knows which SIMD path was compiled
    // We just verify the function returns a valid string
    if (available) {
        EXPECT_STREQ(instruction_set, "AVX2+FMA");
    } else {
        EXPECT_STREQ(instruction_set, "Scalar");
    }
}

TEST_F(SIMDTest, AlignmentCheck) {
    // Test aligned pointer
    alignas(SIMD_ALIGNMENT) Real aligned_data[4];
    EXPECT_TRUE(is_simd_aligned(aligned_data));

    // Test potentially unaligned
    std::vector<Real> vec(10);
    // We can't guarantee alignment, just test the function works
    [[maybe_unused]] bool aligned = is_simd_aligned(vec.data());
}

// ============================================================================
// Position Update Tests
// ============================================================================

TEST_F(SIMDTest, BatchPositionUpdate) {
    Real dt = 0.01;

    // Make copies for expected values
    std::vector<Real> expected_px = px;
    std::vector<Real> expected_py = py;
    std::vector<Real> expected_pz = pz;

    // Compute expected values manually
    for (SizeT i = 0; i < TEST_SIZE; ++i) {
        expected_px[i] += vx[i] * dt;
        expected_py[i] += vy[i] * dt;
        expected_pz[i] += vz[i] * dt;
    }

    // Run SIMD batch update
    batch_position_update(px.data(), py.data(), pz.data(),
                          vx.data(), vy.data(), vz.data(),
                          TEST_SIZE, dt);

    // Verify results
    for (SizeT i = 0; i < TEST_SIZE; ++i) {
        EXPECT_TRUE(nearly_equal(px[i], expected_px[i])) << "px mismatch at index " << i;
        EXPECT_TRUE(nearly_equal(py[i], expected_py[i])) << "py mismatch at index " << i;
        EXPECT_TRUE(nearly_equal(pz[i], expected_pz[i])) << "pz mismatch at index " << i;
    }
}

TEST_F(SIMDTest, BatchPositionUpdateSmallCount) {
    // Test with count smaller than SIMD width
    Real dt = 0.01;
    SizeT small_count = 3;  // Less than AVX_DOUBLES (4)

    std::vector<Real> small_px = {1.0, 2.0, 3.0};
    std::vector<Real> small_py = {4.0, 5.0, 6.0};
    std::vector<Real> small_pz = {7.0, 8.0, 9.0};
    std::vector<Real> small_vx = {0.1, 0.2, 0.3};
    std::vector<Real> small_vy = {0.4, 0.5, 0.6};
    std::vector<Real> small_vz = {0.7, 0.8, 0.9};

    batch_position_update(small_px.data(), small_py.data(), small_pz.data(),
                          small_vx.data(), small_vy.data(), small_vz.data(),
                          small_count, dt);

    EXPECT_TRUE(nearly_equal(small_px[0], 1.0 + 0.1 * dt));
    EXPECT_TRUE(nearly_equal(small_px[1], 2.0 + 0.2 * dt));
    EXPECT_TRUE(nearly_equal(small_px[2], 3.0 + 0.3 * dt));
}

// ============================================================================
// Velocity Update Tests
// ============================================================================

TEST_F(SIMDTest, BatchVelocityUpdate) {
    Real dt = 0.01;

    std::vector<Real> expected_vx = vx;
    std::vector<Real> expected_vy = vy;
    std::vector<Real> expected_vz = vz;

    for (SizeT i = 0; i < TEST_SIZE; ++i) {
        expected_vx[i] += ax[i] * dt;
        expected_vy[i] += ay[i] * dt;
        expected_vz[i] += az[i] * dt;
    }

    batch_velocity_update(vx.data(), vy.data(), vz.data(),
                          ax.data(), ay.data(), az.data(),
                          TEST_SIZE, dt);

    for (SizeT i = 0; i < TEST_SIZE; ++i) {
        EXPECT_TRUE(nearly_equal(vx[i], expected_vx[i])) << "vx mismatch at index " << i;
        EXPECT_TRUE(nearly_equal(vy[i], expected_vy[i])) << "vy mismatch at index " << i;
        EXPECT_TRUE(nearly_equal(vz[i], expected_vz[i])) << "vz mismatch at index " << i;
    }
}

// ============================================================================
// Force to Acceleration Tests
// ============================================================================

TEST_F(SIMDTest, BatchForceToAcceleration) {
    std::vector<Real> out_ax(TEST_SIZE);
    std::vector<Real> out_ay(TEST_SIZE);
    std::vector<Real> out_az(TEST_SIZE);

    batch_force_to_acceleration(out_ax.data(), out_ay.data(), out_az.data(),
                                fx.data(), fy.data(), fz.data(),
                                masses.data(), TEST_SIZE);

    for (SizeT i = 0; i < TEST_SIZE; ++i) {
        Real expected_ax = fx[i] / masses[i];
        Real expected_ay = fy[i] / masses[i];
        Real expected_az = fz[i] / masses[i];

        EXPECT_TRUE(nearly_equal(out_ax[i], expected_ax)) << "ax mismatch at index " << i;
        EXPECT_TRUE(nearly_equal(out_ay[i], expected_ay)) << "ay mismatch at index " << i;
        EXPECT_TRUE(nearly_equal(out_az[i], expected_az)) << "az mismatch at index " << i;
    }
}

// ============================================================================
// Gravity Tests
// ============================================================================

TEST_F(SIMDTest, BatchAddGravity) {
    std::vector<Real> fz_copy = fz;
    Real g = 9.80665;

    batch_add_gravity(fz_copy.data(), masses.data(), g, TEST_SIZE);

    for (SizeT i = 0; i < TEST_SIZE; ++i) {
        Real expected = fz[i] + masses[i] * g;
        EXPECT_TRUE(nearly_equal(fz_copy[i], expected)) << "fz mismatch at index " << i;
    }
}

// ============================================================================
// Vector Operation Tests
// ============================================================================

TEST_F(SIMDTest, BatchScale) {
    Real scalar = 2.5;

    batch_scale(output.data(), px.data(), scalar, TEST_SIZE);

    for (SizeT i = 0; i < TEST_SIZE; ++i) {
        Real expected = px[i] * scalar;
        EXPECT_TRUE(nearly_equal(output[i], expected)) << "mismatch at index " << i;
    }
}

TEST_F(SIMDTest, BatchAdd) {
    batch_add(output.data(), px.data(), vx.data(), TEST_SIZE);

    for (SizeT i = 0; i < TEST_SIZE; ++i) {
        Real expected = px[i] + vx[i];
        EXPECT_TRUE(nearly_equal(output[i], expected)) << "mismatch at index " << i;
    }
}

TEST_F(SIMDTest, BatchFMA) {
    Real scalar = 0.5;

    batch_fma(output.data(), px.data(), vx.data(), scalar, TEST_SIZE);

    for (SizeT i = 0; i < TEST_SIZE; ++i) {
        Real expected = px[i] + vx[i] * scalar;
        EXPECT_TRUE(nearly_equal(output[i], expected)) << "mismatch at index " << i;
    }
}

TEST_F(SIMDTest, BatchDotProduct) {
    batch_dot_product(output.data(),
                      px.data(), py.data(), pz.data(),
                      vx.data(), vy.data(), vz.data(),
                      TEST_SIZE);

    for (SizeT i = 0; i < TEST_SIZE; ++i) {
        Real expected = px[i] * vx[i] + py[i] * vy[i] + pz[i] * vz[i];
        EXPECT_TRUE(nearly_equal(output[i], expected)) << "mismatch at index " << i;
    }
}

TEST_F(SIMDTest, BatchLengthSquared) {
    batch_length_squared(output.data(), px.data(), py.data(), pz.data(), TEST_SIZE);

    for (SizeT i = 0; i < TEST_SIZE; ++i) {
        Real expected = px[i] * px[i] + py[i] * py[i] + pz[i] * pz[i];
        EXPECT_TRUE(nearly_equal(output[i], expected)) << "mismatch at index " << i;
    }
}

// ============================================================================
// Edge Case Tests
// ============================================================================

TEST_F(SIMDTest, EmptyArrays) {
    // Should not crash with zero count
    batch_position_update(px.data(), py.data(), pz.data(),
                          vx.data(), vy.data(), vz.data(),
                          0, 0.01);

    batch_velocity_update(vx.data(), vy.data(), vz.data(),
                          ax.data(), ay.data(), az.data(),
                          0, 0.01);

    batch_scale(output.data(), px.data(), 2.0, 0);
}

TEST_F(SIMDTest, SingleElement) {
    std::vector<Real> single_px = {1.0};
    std::vector<Real> single_py = {2.0};
    std::vector<Real> single_pz = {3.0};
    std::vector<Real> single_vx = {0.1};
    std::vector<Real> single_vy = {0.2};
    std::vector<Real> single_vz = {0.3};

    Real dt = 1.0;
    batch_position_update(single_px.data(), single_py.data(), single_pz.data(),
                          single_vx.data(), single_vy.data(), single_vz.data(),
                          1, dt);

    EXPECT_TRUE(nearly_equal(single_px[0], 1.1));
    EXPECT_TRUE(nearly_equal(single_py[0], 2.2));
    EXPECT_TRUE(nearly_equal(single_pz[0], 3.3));
}

TEST_F(SIMDTest, ExactSIMDWidthMultiple) {
    // Test with count that's exactly a multiple of AVX_DOUBLES (4)
    SizeT exact_count = AVX_DOUBLES * 4;  // 16 elements

    std::vector<Real> exact_px(exact_count, 1.0);
    std::vector<Real> exact_py(exact_count, 2.0);
    std::vector<Real> exact_pz(exact_count, 3.0);
    std::vector<Real> exact_vx(exact_count, 0.1);
    std::vector<Real> exact_vy(exact_count, 0.2);
    std::vector<Real> exact_vz(exact_count, 0.3);

    Real dt = 1.0;
    batch_position_update(exact_px.data(), exact_py.data(), exact_pz.data(),
                          exact_vx.data(), exact_vy.data(), exact_vz.data(),
                          exact_count, dt);

    for (SizeT i = 0; i < exact_count; ++i) {
        EXPECT_TRUE(nearly_equal(exact_px[i], 1.1)) << "px mismatch at " << i;
        EXPECT_TRUE(nearly_equal(exact_py[i], 2.2)) << "py mismatch at " << i;
        EXPECT_TRUE(nearly_equal(exact_pz[i], 3.3)) << "pz mismatch at " << i;
    }
}

// ============================================================================
// Numerical Stability Tests
// ============================================================================

TEST_F(SIMDTest, LargeValues) {
    // Test with large values to check for overflow issues
    std::vector<Real> large_px = {1e10, 1e10, 1e10, 1e10};
    std::vector<Real> large_py = {1e10, 1e10, 1e10, 1e10};
    std::vector<Real> large_pz = {1e10, 1e10, 1e10, 1e10};
    std::vector<Real> large_vx = {1e5, 1e5, 1e5, 1e5};
    std::vector<Real> large_vy = {1e5, 1e5, 1e5, 1e5};
    std::vector<Real> large_vz = {1e5, 1e5, 1e5, 1e5};

    Real dt = 1.0;
    batch_position_update(large_px.data(), large_py.data(), large_pz.data(),
                          large_vx.data(), large_vy.data(), large_vz.data(),
                          4, dt);

    for (SizeT i = 0; i < 4; ++i) {
        EXPECT_TRUE(nearly_equal(large_px[i], 1e10 + 1e5, 1.0));
    }
}

TEST_F(SIMDTest, SmallValues) {
    // Test with small values to check for underflow issues
    std::vector<Real> small_px = {1e-10, 1e-10, 1e-10, 1e-10};
    std::vector<Real> small_py = {1e-10, 1e-10, 1e-10, 1e-10};
    std::vector<Real> small_pz = {1e-10, 1e-10, 1e-10, 1e-10};
    std::vector<Real> small_vx = {1e-15, 1e-15, 1e-15, 1e-15};
    std::vector<Real> small_vy = {1e-15, 1e-15, 1e-15, 1e-15};
    std::vector<Real> small_vz = {1e-15, 1e-15, 1e-15, 1e-15};

    Real dt = 1.0;
    batch_position_update(small_px.data(), small_py.data(), small_pz.data(),
                          small_vx.data(), small_vy.data(), small_vz.data(),
                          4, dt);

    for (SizeT i = 0; i < 4; ++i) {
        EXPECT_TRUE(nearly_equal(small_px[i], 1e-10 + 1e-15, 1e-14));
    }
}
