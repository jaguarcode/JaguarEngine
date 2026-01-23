/**
 * @file physics_kernel_tests.cpp
 * @brief Comprehensive tests for GPU physics kernels
 *
 * Tests all physics kernel functionality across different compute backends:
 * - Integration kernels (positions, velocities, orientations)
 * - Collision detection kernels (broad-phase, AABB)
 * - Terrain sampling kernels
 * - Aerodynamic force kernels
 * - Utility kernels (gravity, damping, clamping)
 *
 * Test Categories:
 * 1. Unit tests for individual kernels
 * 2. Integration tests for kernel pipelines
 * 3. Performance benchmarks
 * 4. Backend comparison tests (CPU vs GPU results)
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "jaguar/gpu/physics_kernels.h"
#include "jaguar/gpu/compute_backend.h"  // BackendFactory is defined here
#include <cmath>
#include <vector>
#include <random>
#include <chrono>

using namespace jaguar::gpu;
using namespace testing;

// ============================================================================
// Test Fixture Base
// ============================================================================

/**
 * @brief Base test fixture for physics kernel tests
 */
class PhysicsKernelTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create best available backend
        m_backend = BackendFactory::create_best_available();
        ASSERT_NE(m_backend, nullptr) << "Failed to create compute backend";

        auto init_result = m_backend->initialize();
        ASSERT_EQ(init_result, BackendResult::Success)
            << "Failed to initialize backend: " << m_backend->last_error();

        // Create kernel manager
        m_kernels = std::make_unique<PhysicsKernelManager>(m_backend.get());

        PhysicsKernelConfig config;
        config.enable_profiling = true;
        config.preferred_workgroup_size = 256;

        auto kernel_init = m_kernels->initialize(config);
        ASSERT_EQ(kernel_init, BackendResult::Success)
            << "Failed to initialize kernel manager: " << m_kernels->last_error();
    }

    void TearDown() override {
        if (m_kernels) {
            m_kernels->shutdown();
            m_kernels.reset();
        }
        if (m_backend) {
            m_backend->shutdown();
            m_backend.reset();
        }
    }

    // Helper: Create random positions
    std::vector<float> create_random_positions(SizeT count, float range = 100.0f) {
        std::vector<float> positions(count * 3);
        std::mt19937 rng(42);  // Fixed seed for reproducibility
        std::uniform_real_distribution<float> dist(-range, range);
        for (size_t i = 0; i < positions.size(); ++i) {
            positions[i] = dist(rng);
        }
        return positions;
    }

    // Helper: Create random velocities
    std::vector<float> create_random_velocities(SizeT count, float max_speed = 10.0f) {
        std::vector<float> velocities(count * 3);
        std::mt19937 rng(123);
        std::uniform_real_distribution<float> dist(-max_speed, max_speed);
        for (size_t i = 0; i < velocities.size(); ++i) {
            velocities[i] = dist(rng);
        }
        return velocities;
    }

    // Helper: Create random masses
    std::vector<float> create_random_masses(SizeT count, float min_mass = 1.0f, float max_mass = 100.0f) {
        std::vector<float> masses(count);
        std::mt19937 rng(456);
        std::uniform_real_distribution<float> dist(min_mass, max_mass);
        for (size_t i = 0; i < masses.size(); ++i) {
            masses[i] = dist(rng);
        }
        return masses;
    }

    // Helper: Create identity quaternions
    std::vector<float> create_identity_quaternions(SizeT count) {
        std::vector<float> quats(count * 4);
        for (SizeT i = 0; i < count; ++i) {
            quats[i * 4 + 0] = 1.0f;  // w
            quats[i * 4 + 1] = 0.0f;  // x
            quats[i * 4 + 2] = 0.0f;  // y
            quats[i * 4 + 3] = 0.0f;  // z
        }
        return quats;
    }

    // Helper: Create random angular velocities
    std::vector<float> create_random_angular_velocities(SizeT count, float max_omega = 1.0f) {
        std::vector<float> omega(count * 3);
        std::mt19937 rng(789);
        std::uniform_real_distribution<float> dist(-max_omega, max_omega);
        for (size_t i = 0; i < omega.size(); ++i) {
            omega[i] = dist(rng);
        }
        return omega;
    }

    // Helper: Quaternion normalization check
    bool is_normalized_quaternion(float w, float x, float y, float z, float tolerance = 1e-5f) {
        float len_sq = w * w + x * x + y * y + z * z;
        return std::abs(len_sq - 1.0f) < tolerance;
    }

    std::unique_ptr<IComputeBackend> m_backend;
    std::unique_ptr<PhysicsKernelManager> m_kernels;
};

// ============================================================================
// Position Integration Tests
// ============================================================================

TEST_F(PhysicsKernelTest, IntegratePositions_ZeroVelocity) {
    const SizeT count = 100;
    const float dt = 0.016f;  // ~60 FPS

    // Create buffers
    auto positions = create_random_positions(count);
    std::vector<float> velocities(count * 3, 0.0f);  // Zero velocity

    auto pos_buffer = m_kernels->allocate_vec3_buffer(count);
    auto vel_buffer = m_kernels->allocate_vec3_buffer(count);
    ASSERT_NE(pos_buffer, INVALID_BUFFER_HANDLE);
    ASSERT_NE(vel_buffer, INVALID_BUFFER_HANDLE);

    // Upload data
    m_backend->write_buffer(pos_buffer, positions.data(), positions.size() * sizeof(float));
    m_backend->write_buffer(vel_buffer, velocities.data(), velocities.size() * sizeof(float));

    // Store original positions
    std::vector<float> original_positions = positions;

    // Run kernel
    auto result = m_kernels->integrate_positions(pos_buffer, vel_buffer, dt, count);
    ASSERT_EQ(result, BackendResult::Success);

    // Synchronize
    m_kernels->synchronize();

    // Read back
    m_backend->read_buffer(pos_buffer, positions.data(), positions.size() * sizeof(float));

    // Verify: positions should be unchanged
    for (SizeT i = 0; i < count * 3; ++i) {
        EXPECT_NEAR(positions[i], original_positions[i], 1e-6f)
            << "Position changed at index " << i << " with zero velocity";
    }

    // Cleanup
    m_kernels->free_buffer(pos_buffer);
    m_kernels->free_buffer(vel_buffer);
}

TEST_F(PhysicsKernelTest, IntegratePositions_ConstantVelocity) {
    const SizeT count = 100;
    const float dt = 0.1f;
    const float vx = 1.0f, vy = 2.0f, vz = 3.0f;

    // Create buffers with constant velocity
    auto positions = create_random_positions(count);
    std::vector<float> velocities(count * 3);
    for (SizeT i = 0; i < count; ++i) {
        velocities[i * 3 + 0] = vx;
        velocities[i * 3 + 1] = vy;
        velocities[i * 3 + 2] = vz;
    }

    auto pos_buffer = m_kernels->allocate_vec3_buffer(count);
    auto vel_buffer = m_kernels->allocate_vec3_buffer(count);

    m_backend->write_buffer(pos_buffer, positions.data(), positions.size() * sizeof(float));
    m_backend->write_buffer(vel_buffer, velocities.data(), velocities.size() * sizeof(float));

    std::vector<float> original_positions = positions;

    // Run kernel
    auto result = m_kernels->integrate_positions(pos_buffer, vel_buffer, dt, count);
    ASSERT_EQ(result, BackendResult::Success);
    m_kernels->synchronize();

    // Read back
    m_backend->read_buffer(pos_buffer, positions.data(), positions.size() * sizeof(float));

    // Verify: positions should move by velocity * dt
    for (SizeT i = 0; i < count; ++i) {
        EXPECT_NEAR(positions[i * 3 + 0], original_positions[i * 3 + 0] + vx * dt, 1e-5f);
        EXPECT_NEAR(positions[i * 3 + 1], original_positions[i * 3 + 1] + vy * dt, 1e-5f);
        EXPECT_NEAR(positions[i * 3 + 2], original_positions[i * 3 + 2] + vz * dt, 1e-5f);
    }

    m_kernels->free_buffer(pos_buffer);
    m_kernels->free_buffer(vel_buffer);
}

TEST_F(PhysicsKernelTest, IntegratePositions_LargeCount) {
    const SizeT count = 100000;  // 100K entities
    const float dt = 0.016f;

    auto positions = create_random_positions(count);
    auto velocities = create_random_velocities(count);

    auto pos_buffer = m_kernels->allocate_vec3_buffer(count);
    auto vel_buffer = m_kernels->allocate_vec3_buffer(count);

    m_backend->write_buffer(pos_buffer, positions.data(), positions.size() * sizeof(float));
    m_backend->write_buffer(vel_buffer, velocities.data(), velocities.size() * sizeof(float));

    std::vector<float> original_positions = positions;

    // Run kernel
    auto start = std::chrono::high_resolution_clock::now();
    auto result = m_kernels->integrate_positions(pos_buffer, vel_buffer, dt, count);
    m_kernels->synchronize();
    auto end = std::chrono::high_resolution_clock::now();

    ASSERT_EQ(result, BackendResult::Success);

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "  [PERF] Integration of " << count << " entities: "
              << duration.count() << " us" << std::endl;

    // Read back and verify a sample
    m_backend->read_buffer(pos_buffer, positions.data(), positions.size() * sizeof(float));

    for (SizeT i = 0; i < std::min(count, SizeT(100)); ++i) {
        for (int j = 0; j < 3; ++j) {
            float expected = original_positions[i * 3 + j] + velocities[i * 3 + j] * dt;
            EXPECT_NEAR(positions[i * 3 + j], expected, 1e-4f);
        }
    }

    m_kernels->free_buffer(pos_buffer);
    m_kernels->free_buffer(vel_buffer);
}

// ============================================================================
// Velocity Integration Tests
// ============================================================================

TEST_F(PhysicsKernelTest, IntegrateVelocities_ZeroForce) {
    const SizeT count = 100;
    const float dt = 0.016f;

    auto velocities = create_random_velocities(count);
    std::vector<float> forces(count * 3, 0.0f);
    auto masses = create_random_masses(count);

    auto vel_buffer = m_kernels->allocate_vec3_buffer(count);
    auto force_buffer = m_kernels->allocate_vec3_buffer(count);
    auto mass_buffer = m_kernels->allocate_scalar_buffer(count);

    m_backend->write_buffer(vel_buffer, velocities.data(), velocities.size() * sizeof(float));
    m_backend->write_buffer(force_buffer, forces.data(), forces.size() * sizeof(float));
    m_backend->write_buffer(mass_buffer, masses.data(), masses.size() * sizeof(float));

    std::vector<float> original_velocities = velocities;

    // With zero force and gravity, velocity should change only due to gravity
    auto result = m_kernels->integrate_velocities(vel_buffer, force_buffer, mass_buffer, dt, count);
    ASSERT_EQ(result, BackendResult::Success);
    m_kernels->synchronize();

    m_backend->read_buffer(vel_buffer, velocities.data(), velocities.size() * sizeof(float));

    // Default gravity is (0, 0, -9.81)
    const float gravity_z = -9.81f;
    for (SizeT i = 0; i < count; ++i) {
        EXPECT_NEAR(velocities[i * 3 + 0], original_velocities[i * 3 + 0], 1e-5f);  // X unchanged
        EXPECT_NEAR(velocities[i * 3 + 1], original_velocities[i * 3 + 1], 1e-5f);  // Y unchanged
        EXPECT_NEAR(velocities[i * 3 + 2], original_velocities[i * 3 + 2] + gravity_z * dt, 1e-4f);  // Z changed by gravity
    }

    m_kernels->free_buffer(vel_buffer);
    m_kernels->free_buffer(force_buffer);
    m_kernels->free_buffer(mass_buffer);
}

TEST_F(PhysicsKernelTest, IntegrateVelocities_WithForce) {
    const SizeT count = 10;
    const float dt = 0.1f;

    std::vector<float> velocities(count * 3, 0.0f);
    std::vector<float> forces(count * 3);
    std::vector<float> masses(count);

    // Constant force and mass
    const float fx = 10.0f, fy = 0.0f, fz = 0.0f;
    const float mass = 2.0f;

    for (SizeT i = 0; i < count; ++i) {
        forces[i * 3 + 0] = fx;
        forces[i * 3 + 1] = fy;
        forces[i * 3 + 2] = fz;
        masses[i] = mass;
    }

    auto vel_buffer = m_kernels->allocate_vec3_buffer(count);
    auto force_buffer = m_kernels->allocate_vec3_buffer(count);
    auto mass_buffer = m_kernels->allocate_scalar_buffer(count);

    m_backend->write_buffer(vel_buffer, velocities.data(), velocities.size() * sizeof(float));
    m_backend->write_buffer(force_buffer, forces.data(), forces.size() * sizeof(float));
    m_backend->write_buffer(mass_buffer, masses.data(), masses.size() * sizeof(float));

    auto result = m_kernels->integrate_velocities(vel_buffer, force_buffer, mass_buffer, dt, count);
    ASSERT_EQ(result, BackendResult::Success);
    m_kernels->synchronize();

    m_backend->read_buffer(vel_buffer, velocities.data(), velocities.size() * sizeof(float));

    // Expected: v += (F/m + g) * dt
    // vx = (10/2 + 0) * 0.1 = 0.5
    const float expected_vx = (fx / mass) * dt;
    const float gravity_z = -9.81f;

    for (SizeT i = 0; i < count; ++i) {
        EXPECT_NEAR(velocities[i * 3 + 0], expected_vx, 1e-5f);
        EXPECT_NEAR(velocities[i * 3 + 1], 0.0f, 1e-5f);
        EXPECT_NEAR(velocities[i * 3 + 2], gravity_z * dt, 1e-4f);
    }

    m_kernels->free_buffer(vel_buffer);
    m_kernels->free_buffer(force_buffer);
    m_kernels->free_buffer(mass_buffer);
}

// ============================================================================
// Orientation Integration Tests
// ============================================================================

TEST_F(PhysicsKernelTest, IntegrateOrientations_ZeroAngularVelocity) {
    const SizeT count = 100;
    const float dt = 0.016f;

    auto orientations = create_identity_quaternions(count);
    std::vector<float> angular_velocities(count * 3, 0.0f);

    auto orient_buffer = m_kernels->allocate_vec4_buffer(count);
    auto omega_buffer = m_kernels->allocate_vec3_buffer(count);

    m_backend->write_buffer(orient_buffer, orientations.data(), orientations.size() * sizeof(float));
    m_backend->write_buffer(omega_buffer, angular_velocities.data(), angular_velocities.size() * sizeof(float));

    auto result = m_kernels->integrate_orientations(orient_buffer, omega_buffer, dt, count);
    ASSERT_EQ(result, BackendResult::Success);
    m_kernels->synchronize();

    m_backend->read_buffer(orient_buffer, orientations.data(), orientations.size() * sizeof(float));

    // With zero angular velocity, quaternion should remain identity
    for (SizeT i = 0; i < count; ++i) {
        EXPECT_NEAR(orientations[i * 4 + 0], 1.0f, 1e-5f);  // w
        EXPECT_NEAR(orientations[i * 4 + 1], 0.0f, 1e-5f);  // x
        EXPECT_NEAR(orientations[i * 4 + 2], 0.0f, 1e-5f);  // y
        EXPECT_NEAR(orientations[i * 4 + 3], 0.0f, 1e-5f);  // z
    }

    m_kernels->free_buffer(orient_buffer);
    m_kernels->free_buffer(omega_buffer);
}

TEST_F(PhysicsKernelTest, IntegrateOrientations_ConstantRotation) {
    const SizeT count = 10;
    const float dt = 0.1f;

    auto orientations = create_identity_quaternions(count);
    std::vector<float> angular_velocities(count * 3);

    // Rotate around Z axis at 1 rad/s
    const float omega_z = 1.0f;
    for (SizeT i = 0; i < count; ++i) {
        angular_velocities[i * 3 + 0] = 0.0f;
        angular_velocities[i * 3 + 1] = 0.0f;
        angular_velocities[i * 3 + 2] = omega_z;
    }

    auto orient_buffer = m_kernels->allocate_vec4_buffer(count);
    auto omega_buffer = m_kernels->allocate_vec3_buffer(count);

    m_backend->write_buffer(orient_buffer, orientations.data(), orientations.size() * sizeof(float));
    m_backend->write_buffer(omega_buffer, angular_velocities.data(), angular_velocities.size() * sizeof(float));

    auto result = m_kernels->integrate_orientations(orient_buffer, omega_buffer, dt, count);
    ASSERT_EQ(result, BackendResult::Success);
    m_kernels->synchronize();

    m_backend->read_buffer(orient_buffer, orientations.data(), orientations.size() * sizeof(float));

    // Verify quaternions are normalized and represent rotation around Z
    for (SizeT i = 0; i < count; ++i) {
        float w = orientations[i * 4 + 0];
        float x = orientations[i * 4 + 1];
        float y = orientations[i * 4 + 2];
        float z = orientations[i * 4 + 3];

        EXPECT_TRUE(is_normalized_quaternion(w, x, y, z, 1e-4f))
            << "Quaternion not normalized at index " << i;

        // X and Y components should be near zero (rotation around Z)
        EXPECT_NEAR(x, 0.0f, 1e-4f);
        EXPECT_NEAR(y, 0.0f, 1e-4f);
    }

    m_kernels->free_buffer(orient_buffer);
    m_kernels->free_buffer(omega_buffer);
}

// ============================================================================
// AABB Update Tests
// ============================================================================

TEST_F(PhysicsKernelTest, UpdateAABBs_Basic) {
    const SizeT count = 100;

    std::vector<float> positions(count * 3);
    std::vector<float> extents(count * 3);

    // Create centered positions with varying extents
    for (SizeT i = 0; i < count; ++i) {
        positions[i * 3 + 0] = static_cast<float>(i * 10);  // X
        positions[i * 3 + 1] = 0.0f;                        // Y
        positions[i * 3 + 2] = 0.0f;                        // Z

        extents[i * 3 + 0] = 1.0f;  // Half-extent X
        extents[i * 3 + 1] = 1.0f;  // Half-extent Y
        extents[i * 3 + 2] = 1.0f;  // Half-extent Z
    }

    auto pos_buffer = m_kernels->allocate_vec3_buffer(count);
    auto ext_buffer = m_kernels->allocate_vec3_buffer(count);
    auto aabb_buffer = m_kernels->allocate_aabb_buffer(count);

    m_backend->write_buffer(pos_buffer, positions.data(), positions.size() * sizeof(float));
    m_backend->write_buffer(ext_buffer, extents.data(), extents.size() * sizeof(float));

    auto result = m_kernels->update_aabbs(pos_buffer, ext_buffer, aabb_buffer, count);
    ASSERT_EQ(result, BackendResult::Success);
    m_kernels->synchronize();

    // Read back AABBs
    std::vector<AABB> aabbs(count);
    m_backend->read_buffer(aabb_buffer, aabbs.data(), count * sizeof(AABB));

    // Verify AABBs
    for (SizeT i = 0; i < count; ++i) {
        float cx = positions[i * 3 + 0];
        float cy = positions[i * 3 + 1];
        float cz = positions[i * 3 + 2];
        float ex = extents[i * 3 + 0];
        float ey = extents[i * 3 + 1];
        float ez = extents[i * 3 + 2];

        EXPECT_NEAR(aabbs[i].min_x, cx - ex, 1e-5f);
        EXPECT_NEAR(aabbs[i].min_y, cy - ey, 1e-5f);
        EXPECT_NEAR(aabbs[i].min_z, cz - ez, 1e-5f);
        EXPECT_NEAR(aabbs[i].max_x, cx + ex, 1e-5f);
        EXPECT_NEAR(aabbs[i].max_y, cy + ey, 1e-5f);
        EXPECT_NEAR(aabbs[i].max_z, cz + ez, 1e-5f);
        EXPECT_EQ(aabbs[i].entity_id, static_cast<UInt32>(i));
    }

    m_kernels->free_buffer(pos_buffer);
    m_kernels->free_buffer(ext_buffer);
    m_kernels->free_buffer(aabb_buffer);
}

// ============================================================================
// Collision Broad-Phase Tests
// ============================================================================

TEST_F(PhysicsKernelTest, CollisionBroadPhase_NoOverlap) {
    const SizeT count = 10;

    // Create AABBs that don't overlap (spaced 100 units apart)
    std::vector<AABB> aabbs(count);
    for (SizeT i = 0; i < count; ++i) {
        float x = static_cast<float>(i * 100);
        aabbs[i] = AABB::from_center_extents(x, 0, 0, 1, 1, 1, static_cast<UInt32>(i));
    }

    auto aabb_buffer = m_kernels->allocate_aabb_buffer(count);
    auto pair_buffer = m_kernels->allocate_collision_pair_buffer(1000);

    m_backend->write_buffer(aabb_buffer, aabbs.data(), count * sizeof(AABB));

    SizeT pair_count = 0;
    auto result = m_kernels->collision_broad_phase(aabb_buffer, count, pair_buffer, &pair_count);
    ASSERT_EQ(result, BackendResult::Success);
    m_kernels->synchronize();

    // No overlaps expected
    EXPECT_EQ(pair_count, 0) << "Expected no collision pairs for non-overlapping AABBs";

    m_kernels->free_buffer(aabb_buffer);
    m_kernels->free_buffer(pair_buffer);
}

TEST_F(PhysicsKernelTest, CollisionBroadPhase_AllOverlap) {
    const SizeT count = 5;

    // Create AABBs that all overlap at origin
    std::vector<AABB> aabbs(count);
    for (SizeT i = 0; i < count; ++i) {
        aabbs[i] = AABB::from_center_extents(0, 0, 0, 2, 2, 2, static_cast<UInt32>(i));
    }

    auto aabb_buffer = m_kernels->allocate_aabb_buffer(count);
    auto pair_buffer = m_kernels->allocate_collision_pair_buffer(1000);

    m_backend->write_buffer(aabb_buffer, aabbs.data(), count * sizeof(AABB));

    SizeT pair_count = 0;
    auto result = m_kernels->collision_broad_phase(aabb_buffer, count, pair_buffer, &pair_count);
    ASSERT_EQ(result, BackendResult::Success);
    m_kernels->synchronize();

    // n*(n-1)/2 pairs expected
    SizeT expected_pairs = count * (count - 1) / 2;
    EXPECT_EQ(pair_count, expected_pairs)
        << "Expected " << expected_pairs << " collision pairs for all-overlapping AABBs";

    m_kernels->free_buffer(aabb_buffer);
    m_kernels->free_buffer(pair_buffer);
}

TEST_F(PhysicsKernelTest, CollisionBroadPhase_SomeOverlap) {
    // Create 3 AABBs: A overlaps B, B overlaps C, A doesn't overlap C
    std::vector<AABB> aabbs(3);
    aabbs[0] = AABB::from_center_extents(0, 0, 0, 1, 1, 1, 0);    // A
    aabbs[1] = AABB::from_center_extents(1.5f, 0, 0, 1, 1, 1, 1); // B (overlaps A)
    aabbs[2] = AABB::from_center_extents(3, 0, 0, 1, 1, 1, 2);    // C (overlaps B, not A)

    auto aabb_buffer = m_kernels->allocate_aabb_buffer(3);
    auto pair_buffer = m_kernels->allocate_collision_pair_buffer(100);

    m_backend->write_buffer(aabb_buffer, aabbs.data(), 3 * sizeof(AABB));

    SizeT pair_count = 0;
    auto result = m_kernels->collision_broad_phase(aabb_buffer, 3, pair_buffer, &pair_count);
    ASSERT_EQ(result, BackendResult::Success);
    m_kernels->synchronize();

    // Expected pairs: (A,B) and (B,C) = 2 pairs
    EXPECT_EQ(pair_count, 2) << "Expected 2 collision pairs";

    // Read back pairs and verify
    std::vector<CollisionPair> pairs(pair_count);
    m_backend->read_buffer(pair_buffer, pairs.data(), pair_count * sizeof(CollisionPair));

    // Verify A-C are NOT paired
    for (SizeT i = 0; i < pair_count; ++i) {
        bool is_ac = (pairs[i].entity_a == 0 && pairs[i].entity_b == 2) ||
                     (pairs[i].entity_a == 2 && pairs[i].entity_b == 0);
        EXPECT_FALSE(is_ac) << "Unexpected pair (A,C) found";
    }

    m_kernels->free_buffer(aabb_buffer);
    m_kernels->free_buffer(pair_buffer);
}

// ============================================================================
// Terrain Sampling Tests
// ============================================================================

TEST_F(PhysicsKernelTest, TerrainSampling_FlatTerrain) {
    const UInt32 width = 128;
    const UInt32 height = 128;
    const float terrain_height = 10.0f;

    // Create flat heightmap
    std::vector<float> heightmap(width * height, terrain_height);

    auto hmap_buffer = m_backend->allocate_buffer(
        heightmap.size() * sizeof(float),
        MemoryAccess::ReadWrite
    );
    m_backend->write_buffer(hmap_buffer, heightmap.data(), heightmap.size() * sizeof(float));

    auto result = m_kernels->set_terrain_heightmap(hmap_buffer, width, height, 0.0f, 0.0f, 1.0f);
    ASSERT_EQ(result, BackendResult::Success);

    // Create sample requests
    const SizeT sample_count = 100;
    std::vector<TerrainSampleRequest> requests(sample_count);
    std::mt19937 rng(42);
    std::uniform_real_distribution<float> dist(0.0f, static_cast<float>(width - 1));

    for (SizeT i = 0; i < sample_count; ++i) {
        requests[i].x = dist(rng);
        requests[i].y = dist(rng);
        requests[i].z = 0.0f;
        requests[i].entity_id = static_cast<UInt32>(i);
    }

    auto req_buffer = m_backend->allocate_buffer(
        sample_count * sizeof(TerrainSampleRequest),
        MemoryAccess::ReadOnly
    );
    auto res_buffer = m_backend->allocate_buffer(
        sample_count * sizeof(TerrainSampleResult),
        MemoryAccess::WriteOnly
    );

    m_backend->write_buffer(req_buffer, requests.data(), sample_count * sizeof(TerrainSampleRequest));

    result = m_kernels->sample_terrain_batch(req_buffer, res_buffer, sample_count);
    ASSERT_EQ(result, BackendResult::Success);
    m_kernels->synchronize();

    // Read results
    std::vector<TerrainSampleResult> results(sample_count);
    m_backend->read_buffer(res_buffer, results.data(), sample_count * sizeof(TerrainSampleResult));

    // Verify flat terrain results
    for (SizeT i = 0; i < sample_count; ++i) {
        EXPECT_NEAR(results[i].height, terrain_height, 1e-4f)
            << "Wrong height at sample " << i;
        EXPECT_NEAR(results[i].normal_y, 1.0f, 1e-4f)
            << "Normal should be up for flat terrain";
        EXPECT_EQ(results[i].entity_id, static_cast<UInt32>(i));
    }

    m_backend->free_buffer(hmap_buffer);
    m_backend->free_buffer(req_buffer);
    m_backend->free_buffer(res_buffer);
}

// ============================================================================
// Aerodynamic Force Tests
// ============================================================================

TEST_F(PhysicsKernelTest, AeroForces_StaticAircraft) {
    const SizeT count = 10;

    // Create static aircraft inputs (no velocity)
    std::vector<AeroInput> inputs(count);
    for (SizeT i = 0; i < count; ++i) {
        inputs[i] = {};
        inputs[i].pos_x = 0;
        inputs[i].pos_y = 0;
        inputs[i].pos_z = 1000.0f;  // Altitude
        inputs[i].vel_x = 0;
        inputs[i].vel_y = 0;
        inputs[i].vel_z = 0;
        inputs[i].quat_w = 1.0f;
        inputs[i].quat_x = 0;
        inputs[i].quat_y = 0;
        inputs[i].quat_z = 0;
        inputs[i].air_density = 1.225f;
        inputs[i].speed_of_sound = 343.0f;
        inputs[i].entity_id = static_cast<UInt32>(i);
    }

    auto input_buffer = m_backend->allocate_buffer(
        count * sizeof(AeroInput),
        MemoryAccess::ReadOnly
    );
    auto output_buffer = m_backend->allocate_buffer(
        count * sizeof(AeroOutput),
        MemoryAccess::WriteOnly
    );

    m_backend->write_buffer(input_buffer, inputs.data(), count * sizeof(AeroInput));

    auto result = m_kernels->compute_aero_forces(input_buffer, output_buffer, count);
    ASSERT_EQ(result, BackendResult::Success);
    m_kernels->synchronize();

    // Read results
    std::vector<AeroOutput> outputs(count);
    m_backend->read_buffer(output_buffer, outputs.data(), count * sizeof(AeroOutput));

    // Static aircraft should have zero/minimal forces
    for (SizeT i = 0; i < count; ++i) {
        // With zero velocity, dynamic pressure should be zero
        EXPECT_NEAR(outputs[i].dynamic_pressure, 0.0f, 1e-4f);
        EXPECT_EQ(outputs[i].entity_id, static_cast<UInt32>(i));
    }

    m_backend->free_buffer(input_buffer);
    m_backend->free_buffer(output_buffer);
}

TEST_F(PhysicsKernelTest, AeroForces_ForwardFlight) {
    const SizeT count = 1;
    const float velocity = 100.0f;  // 100 m/s forward

    AeroInput input = {};
    input.pos_z = 1000.0f;
    input.vel_x = velocity;
    input.vel_y = 0;
    input.vel_z = 0;
    input.quat_w = 1.0f;
    input.air_density = 1.225f;
    input.speed_of_sound = 343.0f;
    input.entity_id = 0;

    auto input_buffer = m_backend->allocate_buffer(sizeof(AeroInput), MemoryAccess::ReadOnly);
    auto output_buffer = m_backend->allocate_buffer(sizeof(AeroOutput), MemoryAccess::WriteOnly);

    m_backend->write_buffer(input_buffer, &input, sizeof(AeroInput));

    auto result = m_kernels->compute_aero_forces(input_buffer, output_buffer, count);
    ASSERT_EQ(result, BackendResult::Success);
    m_kernels->synchronize();

    AeroOutput output;
    m_backend->read_buffer(output_buffer, &output, sizeof(AeroOutput));

    // Dynamic pressure = 0.5 * rho * V^2
    float expected_q = 0.5f * 1.225f * velocity * velocity;
    EXPECT_NEAR(output.dynamic_pressure, expected_q, expected_q * 0.01f);

    // Mach number = V / speed_of_sound
    float expected_mach = velocity / 343.0f;
    EXPECT_NEAR(output.mach, expected_mach, 0.01f);

    m_backend->free_buffer(input_buffer);
    m_backend->free_buffer(output_buffer);
}

// ============================================================================
// Utility Kernel Tests
// ============================================================================

TEST_F(PhysicsKernelTest, ApplyGravity_Basic) {
    const SizeT count = 100;

    std::vector<float> forces(count * 3, 0.0f);  // Zero initial forces
    auto masses = create_random_masses(count);

    auto force_buffer = m_kernels->allocate_vec3_buffer(count);
    auto mass_buffer = m_kernels->allocate_scalar_buffer(count);

    m_backend->write_buffer(force_buffer, forces.data(), forces.size() * sizeof(float));
    m_backend->write_buffer(mass_buffer, masses.data(), masses.size() * sizeof(float));

    auto result = m_kernels->apply_gravity(force_buffer, mass_buffer, count);
    ASSERT_EQ(result, BackendResult::Success);
    m_kernels->synchronize();

    m_backend->read_buffer(force_buffer, forces.data(), forces.size() * sizeof(float));

    // Verify: F = m * g, where g = (0, 0, -9.81)
    const float gravity_z = -9.81f;
    for (SizeT i = 0; i < count; ++i) {
        EXPECT_NEAR(forces[i * 3 + 0], 0.0f, 1e-5f);  // X force
        EXPECT_NEAR(forces[i * 3 + 1], 0.0f, 1e-5f);  // Y force
        EXPECT_NEAR(forces[i * 3 + 2], masses[i] * gravity_z, 1e-3f);  // Z force
    }

    m_kernels->free_buffer(force_buffer);
    m_kernels->free_buffer(mass_buffer);
}

TEST_F(PhysicsKernelTest, ApplyDamping_Basic) {
    const SizeT count = 100;
    const float damping = 0.1f;
    const float dt = 0.016f;

    auto velocities = create_random_velocities(count);
    std::vector<float> original_velocities = velocities;

    auto vel_buffer = m_kernels->allocate_vec3_buffer(count);
    m_backend->write_buffer(vel_buffer, velocities.data(), velocities.size() * sizeof(float));

    auto result = m_kernels->apply_damping(vel_buffer, damping, dt, count);
    ASSERT_EQ(result, BackendResult::Success);
    m_kernels->synchronize();

    m_backend->read_buffer(vel_buffer, velocities.data(), velocities.size() * sizeof(float));

    // Verify: v *= (1 - damping * dt)
    float factor = 1.0f - damping * dt;
    for (SizeT i = 0; i < count * 3; ++i) {
        EXPECT_NEAR(velocities[i], original_velocities[i] * factor, 1e-5f);
    }

    m_kernels->free_buffer(vel_buffer);
}

TEST_F(PhysicsKernelTest, ClampVelocities_Basic) {
    const SizeT count = 100;
    const float max_speed = 10.0f;

    // Create velocities with some exceeding max speed
    auto velocities = create_random_velocities(count, 50.0f);  // Max 50 m/s

    auto vel_buffer = m_kernels->allocate_vec3_buffer(count);
    m_backend->write_buffer(vel_buffer, velocities.data(), velocities.size() * sizeof(float));

    auto result = m_kernels->clamp_velocities(vel_buffer, max_speed, count);
    ASSERT_EQ(result, BackendResult::Success);
    m_kernels->synchronize();

    m_backend->read_buffer(vel_buffer, velocities.data(), velocities.size() * sizeof(float));

    // Verify: all velocities should be <= max_speed
    for (SizeT i = 0; i < count; ++i) {
        float vx = velocities[i * 3 + 0];
        float vy = velocities[i * 3 + 1];
        float vz = velocities[i * 3 + 2];
        float speed = std::sqrt(vx * vx + vy * vy + vz * vz);

        EXPECT_LE(speed, max_speed + 1e-4f)
            << "Speed " << speed << " exceeds max " << max_speed << " at entity " << i;
    }

    m_kernels->free_buffer(vel_buffer);
}

// ============================================================================
// Symplectic Integration Tests
// ============================================================================

TEST_F(PhysicsKernelTest, SymplecticIntegration_FreeFall) {
    const SizeT count = 1;
    const float dt = 0.001f;  // 1ms time step
    const int steps = 1000;   // 1 second total

    // Create rigid body state buffers
    RigidBodyStateBuffers state;
    state.count = count;
    state.positions = m_kernels->allocate_vec3_buffer(count);
    state.velocities = m_kernels->allocate_vec3_buffer(count);
    state.orientations = m_kernels->allocate_vec4_buffer(count);
    state.angular_vels = m_kernels->allocate_vec3_buffer(count);
    state.forces = m_kernels->allocate_vec3_buffer(count);
    state.torques = m_kernels->allocate_vec3_buffer(count);
    state.masses = m_kernels->allocate_scalar_buffer(count);
    state.inv_inertia = m_kernels->allocate_vec3_buffer(count);

    // Initialize: Start at origin with zero velocity
    std::vector<float> pos = {0.0f, 0.0f, 100.0f};  // Start at 100m height
    std::vector<float> vel = {0.0f, 0.0f, 0.0f};
    std::vector<float> orient = {1.0f, 0.0f, 0.0f, 0.0f};
    std::vector<float> ang_vel = {0.0f, 0.0f, 0.0f};
    std::vector<float> forces = {0.0f, 0.0f, 0.0f};
    std::vector<float> torques = {0.0f, 0.0f, 0.0f};
    std::vector<float> masses = {1.0f};
    std::vector<float> inv_inertia = {1.0f, 1.0f, 1.0f};

    m_backend->write_buffer(state.positions, pos.data(), pos.size() * sizeof(float));
    m_backend->write_buffer(state.velocities, vel.data(), vel.size() * sizeof(float));
    m_backend->write_buffer(state.orientations, orient.data(), orient.size() * sizeof(float));
    m_backend->write_buffer(state.angular_vels, ang_vel.data(), ang_vel.size() * sizeof(float));
    m_backend->write_buffer(state.forces, forces.data(), forces.size() * sizeof(float));
    m_backend->write_buffer(state.torques, torques.data(), torques.size() * sizeof(float));
    m_backend->write_buffer(state.masses, masses.data(), masses.size() * sizeof(float));
    m_backend->write_buffer(state.inv_inertia, inv_inertia.data(), inv_inertia.size() * sizeof(float));

    // Run symplectic integration for 1 second
    for (int i = 0; i < steps; ++i) {
        // Apply gravity
        m_kernels->apply_gravity(state.forces, state.masses, count);

        // Integrate
        auto result = m_kernels->integrate_symplectic(state, dt);
        ASSERT_EQ(result, BackendResult::Success);

        // Clear forces for next step
        m_kernels->clear_forces(state.forces, state.torques, count);
    }

    m_kernels->synchronize();

    // Read final state
    m_backend->read_buffer(state.positions, pos.data(), pos.size() * sizeof(float));
    m_backend->read_buffer(state.velocities, vel.data(), vel.size() * sizeof(float));

    // After 1 second of free fall:
    // v = g * t = -9.81 * 1 = -9.81 m/s
    // z = z0 + 0.5 * g * t^2 = 100 - 0.5 * 9.81 * 1 = 95.095 m
    const float gravity = -9.81f;
    const float t = steps * dt;

    EXPECT_NEAR(vel[2], gravity * t, 0.1f)
        << "Expected velocity " << gravity * t << " after " << t << " seconds";

    float expected_z = 100.0f + 0.5f * gravity * t * t;
    EXPECT_NEAR(pos[2], expected_z, 0.5f)
        << "Expected height " << expected_z << " after " << t << " seconds";

    // Cleanup
    m_kernels->free_buffer(state.positions);
    m_kernels->free_buffer(state.velocities);
    m_kernels->free_buffer(state.orientations);
    m_kernels->free_buffer(state.angular_vels);
    m_kernels->free_buffer(state.forces);
    m_kernels->free_buffer(state.torques);
    m_kernels->free_buffer(state.masses);
    m_kernels->free_buffer(state.inv_inertia);
}

// ============================================================================
// Performance Benchmark Tests
// ============================================================================

class PhysicsKernelBenchmark : public PhysicsKernelTest {
protected:
    void RunBenchmark(const std::string& name, std::function<void()> kernel_call, int iterations = 100) {
        // Warmup
        for (int i = 0; i < 10; ++i) {
            kernel_call();
        }
        m_kernels->synchronize();

        // Benchmark
        auto start = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < iterations; ++i) {
            kernel_call();
        }
        m_kernels->synchronize();
        auto end = std::chrono::high_resolution_clock::now();

        auto total_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        double avg_us = static_cast<double>(total_us) / iterations;

        std::cout << "  [BENCHMARK] " << name << ": "
                  << avg_us << " us/call (" << iterations << " iterations)" << std::endl;
    }
};

TEST_F(PhysicsKernelBenchmark, Benchmark_IntegrationKernels) {
    const SizeT count = 10000;
    const float dt = 0.016f;

    auto positions = create_random_positions(count);
    auto velocities = create_random_velocities(count);
    auto orientations = create_identity_quaternions(count);
    auto angular_vels = create_random_angular_velocities(count);

    auto pos_buffer = m_kernels->allocate_vec3_buffer(count);
    auto vel_buffer = m_kernels->allocate_vec3_buffer(count);
    auto orient_buffer = m_kernels->allocate_vec4_buffer(count);
    auto omega_buffer = m_kernels->allocate_vec3_buffer(count);

    m_backend->write_buffer(pos_buffer, positions.data(), positions.size() * sizeof(float));
    m_backend->write_buffer(vel_buffer, velocities.data(), velocities.size() * sizeof(float));
    m_backend->write_buffer(orient_buffer, orientations.data(), orientations.size() * sizeof(float));
    m_backend->write_buffer(omega_buffer, angular_vels.data(), angular_vels.size() * sizeof(float));

    std::cout << "\n  Integration Benchmarks (" << count << " entities):" << std::endl;

    RunBenchmark("integrate_positions", [&]() {
        m_kernels->integrate_positions(pos_buffer, vel_buffer, dt, count);
    });

    RunBenchmark("integrate_orientations", [&]() {
        m_kernels->integrate_orientations(orient_buffer, omega_buffer, dt, count);
    });

    m_kernels->free_buffer(pos_buffer);
    m_kernels->free_buffer(vel_buffer);
    m_kernels->free_buffer(orient_buffer);
    m_kernels->free_buffer(omega_buffer);
}

TEST_F(PhysicsKernelBenchmark, Benchmark_CollisionBroadPhase) {
    const std::vector<SizeT> counts = {100, 1000, 5000};

    std::cout << "\n  Collision Broad-Phase Benchmarks:" << std::endl;

    for (SizeT count : counts) {
        // Create random AABBs with some clustering
        std::vector<AABB> aabbs(count);
        std::mt19937 rng(42);
        std::uniform_real_distribution<float> pos_dist(-100.0f, 100.0f);
        std::uniform_real_distribution<float> size_dist(0.5f, 2.0f);

        for (SizeT i = 0; i < count; ++i) {
            float size = size_dist(rng);
            aabbs[i] = AABB::from_center_extents(
                pos_dist(rng), pos_dist(rng), pos_dist(rng),
                size, size, size,
                static_cast<UInt32>(i)
            );
        }

        auto aabb_buffer = m_kernels->allocate_aabb_buffer(count);
        auto pair_buffer = m_kernels->allocate_collision_pair_buffer(count * 10);

        m_backend->write_buffer(aabb_buffer, aabbs.data(), count * sizeof(AABB));

        SizeT pair_count = 0;
        RunBenchmark("broad_phase_" + std::to_string(count), [&]() {
            m_kernels->collision_broad_phase(aabb_buffer, count, pair_buffer, &pair_count);
        }, 50);

        m_kernels->free_buffer(aabb_buffer);
        m_kernels->free_buffer(pair_buffer);
    }
}

// ============================================================================
// Edge Case Tests
// ============================================================================

TEST_F(PhysicsKernelTest, EdgeCase_ZeroCount) {
    // All kernels should handle zero count gracefully
    auto result = m_kernels->integrate_positions(INVALID_BUFFER_HANDLE, INVALID_BUFFER_HANDLE, 0.016f, 0);
    EXPECT_EQ(result, BackendResult::Success);
}

TEST_F(PhysicsKernelTest, EdgeCase_SingleEntity) {
    const SizeT count = 1;
    const float dt = 0.016f;

    std::vector<float> pos = {1.0f, 2.0f, 3.0f};
    std::vector<float> vel = {10.0f, 20.0f, 30.0f};

    auto pos_buffer = m_kernels->allocate_vec3_buffer(count);
    auto vel_buffer = m_kernels->allocate_vec3_buffer(count);

    m_backend->write_buffer(pos_buffer, pos.data(), pos.size() * sizeof(float));
    m_backend->write_buffer(vel_buffer, vel.data(), vel.size() * sizeof(float));

    auto result = m_kernels->integrate_positions(pos_buffer, vel_buffer, dt, count);
    ASSERT_EQ(result, BackendResult::Success);
    m_kernels->synchronize();

    m_backend->read_buffer(pos_buffer, pos.data(), pos.size() * sizeof(float));

    EXPECT_NEAR(pos[0], 1.0f + 10.0f * dt, 1e-5f);
    EXPECT_NEAR(pos[1], 2.0f + 20.0f * dt, 1e-5f);
    EXPECT_NEAR(pos[2], 3.0f + 30.0f * dt, 1e-5f);

    m_kernels->free_buffer(pos_buffer);
    m_kernels->free_buffer(vel_buffer);
}

TEST_F(PhysicsKernelTest, EdgeCase_LargeDeltaTime) {
    const SizeT count = 10;
    const float dt = 1.0f;  // 1 second time step

    auto positions = create_random_positions(count);
    auto velocities = create_random_velocities(count);

    auto pos_buffer = m_kernels->allocate_vec3_buffer(count);
    auto vel_buffer = m_kernels->allocate_vec3_buffer(count);

    m_backend->write_buffer(pos_buffer, positions.data(), positions.size() * sizeof(float));
    m_backend->write_buffer(vel_buffer, velocities.data(), velocities.size() * sizeof(float));

    std::vector<float> original_positions = positions;

    auto result = m_kernels->integrate_positions(pos_buffer, vel_buffer, dt, count);
    ASSERT_EQ(result, BackendResult::Success);
    m_kernels->synchronize();

    m_backend->read_buffer(pos_buffer, positions.data(), positions.size() * sizeof(float));

    // Verify positions changed by velocity * dt (large change)
    for (SizeT i = 0; i < count * 3; ++i) {
        float expected = original_positions[i] + velocities[i] * dt;
        EXPECT_NEAR(positions[i], expected, 1e-4f);
    }

    m_kernels->free_buffer(pos_buffer);
    m_kernels->free_buffer(vel_buffer);
}

TEST_F(PhysicsKernelTest, EdgeCase_NegativeMass) {
    // Negative mass should be handled (treated as zero or absolute value)
    const SizeT count = 1;
    const float dt = 0.016f;

    std::vector<float> velocities = {0.0f, 0.0f, 0.0f};
    std::vector<float> forces = {10.0f, 0.0f, 0.0f};
    std::vector<float> masses = {-1.0f};  // Negative mass

    auto vel_buffer = m_kernels->allocate_vec3_buffer(count);
    auto force_buffer = m_kernels->allocate_vec3_buffer(count);
    auto mass_buffer = m_kernels->allocate_scalar_buffer(count);

    m_backend->write_buffer(vel_buffer, velocities.data(), velocities.size() * sizeof(float));
    m_backend->write_buffer(force_buffer, forces.data(), forces.size() * sizeof(float));
    m_backend->write_buffer(mass_buffer, masses.data(), masses.size() * sizeof(float));

    // Should not crash or produce NaN
    auto result = m_kernels->integrate_velocities(vel_buffer, force_buffer, mass_buffer, dt, count);
    ASSERT_EQ(result, BackendResult::Success);
    m_kernels->synchronize();

    m_backend->read_buffer(vel_buffer, velocities.data(), velocities.size() * sizeof(float));

    // Verify no NaN values
    for (int i = 0; i < 3; ++i) {
        EXPECT_FALSE(std::isnan(velocities[i])) << "NaN detected at velocity component " << i;
        EXPECT_FALSE(std::isinf(velocities[i])) << "Inf detected at velocity component " << i;
    }

    m_kernels->free_buffer(vel_buffer);
    m_kernels->free_buffer(force_buffer);
    m_kernels->free_buffer(mass_buffer);
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
