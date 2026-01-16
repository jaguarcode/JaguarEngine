#!/usr/bin/env python3
"""
JaguarEngine Python Bindings Test Suite

Tests for pyjaguar module functionality.
Run with: pytest tests/python/test_pyjaguar.py -v
"""

import pytest
import math
import numpy as np

# Import pyjaguar (requires build with JAGUAR_BUILD_PYTHON=ON)
try:
    import pyjaguar as jag
    HAS_PYJAGUAR = True
except ImportError:
    HAS_PYJAGUAR = False
    jag = None

pytestmark = pytest.mark.skipif(not HAS_PYJAGUAR, reason="pyjaguar not built")


# =============================================================================
# Vec3 Tests
# =============================================================================

class TestVec3:
    """Tests for Vec3 3D vector class."""

    def test_default_constructor(self):
        """Test default Vec3 initialization."""
        v = jag.Vec3()
        assert v.x == 0.0
        assert v.y == 0.0
        assert v.z == 0.0

    def test_parameterized_constructor(self):
        """Test Vec3 initialization with values."""
        v = jag.Vec3(1.0, 2.0, 3.0)
        assert v.x == 1.0
        assert v.y == 2.0
        assert v.z == 3.0

    def test_addition(self):
        """Test Vec3 addition operator."""
        v1 = jag.Vec3(1.0, 2.0, 3.0)
        v2 = jag.Vec3(4.0, 5.0, 6.0)
        result = v1 + v2
        assert result.x == 5.0
        assert result.y == 7.0
        assert result.z == 9.0

    def test_subtraction(self):
        """Test Vec3 subtraction operator."""
        v1 = jag.Vec3(5.0, 7.0, 9.0)
        v2 = jag.Vec3(1.0, 2.0, 3.0)
        result = v1 - v2
        assert result.x == 4.0
        assert result.y == 5.0
        assert result.z == 6.0

    def test_scalar_multiplication(self):
        """Test Vec3 scalar multiplication."""
        v = jag.Vec3(1.0, 2.0, 3.0)
        result = v * 2.0
        assert result.x == 2.0
        assert result.y == 4.0
        assert result.z == 6.0

    def test_dot_product(self):
        """Test Vec3 dot product."""
        v1 = jag.Vec3(1.0, 2.0, 3.0)
        v2 = jag.Vec3(4.0, 5.0, 6.0)
        # 1*4 + 2*5 + 3*6 = 4 + 10 + 18 = 32
        assert v1.dot(v2) == pytest.approx(32.0)

    def test_cross_product(self):
        """Test Vec3 cross product."""
        v1 = jag.Vec3(1.0, 0.0, 0.0)
        v2 = jag.Vec3(0.0, 1.0, 0.0)
        result = v1.cross(v2)
        assert result.x == pytest.approx(0.0)
        assert result.y == pytest.approx(0.0)
        assert result.z == pytest.approx(1.0)

    def test_length(self):
        """Test Vec3 length calculation."""
        v = jag.Vec3(3.0, 4.0, 0.0)
        assert v.length() == pytest.approx(5.0)

    def test_length_squared(self):
        """Test Vec3 squared length calculation."""
        v = jag.Vec3(3.0, 4.0, 0.0)
        assert v.length_squared() == pytest.approx(25.0)

    def test_normalized(self):
        """Test Vec3 normalization."""
        v = jag.Vec3(3.0, 4.0, 0.0)
        n = v.normalized()
        assert n.x == pytest.approx(0.6)
        assert n.y == pytest.approx(0.8)
        assert n.z == pytest.approx(0.0)
        assert n.length() == pytest.approx(1.0)

    def test_numpy_conversion(self):
        """Test Vec3 to/from numpy array conversion."""
        v = jag.Vec3(1.0, 2.0, 3.0)
        arr = v.to_numpy()
        assert isinstance(arr, np.ndarray)
        assert arr.shape == (3,)
        assert np.allclose(arr, [1.0, 2.0, 3.0])

        # Convert back
        v2 = jag.Vec3.from_numpy(np.array([7.0, 8.0, 9.0]))
        assert v2.x == 7.0
        assert v2.y == 8.0
        assert v2.z == 9.0

    def test_string_representation(self):
        """Test Vec3 string representation."""
        v = jag.Vec3(1.5, 2.5, 3.5)
        s = str(v)
        assert "1.5" in s
        assert "2.5" in s
        assert "3.5" in s


# =============================================================================
# Quat Tests
# =============================================================================

class TestQuat:
    """Tests for Quat quaternion class."""

    def test_identity(self):
        """Test identity quaternion."""
        q = jag.Quat.identity()
        assert q.w == pytest.approx(1.0)
        assert q.x == pytest.approx(0.0)
        assert q.y == pytest.approx(0.0)
        assert q.z == pytest.approx(0.0)

    def test_from_euler_zero(self):
        """Test quaternion from zero Euler angles."""
        q = jag.Quat.from_euler(0.0, 0.0, 0.0)
        assert q.w == pytest.approx(1.0, abs=1e-6)
        assert q.x == pytest.approx(0.0, abs=1e-6)
        assert q.y == pytest.approx(0.0, abs=1e-6)
        assert q.z == pytest.approx(0.0, abs=1e-6)

    def test_from_euler_90_yaw(self):
        """Test quaternion from 90 degree yaw rotation."""
        yaw = math.pi / 2  # 90 degrees
        q = jag.Quat.from_euler(0.0, 0.0, yaw)

        # Rotate unit X vector, should become unit Y
        v = jag.Vec3(1.0, 0.0, 0.0)
        rotated = q.rotate(v)

        assert rotated.x == pytest.approx(0.0, abs=1e-6)
        assert rotated.y == pytest.approx(1.0, abs=1e-6)
        assert rotated.z == pytest.approx(0.0, abs=1e-6)

    def test_quaternion_multiplication(self):
        """Test quaternion multiplication."""
        q1 = jag.Quat.from_euler(0.0, 0.0, math.pi / 4)  # 45 deg yaw
        q2 = jag.Quat.from_euler(0.0, 0.0, math.pi / 4)  # 45 deg yaw
        q3 = q1 * q2  # Should be 90 deg yaw

        v = jag.Vec3(1.0, 0.0, 0.0)
        rotated = q3.rotate(v)

        assert rotated.x == pytest.approx(0.0, abs=1e-6)
        assert rotated.y == pytest.approx(1.0, abs=1e-6)

    def test_conjugate(self):
        """Test quaternion conjugate."""
        q = jag.Quat.from_euler(0.1, 0.2, 0.3)
        qc = q.conjugate()

        # q * conjugate(q) should be identity
        result = q * qc
        assert result.w == pytest.approx(1.0, abs=1e-6)

    def test_inverse(self):
        """Test quaternion inverse."""
        q = jag.Quat.from_euler(0.1, 0.2, 0.3)
        qi = q.inverse()

        # q * inverse(q) should be identity
        result = q * qi
        assert result.w == pytest.approx(1.0, abs=1e-6)


# =============================================================================
# Mat3x3 Tests
# =============================================================================

class TestMat3x3:
    """Tests for Mat3x3 3x3 matrix class."""

    def test_identity(self):
        """Test identity matrix."""
        m = jag.Mat3x3.identity()
        for i in range(3):
            for j in range(3):
                expected = 1.0 if i == j else 0.0
                assert m[i, j] == pytest.approx(expected)

    def test_zero(self):
        """Test zero matrix."""
        m = jag.Mat3x3.zero()
        for i in range(3):
            for j in range(3):
                assert m[i, j] == pytest.approx(0.0)

    def test_inertia_diagonal(self):
        """Test diagonal inertia matrix creation."""
        m = jag.Mat3x3.inertia(100.0, 200.0, 300.0)
        assert m[0, 0] == pytest.approx(100.0)
        assert m[1, 1] == pytest.approx(200.0)
        assert m[2, 2] == pytest.approx(300.0)
        # Off-diagonal should be zero
        assert m[0, 1] == pytest.approx(0.0)
        assert m[0, 2] == pytest.approx(0.0)
        assert m[1, 2] == pytest.approx(0.0)

    def test_matrix_vector_multiplication(self):
        """Test matrix-vector multiplication."""
        m = jag.Mat3x3.identity()
        v = jag.Vec3(1.0, 2.0, 3.0)
        result = m * v
        assert result.x == pytest.approx(1.0)
        assert result.y == pytest.approx(2.0)
        assert result.z == pytest.approx(3.0)

    def test_transpose(self):
        """Test matrix transpose."""
        m = jag.Mat3x3.identity()
        # Set some off-diagonal elements
        # Note: This depends on whether Mat3x3 has a setter
        mt = m.transpose()
        # Identity transpose is identity
        for i in range(3):
            for j in range(3):
                expected = 1.0 if i == j else 0.0
                assert mt[i, j] == pytest.approx(expected)


# =============================================================================
# Domain and Enums Tests
# =============================================================================

class TestEnums:
    """Tests for enumeration types."""

    def test_domain_values(self):
        """Test Domain enum values exist."""
        assert hasattr(jag, 'Domain')
        assert hasattr(jag.Domain, 'Air')
        assert hasattr(jag.Domain, 'Land')
        assert hasattr(jag.Domain, 'Sea')
        assert hasattr(jag.Domain, 'Space')

    def test_coordinate_frame_values(self):
        """Test CoordinateFrame enum values exist."""
        assert hasattr(jag, 'CoordinateFrame')
        assert hasattr(jag.CoordinateFrame, 'ECEF')
        assert hasattr(jag.CoordinateFrame, 'ECI')
        assert hasattr(jag.CoordinateFrame, 'NED')
        assert hasattr(jag.CoordinateFrame, 'Body')

    def test_simulation_state_values(self):
        """Test SimulationState enum values exist."""
        assert hasattr(jag, 'SimulationState')
        assert hasattr(jag.SimulationState, 'Uninitialized')
        assert hasattr(jag.SimulationState, 'Initialized')
        assert hasattr(jag.SimulationState, 'Running')
        assert hasattr(jag.SimulationState, 'Paused')
        assert hasattr(jag.SimulationState, 'Stopped')


# =============================================================================
# Engine Tests
# =============================================================================

class TestEngine:
    """Tests for Engine simulation class."""

    def test_engine_creation(self):
        """Test Engine instantiation."""
        engine = jag.Engine()
        assert engine is not None

    def test_engine_initialize(self):
        """Test Engine initialization."""
        engine = jag.Engine()
        result = engine.initialize()
        assert result is True
        engine.shutdown()

    def test_engine_state_transitions(self):
        """Test Engine state transitions."""
        engine = jag.Engine()

        # Before init
        assert engine.get_state() == jag.SimulationState.Uninitialized

        # After init
        engine.initialize()
        assert engine.get_state() == jag.SimulationState.Initialized

        # After step
        engine.step(0.01)
        assert engine.get_state() == jag.SimulationState.Running

        # After pause
        engine.pause()
        assert engine.get_state() == jag.SimulationState.Paused

        # After resume
        engine.resume()
        assert engine.get_state() == jag.SimulationState.Running

        # After stop
        engine.stop()
        assert engine.get_state() == jag.SimulationState.Stopped

        engine.shutdown()

    def test_create_entity(self):
        """Test entity creation."""
        engine = jag.Engine()
        engine.initialize()

        entity_id = engine.create_entity("test_aircraft", jag.Domain.Air)
        assert entity_id > 0

        engine.shutdown()

    def test_entity_state_manipulation(self):
        """Test getting and setting entity state."""
        engine = jag.Engine()
        engine.initialize()

        entity_id = engine.create_entity("test_vehicle", jag.Domain.Land)

        # Get initial state
        state = engine.get_entity_state(entity_id)
        assert state is not None

        # Modify state
        state.position = jag.Vec3(100.0, 200.0, 0.0)
        state.velocity = jag.Vec3(10.0, 0.0, 0.0)
        state.mass = 5000.0

        # Set modified state
        engine.set_entity_state(entity_id, state)

        # Verify
        retrieved = engine.get_entity_state(entity_id)
        assert retrieved.position.x == pytest.approx(100.0)
        assert retrieved.position.y == pytest.approx(200.0)
        assert retrieved.velocity.x == pytest.approx(10.0)
        assert retrieved.mass == pytest.approx(5000.0)

        engine.shutdown()

    def test_simulation_step(self):
        """Test simulation stepping."""
        engine = jag.Engine()
        engine.initialize()

        initial_time = engine.get_time()
        assert initial_time == pytest.approx(0.0)

        # Step simulation
        dt = 0.1
        engine.step(dt)

        # Time should advance
        assert engine.get_time() == pytest.approx(dt)

        engine.shutdown()

    def test_remove_entity(self):
        """Test entity removal."""
        engine = jag.Engine()
        engine.initialize()

        entity_id = engine.create_entity("temporary", jag.Domain.Sea)
        assert engine.remove_entity(entity_id) is True

        engine.shutdown()


# =============================================================================
# EntityState Tests
# =============================================================================

class TestEntityState:
    """Tests for EntityState structure."""

    def test_entity_state_creation(self):
        """Test EntityState instantiation."""
        state = jag.EntityState()
        assert state is not None

    def test_entity_state_position(self):
        """Test EntityState position property."""
        state = jag.EntityState()
        state.position = jag.Vec3(1.0, 2.0, 3.0)
        assert state.position.x == 1.0
        assert state.position.y == 2.0
        assert state.position.z == 3.0

    def test_entity_state_velocity(self):
        """Test EntityState velocity property."""
        state = jag.EntityState()
        state.velocity = jag.Vec3(10.0, 20.0, 30.0)
        assert state.velocity.x == 10.0
        assert state.velocity.y == 20.0
        assert state.velocity.z == 30.0

    def test_entity_state_mass(self):
        """Test EntityState mass property."""
        state = jag.EntityState()
        state.mass = 12500.0
        assert state.mass == pytest.approx(12500.0)


# =============================================================================
# Constants Tests
# =============================================================================

class TestConstants:
    """Tests for physical constants."""

    def test_gravitational_constant(self):
        """Test gravitational constant G."""
        assert hasattr(jag.constants, 'G')
        assert jag.constants.G == pytest.approx(6.67430e-11, rel=1e-4)

    def test_earth_radius(self):
        """Test Earth equatorial radius."""
        assert hasattr(jag.constants, 'R_EARTH_EQUATOR')
        assert jag.constants.R_EARTH_EQUATOR == pytest.approx(6378137.0)

    def test_standard_gravity(self):
        """Test standard gravity g0."""
        assert hasattr(jag.constants, 'G0')
        assert jag.constants.G0 == pytest.approx(9.80665)

    def test_sea_level_density(self):
        """Test sea level air density."""
        assert hasattr(jag.constants, 'RHO0')
        assert jag.constants.RHO0 == pytest.approx(1.225, rel=0.01)

    def test_pi(self):
        """Test PI constant."""
        assert hasattr(jag.constants, 'PI')
        assert jag.constants.PI == pytest.approx(math.pi)


# =============================================================================
# Unit Conversion Tests
# =============================================================================

class TestUnitConversions:
    """Tests for unit conversion functions."""

    def test_feet_to_meters(self):
        """Test feet to meters conversion."""
        assert jag.ft_to_m(1.0) == pytest.approx(0.3048)
        assert jag.ft_to_m(100.0) == pytest.approx(30.48)

    def test_meters_to_feet(self):
        """Test meters to feet conversion."""
        assert jag.m_to_ft(1.0) == pytest.approx(3.28084, rel=1e-4)
        assert jag.m_to_ft(30.48) == pytest.approx(100.0, rel=1e-4)

    def test_knots_to_ms(self):
        """Test knots to m/s conversion."""
        assert jag.kt_to_ms(1.0) == pytest.approx(0.514444, rel=1e-4)
        assert jag.kt_to_ms(100.0) == pytest.approx(51.4444, rel=1e-4)

    def test_ms_to_knots(self):
        """Test m/s to knots conversion."""
        assert jag.ms_to_kt(1.0) == pytest.approx(1.94384, rel=1e-4)

    def test_deg_to_rad(self):
        """Test degrees to radians conversion."""
        assert jag.deg_to_rad(0.0) == pytest.approx(0.0)
        assert jag.deg_to_rad(90.0) == pytest.approx(math.pi / 2)
        assert jag.deg_to_rad(180.0) == pytest.approx(math.pi)

    def test_rad_to_deg(self):
        """Test radians to degrees conversion."""
        assert jag.rad_to_deg(0.0) == pytest.approx(0.0)
        assert jag.rad_to_deg(math.pi / 2) == pytest.approx(90.0)
        assert jag.rad_to_deg(math.pi) == pytest.approx(180.0)


# =============================================================================
# Integration Tests
# =============================================================================

class TestIntegration:
    """Integration tests for full simulation workflows."""

    def test_basic_simulation_workflow(self):
        """Test a basic simulation workflow end-to-end."""
        engine = jag.Engine()
        engine.initialize()

        # Create aircraft
        aircraft = engine.create_entity("f16", jag.Domain.Air)

        # Set initial state
        state = engine.get_entity_state(aircraft)
        state.position = jag.Vec3(0.0, 0.0, -1000.0)  # 1000m altitude (NED)
        state.velocity = jag.Vec3(200.0, 0.0, 0.0)    # 200 m/s
        state.mass = 12000.0
        engine.set_entity_state(aircraft, state)

        # Run simulation for 1 second
        dt = 0.01
        for _ in range(100):
            engine.step(dt)

        # Check final state
        final_state = engine.get_entity_state(aircraft)

        # Aircraft should have moved forward
        assert final_state.position.x > 0.0

        # Simulation time should be ~1 second
        assert engine.get_time() == pytest.approx(1.0, abs=0.01)

        engine.shutdown()

    def test_multi_entity_simulation(self):
        """Test simulation with multiple entities."""
        engine = jag.Engine()
        engine.initialize()

        # Create entities in different domains
        entities = {
            'aircraft': engine.create_entity("f16", jag.Domain.Air),
            'tank': engine.create_entity("m1a2", jag.Domain.Land),
            'ship': engine.create_entity("destroyer", jag.Domain.Sea),
            'satellite': engine.create_entity("gps01", jag.Domain.Space),
        }

        # All entities should have unique IDs
        ids = list(entities.values())
        assert len(ids) == len(set(ids))

        # All IDs should be positive
        for eid in ids:
            assert eid > 0

        # Run a few steps
        for _ in range(10):
            engine.step(0.01)

        # All entities should still exist and have valid states
        for name, eid in entities.items():
            state = engine.get_entity_state(eid)
            assert state is not None

        engine.shutdown()

    def test_numpy_workflow(self):
        """Test numpy integration in simulation workflow."""
        engine = jag.Engine()
        engine.initialize()

        entity = engine.create_entity("test", jag.Domain.Land)

        # Set position using numpy
        pos_np = np.array([100.0, 200.0, 0.0])
        state = engine.get_entity_state(entity)
        state.position = jag.Vec3.from_numpy(pos_np)
        engine.set_entity_state(entity, state)

        # Retrieve and convert back to numpy
        retrieved = engine.get_entity_state(entity)
        pos_out = retrieved.position.to_numpy()

        assert np.allclose(pos_np, pos_out)

        engine.shutdown()


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
