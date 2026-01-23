#!/usr/bin/env python3
"""
Basic tests for JaguarEngine Python bindings (pyjaguar module).

This test suite verifies:
- Core type operations (Vec3, Quat, Mat3x3)
- Entity state management
- Simulation control
- NumPy interoperability
- Event system integration

Usage:
    python -m pytest test_basic.py -v

Or run directly:
    python test_basic.py
"""

import sys
import math
import unittest

# Import will fail until the module is built with -DJAGUAR_BUILD_PYTHON=ON
try:
    import pyjaguar as jag
    import numpy as np
    PYJAGUAR_AVAILABLE = True
except ImportError:
    PYJAGUAR_AVAILABLE = False
    print("Warning: pyjaguar module not available. Build with -DJAGUAR_BUILD_PYTHON=ON")


@unittest.skipUnless(PYJAGUAR_AVAILABLE, "pyjaguar not built")
class TestVec3(unittest.TestCase):
    """Test Vec3 3D vector operations."""

    def test_construction(self):
        """Test Vec3 construction."""
        v1 = jag.Vec3()
        self.assertEqual(v1.x, 0.0)
        self.assertEqual(v1.y, 0.0)
        self.assertEqual(v1.z, 0.0)

        v2 = jag.Vec3(1.0, 2.0, 3.0)
        self.assertEqual(v2.x, 1.0)
        self.assertEqual(v2.y, 2.0)
        self.assertEqual(v2.z, 3.0)

    def test_static_constructors(self):
        """Test Vec3 static factory methods."""
        zero = jag.Vec3.zero()
        self.assertEqual(zero.x, 0.0)

        unit_x = jag.Vec3.unit_x()
        self.assertEqual(unit_x.x, 1.0)
        self.assertEqual(unit_x.y, 0.0)
        self.assertEqual(unit_x.z, 0.0)

        unit_y = jag.Vec3.unit_y()
        self.assertEqual(unit_y.y, 1.0)

        unit_z = jag.Vec3.unit_z()
        self.assertEqual(unit_z.z, 1.0)

    def test_arithmetic(self):
        """Test Vec3 arithmetic operations."""
        v1 = jag.Vec3(1.0, 2.0, 3.0)
        v2 = jag.Vec3(4.0, 5.0, 6.0)

        # Addition
        v3 = v1 + v2
        self.assertEqual(v3.x, 5.0)
        self.assertEqual(v3.y, 7.0)
        self.assertEqual(v3.z, 9.0)

        # Subtraction
        v4 = v2 - v1
        self.assertEqual(v4.x, 3.0)
        self.assertEqual(v4.y, 3.0)
        self.assertEqual(v4.z, 3.0)

        # Scalar multiplication
        v5 = v1 * 2.0
        self.assertEqual(v5.x, 2.0)
        self.assertEqual(v5.y, 4.0)
        self.assertEqual(v5.z, 6.0)

        # Scalar division
        v6 = v2 / 2.0
        self.assertEqual(v6.x, 2.0)
        self.assertEqual(v6.y, 2.5)
        self.assertEqual(v6.z, 3.0)

        # Negation
        v7 = -v1
        self.assertEqual(v7.x, -1.0)
        self.assertEqual(v7.y, -2.0)
        self.assertEqual(v7.z, -3.0)

    def test_vector_operations(self):
        """Test Vec3 dot product, cross product, length."""
        v1 = jag.Vec3(1.0, 0.0, 0.0)
        v2 = jag.Vec3(0.0, 1.0, 0.0)

        # Dot product
        dot = v1.dot(v2)
        self.assertAlmostEqual(dot, 0.0)

        # Cross product
        cross = v1.cross(v2)
        self.assertAlmostEqual(cross.x, 0.0)
        self.assertAlmostEqual(cross.y, 0.0)
        self.assertAlmostEqual(cross.z, 1.0)

        # Length
        v3 = jag.Vec3(3.0, 4.0, 0.0)
        self.assertAlmostEqual(v3.length(), 5.0)
        self.assertAlmostEqual(v3.length_squared(), 25.0)

        # Normalized
        v4 = v3.normalized()
        self.assertAlmostEqual(v4.length(), 1.0)

    def test_sequence_protocol(self):
        """Test Vec3 as a sequence."""
        v = jag.Vec3(1.0, 2.0, 3.0)

        self.assertEqual(len(v), 3)
        self.assertEqual(v[0], 1.0)
        self.assertEqual(v[1], 2.0)
        self.assertEqual(v[2], 3.0)

        v[0] = 10.0
        self.assertEqual(v.x, 10.0)

    def test_numpy_interop(self):
        """Test Vec3 NumPy array conversion."""
        v = jag.Vec3(1.0, 2.0, 3.0)
        arr = v.to_numpy()

        self.assertEqual(len(arr), 3)
        self.assertEqual(arr[0], 1.0)
        self.assertEqual(arr[1], 2.0)
        self.assertEqual(arr[2], 3.0)

        arr2 = np.array([4.0, 5.0, 6.0])
        v2 = jag.Vec3.from_numpy(arr2)
        self.assertEqual(v2.x, 4.0)
        self.assertEqual(v2.y, 5.0)
        self.assertEqual(v2.z, 6.0)


@unittest.skipUnless(PYJAGUAR_AVAILABLE, "pyjaguar not built")
class TestQuat(unittest.TestCase):
    """Test Quat quaternion operations."""

    def test_construction(self):
        """Test Quat construction."""
        q1 = jag.Quat()  # Identity
        self.assertEqual(q1.w, 1.0)
        self.assertEqual(q1.x, 0.0)
        self.assertEqual(q1.y, 0.0)
        self.assertEqual(q1.z, 0.0)

        q2 = jag.Quat(0.707, 0.707, 0.0, 0.0)
        self.assertAlmostEqual(q2.w, 0.707, places=3)

    def test_identity(self):
        """Test identity quaternion."""
        q = jag.Quat.identity()
        self.assertEqual(q.w, 1.0)
        self.assertEqual(q.x, 0.0)
        self.assertEqual(q.y, 0.0)
        self.assertEqual(q.z, 0.0)

    def test_from_axis_angle(self):
        """Test quaternion from axis-angle."""
        axis = jag.Vec3(0.0, 0.0, 1.0)
        angle = math.pi / 2  # 90 degrees

        q = jag.Quat.from_axis_angle(axis, angle)

        # Rotate a vector
        v = jag.Vec3(1.0, 0.0, 0.0)
        v_rotated = q.rotate(v)

        self.assertAlmostEqual(v_rotated.x, 0.0, places=5)
        self.assertAlmostEqual(v_rotated.y, 1.0, places=5)
        self.assertAlmostEqual(v_rotated.z, 0.0, places=5)

    def test_from_euler(self):
        """Test quaternion from Euler angles."""
        roll = 0.0
        pitch = 0.0
        yaw = math.pi / 2  # 90 degrees

        q = jag.Quat.from_euler(roll, pitch, yaw)

        # Convert back
        r, p, y = q.to_euler()
        self.assertAlmostEqual(yaw, y, places=5)

    def test_operations(self):
        """Test quaternion operations."""
        q1 = jag.Quat.from_axis_angle(jag.Vec3(0, 0, 1), math.pi / 4)
        q2 = jag.Quat.from_axis_angle(jag.Vec3(0, 0, 1), math.pi / 4)

        # Multiplication
        q3 = q1 * q2  # Should be 90 degree rotation

        v = jag.Vec3(1.0, 0.0, 0.0)
        v_rotated = q3.rotate(v)

        self.assertAlmostEqual(v_rotated.x, 0.0, places=5)
        self.assertAlmostEqual(v_rotated.y, 1.0, places=5)

        # Conjugate
        q_conj = q1.conjugate()
        self.assertAlmostEqual(q_conj.x, -q1.x, places=5)

        # Norm
        self.assertAlmostEqual(q1.norm(), 1.0, places=5)

        # Normalized
        q_norm = q1.normalized()
        self.assertAlmostEqual(q_norm.norm(), 1.0, places=5)


@unittest.skipUnless(PYJAGUAR_AVAILABLE, "pyjaguar not built")
class TestMat3x3(unittest.TestCase):
    """Test Mat3x3 matrix operations."""

    def test_construction(self):
        """Test Mat3x3 construction."""
        m = jag.Mat3x3()  # Identity
        self.assertEqual(m(0, 0), 1.0)
        self.assertEqual(m(1, 1), 1.0)
        self.assertEqual(m(2, 2), 1.0)
        self.assertEqual(m(0, 1), 0.0)

    def test_static_constructors(self):
        """Test Mat3x3 static factory methods."""
        identity = jag.Mat3x3.identity()
        self.assertEqual(identity(0, 0), 1.0)

        zero = jag.Mat3x3.zero()
        self.assertEqual(zero(0, 0), 0.0)

        diag = jag.Mat3x3.diagonal(1.0, 2.0, 3.0)
        self.assertEqual(diag(0, 0), 1.0)
        self.assertEqual(diag(1, 1), 2.0)
        self.assertEqual(diag(2, 2), 3.0)

    def test_matrix_vector_multiply(self):
        """Test matrix-vector multiplication."""
        m = jag.Mat3x3.diagonal(2.0, 3.0, 4.0)
        v = jag.Vec3(1.0, 1.0, 1.0)

        result = m * v
        self.assertEqual(result.x, 2.0)
        self.assertEqual(result.y, 3.0)
        self.assertEqual(result.z, 4.0)

    def test_numpy_interop(self):
        """Test Mat3x3 NumPy conversion."""
        m = jag.Mat3x3.diagonal(1.0, 2.0, 3.0)
        arr = m.to_numpy()

        self.assertEqual(arr.shape, (3, 3))
        self.assertEqual(arr[0, 0], 1.0)
        self.assertEqual(arr[1, 1], 2.0)
        self.assertEqual(arr[2, 2], 3.0)


@unittest.skipUnless(PYJAGUAR_AVAILABLE, "pyjaguar not built")
class TestEntityState(unittest.TestCase):
    """Test EntityState structure."""

    def test_construction(self):
        """Test EntityState default construction."""
        state = jag.EntityState()

        self.assertEqual(state.position.x, 0.0)
        self.assertEqual(state.velocity.x, 0.0)
        self.assertEqual(state.mass, 1.0)

    def test_modification(self):
        """Test EntityState modification."""
        state = jag.EntityState()

        state.position = jag.Vec3(100.0, 200.0, 300.0)
        state.velocity = jag.Vec3(10.0, 0.0, 0.0)
        state.mass = 1000.0

        self.assertEqual(state.position.x, 100.0)
        self.assertEqual(state.velocity.x, 10.0)
        self.assertEqual(state.mass, 1000.0)


@unittest.skipUnless(PYJAGUAR_AVAILABLE, "pyjaguar not built")
class TestConstants(unittest.TestCase):
    """Test physical constants."""

    def test_constants_available(self):
        """Test that constants are accessible."""
        self.assertAlmostEqual(jag.constants.PI, math.pi, places=10)
        self.assertEqual(jag.constants.G0, 9.80665)
        self.assertGreater(jag.constants.C, 299000000)

    def test_unit_conversions(self):
        """Test unit conversion functions."""
        self.assertAlmostEqual(jag.deg_to_rad(180.0), math.pi, places=5)
        self.assertAlmostEqual(jag.rad_to_deg(math.pi), 180.0, places=5)
        self.assertAlmostEqual(jag.ft_to_m(1.0), 0.3048, places=4)


@unittest.skipUnless(PYJAGUAR_AVAILABLE, "pyjaguar not built")
class TestEnums(unittest.TestCase):
    """Test enumeration bindings."""

    def test_domain_enum(self):
        """Test Domain enum."""
        self.assertIsNotNone(jag.Domain.Air)
        self.assertIsNotNone(jag.Domain.Land)
        self.assertIsNotNone(jag.Domain.Sea)
        self.assertIsNotNone(jag.Domain.Space)
        self.assertIsNotNone(jag.Domain.Generic)

    def test_coordinate_frame_enum(self):
        """Test CoordinateFrame enum."""
        self.assertIsNotNone(jag.CoordinateFrame.ECEF)
        self.assertIsNotNone(jag.CoordinateFrame.ECI)
        self.assertIsNotNone(jag.CoordinateFrame.NED)
        self.assertIsNotNone(jag.CoordinateFrame.ENU)
        self.assertIsNotNone(jag.CoordinateFrame.Body)

    def test_simulation_state_enum(self):
        """Test SimulationState enum."""
        self.assertIsNotNone(jag.SimulationState.Uninitialized)
        self.assertIsNotNone(jag.SimulationState.Initialized)
        self.assertIsNotNone(jag.SimulationState.Running)
        self.assertIsNotNone(jag.SimulationState.Paused)
        self.assertIsNotNone(jag.SimulationState.Stopped)


@unittest.skipUnless(PYJAGUAR_AVAILABLE, "pyjaguar not built")
class TestEventSystem(unittest.TestCase):
    """Test event system bindings."""

    def test_event_types(self):
        """Test EventType enum."""
        self.assertIsNotNone(jag.EventType.SimulationStarted)
        self.assertIsNotNone(jag.EventType.EntityCreated)
        self.assertIsNotNone(jag.EventType.CollisionEnter)
        self.assertIsNotNone(jag.EventType.Stall)
        self.assertIsNotNone(jag.EventType.ThresholdExceeded)

    def test_event_category(self):
        """Test EventCategory enum."""
        self.assertIsNotNone(jag.EventCategory.System)
        self.assertIsNotNone(jag.EventCategory.Entity)
        self.assertIsNotNone(jag.EventCategory.Physics)
        self.assertIsNotNone(jag.EventCategory.Domain)
        self.assertIsNotNone(jag.EventCategory.Threshold)

    def test_event_priority(self):
        """Test EventPriority enum."""
        self.assertIsNotNone(jag.EventPriority.Immediate)
        self.assertIsNotNone(jag.EventPriority.High)
        self.assertIsNotNone(jag.EventPriority.Normal)
        self.assertIsNotNone(jag.EventPriority.Low)
        self.assertIsNotNone(jag.EventPriority.Deferred)

    def test_threshold_config(self):
        """Test ThresholdConfig creation."""
        config = jag.ThresholdConfig()
        config.parameter_name = "speed"
        config.upper_threshold = 100.0
        config.hysteresis = 5.0
        config.emit_on_recover = True

        self.assertEqual(config.parameter_name, "speed")
        self.assertEqual(config.upper_threshold, 100.0)

    def test_threshold_presets(self):
        """Test threshold preset factory functions."""
        fuel_config = jag.fuel_threshold(20.0, 5.0)
        self.assertEqual(fuel_config.parameter_name, "fuel_level")

        speed_config = jag.speed_threshold(100.0, 5.0)
        self.assertEqual(speed_config.parameter_name, "speed")

        alt_config = jag.altitude_threshold(15000.0, 100.0)
        self.assertEqual(alt_config.parameter_name, "altitude")


if __name__ == "__main__":
    unittest.main()
