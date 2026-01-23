#!/usr/bin/env lua
--[[
    Basic tests for JaguarEngine Lua bindings

    This test suite verifies:
    - Core type operations (Vec3, Quat, Mat3x3)
    - Entity state management
    - Simulation control
    - Enums and constants

    Usage:
        lua test_basic.lua
]]

-- Add build directory to package.path
package.cpath = package.cpath .. ";./build/?.so;./build/?.dll"

-- Simple test framework
local tests_run = 0
local tests_passed = 0
local tests_failed = 0

local function assert_equal(actual, expected, message)
    tests_run = tests_run + 1
    if actual == expected then
        tests_passed = tests_passed + 1
        return true
    else
        tests_failed = tests_failed + 1
        print(string.format("  FAIL: %s (expected %s, got %s)",
            message or "test", tostring(expected), tostring(actual)))
        return false
    end
end

local function assert_near(actual, expected, tolerance, message)
    tests_run = tests_run + 1
    if math.abs(actual - expected) <= tolerance then
        tests_passed = tests_passed + 1
        return true
    else
        tests_failed = tests_failed + 1
        print(string.format("  FAIL: %s (expected %.6f +/- %.6f, got %.6f)",
            message or "test", expected, tolerance, actual))
        return false
    end
end

local function assert_true(condition, message)
    return assert_equal(condition, true, message)
end

local function assert_false(condition, message)
    return assert_equal(condition, false, message)
end

-- Try to load JaguarEngine
local status, jag = pcall(require, "jaguar")
if not status then
    print("SKIP: JaguarEngine Lua module not available")
    print("Build with: cmake -B build -DJAGUAR_BUILD_LUA=ON && cmake --build build")
    os.exit(0)
end

print("JaguarEngine Lua Bindings Test Suite")
print(string.rep("=", 50))

-- ========================================================================
-- Test Vec3
-- ========================================================================
print("\n[Test Vec3]")

-- Construction
local v1 = Vec3()
assert_equal(v1.x, 0.0, "Vec3() x")
assert_equal(v1.y, 0.0, "Vec3() y")
assert_equal(v1.z, 0.0, "Vec3() z")

local v2 = Vec3(1.0, 2.0, 3.0)
assert_equal(v2.x, 1.0, "Vec3(1,2,3) x")
assert_equal(v2.y, 2.0, "Vec3(1,2,3) y")
assert_equal(v2.z, 3.0, "Vec3(1,2,3) z")

-- Static constructors
local zero = Vec3.zero()
assert_equal(zero.x, 0.0, "Vec3.zero() x")

local unit_x = Vec3.unit_x()
assert_equal(unit_x.x, 1.0, "Vec3.unit_x() x")
assert_equal(unit_x.y, 0.0, "Vec3.unit_x() y")

-- Arithmetic
local v3 = Vec3(1.0, 2.0, 3.0)
local v4 = Vec3(4.0, 5.0, 6.0)

local sum = v3 + v4
assert_equal(sum.x, 5.0, "Vec3 + Vec3 x")
assert_equal(sum.y, 7.0, "Vec3 + Vec3 y")
assert_equal(sum.z, 9.0, "Vec3 + Vec3 z")

local diff = v4 - v3
assert_equal(diff.x, 3.0, "Vec3 - Vec3 x")

local scaled = v3 * 2.0
assert_equal(scaled.x, 2.0, "Vec3 * scalar x")
assert_equal(scaled.y, 4.0, "Vec3 * scalar y")

local neg = -v3
assert_equal(neg.x, -1.0, "Vec3 negation x")

-- Vector operations
local dot = v3:dot(v4)
assert_near(dot, 32.0, 0.0001, "Vec3 dot product")

local v5 = Vec3(3.0, 4.0, 0.0)
assert_near(v5:length(), 5.0, 0.0001, "Vec3 length")
assert_near(v5:length_squared(), 25.0, 0.0001, "Vec3 length_squared")

local norm = v5:normalized()
assert_near(norm:length(), 1.0, 0.0001, "Vec3 normalized length")

-- Cross product
local x = Vec3(1, 0, 0)
local y = Vec3(0, 1, 0)
local z = x:cross(y)
assert_near(z.x, 0.0, 0.0001, "Vec3 cross x")
assert_near(z.y, 0.0, 0.0001, "Vec3 cross y")
assert_near(z.z, 1.0, 0.0001, "Vec3 cross z")

print("  Vec3 tests: PASSED")

-- ========================================================================
-- Test Quat
-- ========================================================================
print("\n[Test Quat]")

-- Identity
local qi = Quat.identity()
assert_equal(qi.w, 1.0, "Quat.identity() w")
assert_equal(qi.x, 0.0, "Quat.identity() x")
assert_equal(qi.y, 0.0, "Quat.identity() y")
assert_equal(qi.z, 0.0, "Quat.identity() z")

-- From axis-angle
local axis = Vec3(0.0, 0.0, 1.0)
local angle = math.pi / 2  -- 90 degrees
local q = Quat.from_axis_angle(axis, angle)

-- Rotate a vector
local v = Vec3(1.0, 0.0, 0.0)
local rotated = q:rotate(v)
assert_near(rotated.x, 0.0, 0.0001, "Quat rotate x")
assert_near(rotated.y, 1.0, 0.0001, "Quat rotate y")
assert_near(rotated.z, 0.0, 0.0001, "Quat rotate z")

-- From euler and back
local roll, pitch, yaw = 0.0, 0.0, math.pi / 2
local qe = Quat.from_euler(roll, pitch, yaw)
local r2, p2, y2 = qe:to_euler()
assert_near(y2, yaw, 0.0001, "Quat euler round-trip yaw")

-- Conjugate and operations
local qc = q:conjugate()
assert_near(qc.x, -q.x, 0.0001, "Quat conjugate x")

print("  Quat tests: PASSED")

-- ========================================================================
-- Test Mat3x3
-- ========================================================================
print("\n[Test Mat3x3]")

local mi = Mat3x3.identity()
assert_equal(mi:get(0, 0), 1.0, "Mat3x3.identity() (0,0)")
assert_equal(mi:get(1, 1), 1.0, "Mat3x3.identity() (1,1)")
assert_equal(mi:get(2, 2), 1.0, "Mat3x3.identity() (2,2)")
assert_equal(mi:get(0, 1), 0.0, "Mat3x3.identity() (0,1)")

local mz = Mat3x3.zero()
assert_equal(mz:get(0, 0), 0.0, "Mat3x3.zero() (0,0)")

-- Diagonal and inertia
local md = Mat3x3.inertia(1.0, 2.0, 3.0)
assert_equal(md:get(0, 0), 1.0, "Mat3x3.inertia (0,0)")
assert_equal(md:get(1, 1), 2.0, "Mat3x3.inertia (1,1)")
assert_equal(md:get(2, 2), 3.0, "Mat3x3.inertia (2,2)")

-- Trace
assert_near(mi:trace(), 3.0, 0.0001, "Mat3x3 identity trace")

-- Matrix-vector multiply
local m = Mat3x3.inertia(2.0, 3.0, 4.0)
local vv = Vec3(1.0, 1.0, 1.0)
local result = m * vv
assert_near(result.x, 2.0, 0.0001, "Mat3x3 * Vec3 x")
assert_near(result.y, 3.0, 0.0001, "Mat3x3 * Vec3 y")
assert_near(result.z, 4.0, 0.0001, "Mat3x3 * Vec3 z")

print("  Mat3x3 tests: PASSED")

-- ========================================================================
-- Test EntityState
-- ========================================================================
print("\n[Test EntityState]")

local state = EntityState()

-- Default values
assert_equal(state.position.x, 0.0, "EntityState default position.x")
assert_equal(state.mass, 1.0, "EntityState default mass")

-- Modification
state.position = Vec3(100.0, 200.0, 300.0)
state.velocity = Vec3(10.0, 0.0, 0.0)
state.mass = 1000.0

assert_equal(state.position.x, 100.0, "EntityState position.x after set")
assert_equal(state.velocity.x, 10.0, "EntityState velocity.x after set")
assert_equal(state.mass, 1000.0, "EntityState mass after set")

print("  EntityState tests: PASSED")

-- ========================================================================
-- Test Constants
-- ========================================================================
print("\n[Test Constants]")

assert_near(constants.PI, math.pi, 0.0000001, "PI constant")
assert_equal(constants.G0, 9.80665, "G0 constant")
assert_true(constants.C > 299000000, "Speed of light")

print("  Constants tests: PASSED")

-- ========================================================================
-- Test Unit Conversions
-- ========================================================================
print("\n[Test Unit Conversions]")

assert_near(deg_to_rad(180.0), math.pi, 0.0001, "deg_to_rad(180)")
assert_near(rad_to_deg(math.pi), 180.0, 0.0001, "rad_to_deg(PI)")
assert_near(ft_to_m(1.0), 0.3048, 0.0001, "ft_to_m(1)")

print("  Unit conversion tests: PASSED")

-- ========================================================================
-- Test Enums
-- ========================================================================
print("\n[Test Enums]")

assert_true(Domain.Air ~= nil, "Domain.Air exists")
assert_true(Domain.Land ~= nil, "Domain.Land exists")
assert_true(Domain.Sea ~= nil, "Domain.Sea exists")
assert_true(Domain.Space ~= nil, "Domain.Space exists")
assert_true(Domain.Generic ~= nil, "Domain.Generic exists")

assert_true(CoordinateFrame.ECEF ~= nil, "CoordinateFrame.ECEF exists")
assert_true(CoordinateFrame.NED ~= nil, "CoordinateFrame.NED exists")

assert_true(SimulationState.Uninitialized ~= nil, "SimulationState.Uninitialized exists")
assert_true(SimulationState.Running ~= nil, "SimulationState.Running exists")

print("  Enum tests: PASSED")

-- ========================================================================
-- Summary
-- ========================================================================
print("\n" .. string.rep("=", 50))
print(string.format("Tests run: %d", tests_run))
print(string.format("Passed: %d", tests_passed))
print(string.format("Failed: %d", tests_failed))
print(string.rep("=", 50))

if tests_failed > 0 then
    print("SOME TESTS FAILED")
    os.exit(1)
else
    print("ALL TESTS PASSED")
    os.exit(0)
end
