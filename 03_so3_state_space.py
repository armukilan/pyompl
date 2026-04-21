"""
===============================================================
pyompl Example 03 — SO3StateSpace
===============================================================

WHAT IS SO(3)?
--------------
SO(3) = Special Orthogonal Group in 3 dimensions.
It is the space of ALL possible 3D rotations.

Think of it as: every possible way you can orient a rigid body
in 3D space — every roll, pitch, and yaw combination combined.

INTERNAL REPRESENTATION: UNIT QUATERNIONS
------------------------------------------
OMPL uses unit quaternions q = (x, y, z, w) where:
  x^2 + y^2 + z^2 + w^2 = 1

A quaternion encodes:
  - axis of rotation: (x, y, z) / sin(angle/2)
  - amount of rotation: angle = 2 * acos(w)

WHY QUATERNIONS INSTEAD OF EULER ANGLES?
-----------------------------------------
  Euler angles (roll/pitch/yaw) have GIMBAL LOCK:
    At certain orientations, one degree of freedom is lost.
    Interpolation breaks down near these singularities.

  Quaternions have NO gimbal lock:
    Smooth interpolation (SLERP) everywhere.
    Well-defined distance metric: angle between orientations.

IMPORTANT PROPERTY: DOUBLE COVER
----------------------------------
q and -q represent the SAME physical rotation.
This means the maximum distance between any two orientations
is π/2, not π (because of this double cover).

USE FOR:
  - 3D robot orientation (drone, robotic arm end-effector)
  - Camera pose
  - Any freely-rotating 3D joint

C++ HEADER: ompl/base/spaces/SO3StateSpace.h
C++ CLASS:  ompl::base::SO3StateSpace

RUN THIS FILE:
    python examples/03_so3_state_space.py
===============================================================
"""

import math
import pyompl

PI = math.pi

def check(condition, msg):
    print(f"  {'PASS' if condition else 'FAIL'} — {msg}")

def approx(a, b, tol=1e-6):
    return abs(a - b) < tol


print("=" * 60)
print("SECTION 1: SO3StateSpace — Construction and Properties")
print("=" * 60)
print("""
SO3StateSpace needs no arguments. The topology is fixed.
No bounds are needed — orientation space is bounded by the
spherical topology itself.

C++: auto space = std::make_shared<SO3StateSpace>();
""")

space = pyompl.SO3StateSpace()
space.setup()

print(f"  getName()          = {space.getName()}")
print(f"  getDimension()     = {space.getDimension()}")
print(f"  getMaximumExtent() = {space.getMaximumExtent():.6f}  (should be π/2 ≈ {PI/2:.6f})")
print(f"  getMeasure()       = {space.getMeasure():.6f}  (should be π^2 ≈ {PI**2:.6f})")

check(space.getDimension() == 3, "getDimension() == 3")
check(approx(space.getMaximumExtent(), PI/2), "max extent == π/2")
check(approx(space.getMeasure(), PI**2, tol=1e-4), "measure == π^2")

# setName / getName
space.setName("DroneOrientation")
print(f"\n  After setName: {space.getName()}")
check("DroneOrientation" in space.getName(), "getName contains 'DroneOrientation'")

# printSettings
settings = space.printSettings()
print(f"\n-- printSettings():\n{settings}")
check(len(settings) > 0, "printSettings returns non-empty string")


print("\n" + "=" * 60)
print("SECTION 2: SO3State — The State Object (Unit Quaternion)")
print("=" * 60)
print("""
A SO3State stores a unit quaternion (x, y, z, w).
Access fields as: state.x, state.y, state.z, state.w

UNIT QUATERNION REMINDER:
  x^2 + y^2 + z^2 + w^2 = 1  (always!)
  Identity (no rotation): (0, 0, 0, 1)
  180° around Z:          (0, 0, 1, 0)
  90° around Y:           (0, sin(π/4), 0, cos(π/4)) ≈ (0, 0.707, 0, 0.707)

C++ access: state->x, state->y, state->z, state->w
""")

# allocState
print("-- allocState() --")
s1 = space.allocState()
s2 = space.allocState()
check(s1 is not None, "allocState returns non-None")

# setIdentity
print("\n-- setIdentity() — no rotation —")
s1.setIdentity()
print(f"  After setIdentity(): x={s1.x}, y={s1.y}, z={s1.z}, w={s1.w}")
check(s1.x == 0.0 and s1.y == 0.0 and s1.z == 0.0 and s1.w == 1.0,
      "identity is (0,0,0,1)")

# norm check on identity
n = space.norm(s1)
print(f"  norm of identity = {n:.6f}  (should be 1.0)")
check(approx(n, 1.0), "identity has unit norm")

# setAxisAngle
print("\n-- setAxisAngle(ax, ay, az, angle) —")
print("  90° rotation around Z axis: setAxisAngle(0, 0, 1, π/2)")
s1.setAxisAngle(0, 0, 1, PI/2)
print(f"  x={s1.x:.6f}, y={s1.y:.6f}, z={s1.z:.6f}, w={s1.w:.6f}")
expected_z = math.sin(PI/4)
expected_w = math.cos(PI/4)
check(approx(s1.x, 0.0), "x ≈ 0")
check(approx(s1.z, expected_z, tol=1e-5), f"z ≈ sin(π/4) ≈ {expected_z:.4f}")
check(approx(s1.w, expected_w, tol=1e-5), f"w ≈ cos(π/4) ≈ {expected_w:.4f}")
check(approx(space.norm(s1), 1.0, tol=1e-5), "still unit norm after setAxisAngle")

print("\n  45° around Y: setAxisAngle(0, 1, 0, π/4)")
s2.setAxisAngle(0, 1, 0, PI/4)
print(f"  x={s2.x:.6f}, y={s2.y:.6f}, z={s2.z:.6f}, w={s2.w:.6f}")
check(approx(space.norm(s2), 1.0, tol=1e-5), "unit norm after 45° around Y")

# Direct property access
print("\n-- Direct x,y,z,w property access —")
s1.x = 0.0
s1.y = 0.0
s1.z = 0.0
s1.w = 1.0
print(f"  Set (0,0,0,1) manually: x={s1.x}, y={s1.y}, z={s1.z}, w={s1.w}")
check(space.satisfiesBounds(s1), "manually set (0,0,0,1) satisfies bounds")

# repr
print(f"\n  repr: {repr(s1)}")


print("\n" + "=" * 60)
print("SECTION 3: norm, satisfiesBounds, enforceBounds")
print("=" * 60)
print("""
norm(state)
    Computes sqrt(x^2 + y^2 + z^2 + w^2).
    A valid SO(3) state always has norm == 1.0.
    C++: space->norm(state)

satisfiesBounds(state)
    Returns True if the quaternion has unit norm (within tolerance).
    C++: space->satisfiesBounds(state)

enforceBounds(state)
    Normalises the quaternion to unit length (divides by norm).
    Call this if you set x,y,z,w manually to non-unit values.
    C++: space->enforceBounds(state)
""")

s = space.allocState()

# Valid unit quaternion
s.setIdentity()
print(f"-- identity: norm={space.norm(s):.6f}, satisfiesBounds={space.satisfiesBounds(s)}")
check(space.satisfiesBounds(s) is True, "identity satisfies bounds")

# Non-unit quaternion
s.x, s.y, s.z, s.w = 1.0, 1.0, 1.0, 1.0  # norm = 2, not valid
n = space.norm(s)
print(f"\n-- (1,1,1,1): norm={n:.6f}, satisfiesBounds={space.satisfiesBounds(s)}")
check(approx(n, 2.0, tol=1e-5), "norm of (1,1,1,1) == 2.0")
check(space.satisfiesBounds(s) is False, "(1,1,1,1) does NOT satisfy bounds")

# enforceBounds — normalises to unit length
space.enforceBounds(s)
n_after = space.norm(s)
print(f"  After enforceBounds: norm={n_after:.6f}")
print(f"  x={s.x:.4f}, y={s.y:.4f}, z={s.z:.4f}, w={s.w:.4f}")
check(approx(n_after, 1.0, tol=1e-5), "norm == 1.0 after enforceBounds")
check(space.satisfiesBounds(s) is True, "satisfiesBounds after enforceBounds")

# Each component should be 1/sqrt(4) = 0.5
check(approx(s.x, 0.5, tol=1e-5), "x normalised to 0.5")

space.freeState(s)


print("\n" + "=" * 60)
print("SECTION 4: copyState, cloneState, equalStates")
print("=" * 60)

src = space.allocState()
dst = space.allocState()
src.setAxisAngle(0, 0, 1, PI/3)  # 60° around Z

print("-- copyState(destination, source) --")
space.copyState(dst, src)
print(f"  src: x={src.x:.4f} y={src.y:.4f} z={src.z:.4f} w={src.w:.4f}")
print(f"  dst: x={dst.x:.4f} y={dst.y:.4f} z={dst.z:.4f} w={dst.w:.4f}")
check(approx(dst.x, src.x) and approx(dst.w, src.w), "copyState copies all components")

print("\n-- cloneState(source) --")
clone = space.cloneState(src)
check(approx(clone.x, src.x) and approx(clone.w, src.w), "cloneState correct")
space.freeState(clone)

print("\n-- equalStates() --")
e1 = space.allocState(); e1.setIdentity()
e2 = space.allocState(); e2.setIdentity()
e3 = space.allocState(); e3.setAxisAngle(1, 0, 0, PI/4)

print(f"  equalStates(identity, identity) = {space.equalStates(e1, e2)}")
print(f"  equalStates(identity, 45°rot)   = {space.equalStates(e1, e3)}")
check(space.equalStates(e1, e2) is True,  "identical states are equal")
check(space.equalStates(e1, e3) is False, "different states are not equal")

# q and -q: same rotation, not equalStates
e1.setIdentity()       # (0,0,0,1)
e2.x, e2.y, e2.z, e2.w = 0.0, 0.0, 0.0, -1.0  # (0,0,0,-1) = same rotation
print(f"\n  equalStates(q, -q) = {space.equalStates(e1, e2)}")
print(f"  distance(q, -q)    = {space.distance(e1, e2):.6f}")
print("  q and -q are the same rotation but NOT equalStates!")
check(space.equalStates(e1, e2) is False, "q and -q are not equalStates")

space.freeState(src)
space.freeState(dst)
space.freeState(e1)
space.freeState(e2)
space.freeState(e3)


print("\n" + "=" * 60)
print("SECTION 5: copyToReals, copyFromReals, getSerializationLength")
print("=" * 60)
print("""
copyToReals(source)    → returns [x, y, z, w] as 4-element list
copyFromReals(dst, reals) ← sets state from [x, y, z, w] list
getSerializationLength() → 32 bytes (4 × 8)
""")

s = space.allocState()
s.setAxisAngle(1, 0, 0, PI/2)  # 90° around X

reals = space.copyToReals(s)
print(f"  copyToReals: {[round(v, 4) for v in reals]}")
check(len(reals) == 4, "copyToReals returns 4-element list")
check(approx(reals[0], s.x), "reals[0] == x")
check(approx(reals[3], s.w), "reals[3] == w")

s2 = space.allocState()
s2.setIdentity()
space.copyFromReals(s2, reals)
print(f"  After copyFromReals: x={s2.x:.4f} y={s2.y:.4f} z={s2.z:.4f} w={s2.w:.4f}")
check(approx(s2.x, s.x) and approx(s2.w, s.w), "copyFromReals restores state")

slen = space.getSerializationLength()
print(f"\n  getSerializationLength() = {slen} bytes  (4 doubles × 8 bytes)")
check(slen == 32, "serialization length == 32")

space.freeState(s)
space.freeState(s2)


print("\n" + "=" * 60)
print("SECTION 6: distance — Angular Distance Between Orientations")
print("=" * 60)
print("""
distance(state1, state2)
    Returns the angular distance between two 3D orientations.
    Formula: acos(|q1·q2|)  — angle between quaternions on S^3.
    The |·| handles the q/-q double cover.
    Range: [0, π/2].

    C++: space->distance(state1, state2)

IMPORTANT:
  Maximum distance is π/2, NOT π.
  This is because q and -q are the same rotation,
  so the effective space is half of S^3.
""")

a = space.allocState()
b = space.allocState()

# Same rotation → distance = 0
a.setIdentity(); b.setIdentity()
d = space.distance(a, b)
print(f"-- d(identity, identity) = {d:.6f}  (should be 0)")
check(approx(d, 0.0), "same rotation has distance 0")

# 90° rotation around Z
a.setIdentity()
b.setAxisAngle(0, 0, 1, PI/2)
d = space.distance(a, b)
print(f"-- d(identity, 90° around Z) = {d:.6f}  (should be π/4 ≈ {PI/4:.6f})")
check(approx(d, PI/4, tol=1e-5), "90° rotation has distance π/4")

# 180° rotation → distance = π/2 (maximum)
a.setIdentity()
b.setAxisAngle(0, 0, 1, PI)
d = space.distance(a, b)
print(f"-- d(identity, 180° around Z) = {d:.6f}  (should be π/2 ≈ {PI/2:.6f})")
check(approx(d, PI/2, tol=1e-5), "180° rotation has distance π/2 (max)")

# q and -q → distance = 0 (same rotation)
a.setIdentity()    # (0,0,0,1)
b.x, b.y, b.z, b.w = 0.0, 0.0, 0.0, -1.0  # (0,0,0,-1) = same rotation
d = space.distance(a, b)
print(f"-- d(q, -q) = {d:.10f}  (same rotation → 0)")
check(d < 1e-6, "q and -q have distance 0")

# Symmetry
a.setAxisAngle(1, 0, 0, PI/3)
b.setAxisAngle(0, 1, 0, PI/4)
d_ab = space.distance(a, b)
d_ba = space.distance(b, a)
print(f"-- d(a,b) = {d_ab:.6f}, d(b,a) = {d_ba:.6f}  (symmetric)")
check(approx(d_ab, d_ba), "distance is symmetric")

space.freeState(a)
space.freeState(b)


print("\n" + "=" * 60)
print("SECTION 7: interpolate — SLERP")
print("=" * 60)
print("""
interpolate(from, to, t, result)
    Interpolates using SLERP (Spherical Linear Interpolation).
    t=0 → from,  t=1 → to,  t=0.5 → halfway.

    SLERP properties:
      - Constant angular velocity (smooth rotation)
      - Shortest path on S^3
      - Result is always a valid unit quaternion
      - No gimbal lock

    C++: space->interpolate(from, to, t, result);
""")

f = space.allocState()
t_state = space.allocState()
r = space.allocState()

# t=0 gives from, t=1 gives to
f.setIdentity()
t_state.setAxisAngle(0, 0, 1, PI/2)  # 90° around Z

space.interpolate(f, t_state, 0.0, r)
print(f"-- t=0.0: x={r.x:.4f} y={r.y:.4f} z={r.z:.4f} w={r.w:.4f}")
check(approx(r.x, f.x) and approx(r.w, f.w), "t=0 gives from")

space.interpolate(f, t_state, 1.0, r)
print(f"-- t=1.0: x={r.x:.4f} y={r.y:.4f} z={r.z:.4f} w={r.w:.4f}")
check(approx(r.x, t_state.x, tol=1e-4) and approx(r.w, t_state.w, tol=1e-4),
      "t=1 gives to")

# At t=0.5 result should be unit quaternion and halfway in rotation space
space.interpolate(f, t_state, 0.5, r)
d_from = space.distance(f, r)
d_to   = space.distance(r, t_state)
print(f"-- t=0.5: x={r.x:.4f} y={r.y:.4f} z={r.z:.4f} w={r.w:.4f}")
print(f"  distance from 'from': {d_from:.4f}")
print(f"  distance from 'to':   {d_to:.4f}")
print(f"  norm of result:       {space.norm(r):.6f}")
check(approx(space.norm(r), 1.0, tol=1e-5), "SLERP result is unit quaternion")
check(approx(d_from, d_to, tol=1e-4), "t=0.5 is equidistant from from and to")

# Quarter steps — distance should increase linearly
print("\n-- SLERP quarter steps (identity → 90° around Z) --")
total = space.distance(f, t_state)
for step_t in [0.0, 0.25, 0.5, 0.75, 1.0]:
    space.interpolate(f, t_state, step_t, r)
    d = space.distance(f, r)
    print(f"  t={step_t}: distance from start = {d:.4f}  (expected {step_t * total:.4f})")
    check(approx(space.norm(r), 1.0, tol=1e-5), f"t={step_t} result is unit quaternion")

space.freeState(f)
space.freeState(t_state)
space.freeState(r)


print("\n" + "=" * 60)
print("SECTION 8: Sampler")
print("=" * 60)
print("""
allocDefaultStateSampler() returns a SO3StateSampler.
All three methods use the tangent space approach to ensure
correct distribution on the 3-sphere S^3.

sampleUniform(state)         — any random 3D orientation
sampleUniformNear(state, near, distance)  — within distance radians of near
sampleGaussian(state, mean, stdDev)       — Gaussian around mean
""")

sampler = space.allocDefaultStateSampler()
print(f"  allocDefaultStateSampler() → {type(sampler).__name__}")

ss = space.allocState()
near_s = space.allocState()
near_s.setIdentity()

print("\n-- sampleUniform() × 5 --")
for i in range(5):
    sampler.sampleUniform(ss)
    n = space.norm(ss)
    valid = space.satisfiesBounds(ss)
    if i < 3:
        print(f"  sample {i}: x={ss.x:.3f} y={ss.y:.3f} z={ss.z:.3f} w={ss.w:.3f}  norm={n:.4f}")
    check(valid, f"uniform sample {i} satisfies bounds")

print("\n-- sampleUniformNear(near=identity, distance=0.3) × 3 --")
for i in range(3):
    sampler.sampleUniformNear(ss, near_s, 0.3)
    d = space.distance(ss, near_s)
    print(f"  sample {i}: dist from identity = {d:.4f}  (should be ≤ 0.3)")
    check(space.satisfiesBounds(ss), f"near sample {i} satisfies bounds")

print("\n-- sampleGaussian(mean=identity, stdDev=0.2) × 3 --")
for i in range(3):
    sampler.sampleGaussian(ss, near_s, 0.2)
    n = space.norm(ss)
    print(f"  sample {i}: norm={n:.4f}, satisfiesBounds={space.satisfiesBounds(ss)}")
    check(space.satisfiesBounds(ss), f"Gaussian sample {i} satisfies bounds")

space.freeState(ss)
space.freeState(near_s)


print("\n" + "=" * 60)
print("SECTION 9: Real-world — drone attitude planning")
print("=" * 60)
print("""
Use case: plan orientations for a drone between two attitudes.
Start: hovering level (identity rotation)
Goal:  tilted 30° forward and rotated 45° around vertical
""")

drone_space = pyompl.SO3StateSpace()
drone_space.setName("DroneAttitude")
drone_space.setup()

hover = drone_space.allocState()
hover.setIdentity()
print(f"  Hover attitude: x={hover.x}, y={hover.y}, z={hover.z}, w={hover.w}")

goal = drone_space.allocState()
goal.setAxisAngle(0, 1, 0, PI/6)   # 30° forward tilt around Y
print(f"  Goal  attitude: x={goal.x:.4f}, y={goal.y:.4f}, z={goal.z:.4f}, w={goal.w:.4f}")

total_dist = drone_space.distance(hover, goal)
print(f"\n  Total angular distance: {total_dist:.4f} rad = {math.degrees(total_dist):.1f}°")

wp = drone_space.allocState()
print(f"\n  Waypoints (SLERP from hover to goal):")
for step_t in [0.0, 0.25, 0.5, 0.75, 1.0]:
    drone_space.interpolate(hover, goal, step_t, wp)
    print(f"    t={step_t}: angle from start = {drone_space.distance(hover, wp):.4f} rad")
    check(drone_space.satisfiesBounds(wp), f"waypoint t={step_t} is valid")

drone_space.freeState(hover)
drone_space.freeState(goal)
drone_space.freeState(wp)


print("\n" + "=" * 60)
print("ALL SECTIONS COMPLETE")
print("=" * 60)
print("""
What we covered:
  SO3State:
    .x .y .z .w (properties), setIdentity, setAxisAngle, repr

  SO3StateSpace:
    constructor, setName/getName, getDimension,
    getMaximumExtent, getMeasure, printSettings,
    norm, satisfiesBounds, enforceBounds,
    copyState, cloneState, equalStates,
    copyToReals, copyFromReals, getSerializationLength,
    distance, interpolate (SLERP),
    allocState, freeState,
    allocDefaultStateSampler (sampleUniform, sampleUniformNear, sampleGaussian),
    setup, registerProjections

Key concepts:
  - Quaternions avoid gimbal lock
  - q and -q are the same rotation (double cover)
  - Max distance is π/2, not π
  - interpolate uses SLERP — constant angular velocity
  - enforceBounds normalises to unit quaternion

Next: examples/04_se2_state_space.py
""")