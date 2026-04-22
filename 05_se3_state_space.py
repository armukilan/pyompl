"""
===============================================================
pyompl Example 05 — SE3StateSpace
===============================================================

WHAT IS SE(3)?
--------------
SE(3) = Special Euclidean Group in 3 dimensions.
It represents the COMPLETE POSE of a rigid body in 3D space:
  x, y, z    — 3D position
  rotation   — 3D orientation (unit quaternion)

Total degrees of freedom: 6 (3 position + 3 rotation)
This is why robot end-effectors have "6-DOF" — SE(3) is their
natural state space.

INTERNAL STRUCTURE — COMPOUND STATE SPACE:
-------------------------------------------
SE(3) is composed of two subspaces:
  Subspace 0: RealVectorStateSpace(3)  weight=1.0  → (x, y, z)
  Subspace 1: SO3StateSpace()           weight=1.0  → quaternion

KEY DIFFERENCES FROM SE(2):
  SE(2) = x, y, yaw          (2D, 3 DOF, yaw is SO(2))
  SE(3) = x, y, z, quaternion (3D, 6 DOF, rotation is SO(3))

  In SE(3), BOTH subspaces have weight=1.0 (unlike SE(2) where yaw=0.5).

COMPOUND DISTANCE:
  d = 1.0 × euclidean(p1, p2)  +  1.0 × acos(|q1·q2|)

COMPOUND INTERPOLATION:
  position: linear (straight line in 3D)
  rotation: SLERP  (smooth quaternion interpolation)

USE FOR:
  - 6-DOF robot arm end-effector
  - Drone pose (position + attitude)
  - Object manipulation in 3D
  - Surgical robot tip pose

C++ HEADER: ompl/base/spaces/SE3StateSpace.h
C++ CLASS:  ompl::base::SE3StateSpace
INHERITS:   CompoundStateSpace → StateSpace

RUN THIS FILE:
    python examples/05_se3_state_space.py
===============================================================
"""

import math
import pyompl

PI = math.pi

def check(condition, msg):
    print(f"  {'PASS' if condition else 'FAIL'} — {msg}")

def approx(a, b, tol=1e-5):
    return abs(a - b) < tol


print("=" * 60)
print("SECTION 1: SE3StateSpace — Construction and Setup")
print("=" * 60)
print("""
SETUP SEQUENCE:
  1. space = SE3StateSpace()
  2. bounds = RealVectorBounds(3)   ← 3D because x,y,z
  3. bounds.setLow / setHigh
  4. space.setBounds(bounds)
  5. space.setup()

C++:
  auto space = std::make_shared<SE3StateSpace>();
  RealVectorBounds bounds(3);
  bounds.setLow(-5.0);
  bounds.setHigh(5.0);
  space->setBounds(bounds);
  space->setup();
""")

space = pyompl.SE3StateSpace()

bounds = pyompl.RealVectorBounds(3)
bounds.setLow(-5.0)
bounds.setHigh(5.0)
space.setBounds(bounds)
space.setup()

print(f"  getName()          = {space.getName()}")
print(f"  getDimension()     = {space.getDimension()}  (3 position + 3 rotation)")
print(f"  getMaximumExtent() = {space.getMaximumExtent():.4f}")
print(f"  getMeasure()       = {space.getMeasure():.4f}")
check(space.getDimension() == 6, "getDimension() == 6")

space.setName("EndEffector")
check("EndEffector" in space.getName(), "setName/getName work")

b = space.getBounds()
print(f"\n  getBounds().getLow()  = {b.getLow()}")
print(f"  getBounds().getHigh() = {b.getHigh()}")
check(b.getLow() == [-5.0, -5.0, -5.0], "position lower bounds correct")
check(b.getHigh() == [5.0, 5.0, 5.0],   "position upper bounds correct")

print(f"\n  getSubspaceCount()       = {space.getSubspaceCount()}")
print(f"  getSubspaceWeight(0)     = {space.getSubspaceWeight(0)}  (position)")
print(f"  getSubspaceWeight(1)     = {space.getSubspaceWeight(1)}  (rotation)")
print(f"  isLocked()               = {space.isLocked()}")
check(space.getSubspaceCount() == 2, "2 subspaces")
check(space.getSubspaceWeight(0) == 1.0, "position weight == 1.0")
check(space.getSubspaceWeight(1) == 1.0, "rotation weight == 1.0")


print("\n" + "=" * 60)
print("SECTION 2: SE3State — Position and Rotation")
print("=" * 60)
print("""
An SE3State has SEVEN components total:
  x, y, z        — 3D position
  rotation       — SO3State (quaternion: qx, qy, qz, qw)

Access position:  state.x, state.y, state.z
Set position:     state.setXYZ(x, y, z)
Access rotation:  state.rotation()  → returns SO3State reference
Set rotation:     state.rotation().setAxisAngle(ax, ay, az, angle)
                  state.rotation().setIdentity()

C++ equivalents:
  state->getX()             state->setX(v)
  state->getY()             state->setY(v)
  state->getZ()             state->setZ(v)
  state->setXYZ(x, y, z)
  state->rotation()         ← returns SO3StateSpace::StateType&
  state->rotation().setAxisAngle(ax, ay, az, angle)
  state->rotation().setIdentity()
""")

s1 = space.allocState()
s2 = space.allocState()

print("-- allocState() --")
check(s1 is not None, "allocState returns non-None")

print("\n-- Setting position with .x .y .z properties --")
s1.x = 1.0
s1.y = 2.0
s1.z = 3.0
print(f"  x={s1.x}, y={s1.y}, z={s1.z}")
check(s1.x == 1.0 and s1.y == 2.0 and s1.z == 3.0, "x,y,z set correctly")

print("\n-- setXYZ(x, y, z) — set all position at once --")
s1.setXYZ(4.0, 5.0, 6.0)  # Will be clamped to bounds later
print(f"  After setXYZ(4,5,6): x={s1.x}, y={s1.y}, z={s1.z}")
check(s1.x == 4.0 and s1.z == 6.0, "setXYZ works")

print("\n-- getX, getY, getZ explicit methods --")
s1.setXYZ(1.5, 2.5, 3.5)
print(f"  getX()={s1.getX()}, getY()={s1.getY()}, getZ()={s1.getZ()}")
check(s1.getX() == 1.5, "getX() correct")

print("\n-- setX, setY, setZ explicit methods --")
s1.setX(-1.0); s1.setY(-2.0); s1.setZ(-3.0)
print(f"  After setX(-1) setY(-2) setZ(-3): ({s1.x}, {s1.y}, {s1.z})")
check(s1.x == -1.0 and s1.y == -2.0 and s1.z == -3.0, "setX/setY/setZ work")

print("\n-- rotation() — access the SO3State (quaternion) --")
s1.rotation().setIdentity()
rot = s1.rotation()
print(f"  After setIdentity(): qx={rot.x}, qy={rot.y}, qz={rot.z}, qw={rot.w}")
check(rot.x == 0.0 and rot.w == 1.0, "identity rotation is (0,0,0,1)")

print("\n-- rotation().setAxisAngle(ax, ay, az, angle) --")
s1.rotation().setAxisAngle(0, 0, 1, PI/2)   # 90° around Z
rot = s1.rotation()
print(f"  90° around Z: qx={rot.x:.4f} qy={rot.y:.4f} qz={rot.z:.4f} qw={rot.w:.4f}")
check(approx(rot.z, math.sin(PI/4)), "qz ≈ sin(π/4)")
check(approx(rot.w, math.cos(PI/4)), "qw ≈ cos(π/4)")

print(f"\n  repr: {repr(s1)}")


print("\n" + "=" * 60)
print("SECTION 3: satisfiesBounds and enforceBounds")
print("=" * 60)
print("""
satisfiesBounds checks BOTH:
  1. x, y, z are within position bounds [-5, 5]
  2. quaternion has unit norm

enforceBounds:
  1. Clamps x, y, z to position bounds
  2. Normalises the quaternion to unit length

C++: space->satisfiesBounds(state)
C++: space->enforceBounds(state)
""")

s = space.allocState()

# Valid state
s.setXYZ(1.0, 2.0, -3.0)
s.rotation().setIdentity()
print(f"-- (1,2,-3, identity): satisfiesBounds = {space.satisfiesBounds(s)}")
check(space.satisfiesBounds(s) is True, "valid state satisfies bounds")

# Position out of bounds
s.setXYZ(10.0, 0.0, 0.0)
s.rotation().setIdentity()
print(f"-- (10,0,0, identity): satisfiesBounds = {space.satisfiesBounds(s)}")
check(space.satisfiesBounds(s) is False, "x=10 out of bounds")

# Non-unit quaternion
s.setXYZ(1.0, 1.0, 1.0)
s.rotation().x = 1.0; s.rotation().y = 1.0
s.rotation().z = 1.0; s.rotation().w = 1.0
print(f"-- (1,1,1, non-unit quat): satisfiesBounds = {space.satisfiesBounds(s)}")
check(space.satisfiesBounds(s) is False, "non-unit quaternion fails bounds")

# enforceBounds — clamp position AND normalise quaternion
s.setXYZ(10.0, -8.0, 7.0)
s.rotation().x = 1.0; s.rotation().y = 1.0
s.rotation().z = 1.0; s.rotation().w = 1.0
print(f"\n-- Before enforceBounds: x={s.x}, y={s.y}, z={s.z}")
space.enforceBounds(s)
print(f"   After  enforceBounds: x={s.x}, y={s.y}, z={s.z}")
print(f"   Quaternion norm: {approx(s.rotation().x**2 + s.rotation().y**2 + s.rotation().z**2 + s.rotation().w**2, 1.0)}")
check(s.x == 5.0, "x clamped to 5.0")
check(s.y == -5.0, "y clamped to -5.0")
check(s.z == 5.0, "z clamped to 5.0")
check(space.satisfiesBounds(s), "state satisfies bounds after enforceBounds")

space.freeState(s)


print("\n" + "=" * 60)
print("SECTION 4: copyState, cloneState, equalStates")
print("=" * 60)

src = space.allocState()
dst = space.allocState()
src.setXYZ(1.0, 2.0, 3.0)
src.rotation().setAxisAngle(0, 1, 0, PI/4)

print("-- copyState(dst, src) --")
space.copyState(dst, src)
print(f"  src: ({src.x},{src.y},{src.z}) qw={src.rotation().w:.4f}")
print(f"  dst: ({dst.x},{dst.y},{dst.z}) qw={dst.rotation().w:.4f}")
check(dst.x == src.x and dst.y == src.y and dst.z == src.z,
      "position copied correctly")
check(approx(dst.rotation().w, src.rotation().w), "rotation copied correctly")

print("\n-- cloneState(src) --")
clone = space.cloneState(src)
check(clone.x == src.x and approx(clone.rotation().w, src.rotation().w),
      "cloneState correct")
space.freeState(clone)

print("\n-- equalStates() --")
e1 = space.allocState(); e1.setXYZ(1.0,2.0,3.0); e1.rotation().setIdentity()
e2 = space.allocState(); e2.setXYZ(1.0,2.0,3.0); e2.rotation().setIdentity()
e3 = space.allocState(); e3.setXYZ(1.0,2.0,4.0); e3.rotation().setIdentity()

print(f"  equalStates(same, same) = {space.equalStates(e1, e2)}")
print(f"  equalStates(diff z)     = {space.equalStates(e1, e3)}")
check(space.equalStates(e1, e2) is True,  "identical states are equal")
check(space.equalStates(e1, e3) is False, "different z → not equal")

space.freeState(src); space.freeState(dst)
space.freeState(e1); space.freeState(e2); space.freeState(e3)


print("\n" + "=" * 60)
print("SECTION 5: copyToReals, copyFromReals, getSerializationLength")
print("=" * 60)
print("""
copyToReals  → returns [x, y, z, qx, qy, qz, qw] (7-element list)
copyFromReals← sets state from [x, y, z, qx, qy, qz, qw]
serialization length = 7 × 8 = 56 bytes
""")

s = space.allocState()
s.setXYZ(1.0, 2.0, 3.0)
s.rotation().setAxisAngle(0, 0, 1, PI/2)

reals = space.copyToReals(s)
print(f"  copyToReals: {[round(v,4) for v in reals]}")
check(len(reals) == 7, "copyToReals returns 7-element list")
check(approx(reals[0], 1.0) and approx(reals[1], 2.0), "x,y correct")

s2 = space.allocState()
space.copyFromReals(s2, reals)
print(f"  copyFromReals: x={s2.x:.2f} y={s2.y:.2f} z={s2.z:.2f} qw={s2.rotation().w:.4f}")
check(approx(s2.x, s.x) and approx(s2.rotation().w, s.rotation().w),
      "copyFromReals restores state")

slen = space.getSerializationLength()
print(f"\n  getSerializationLength() = {slen} bytes  (7 doubles × 8 bytes)")
check(slen == 56, "serialization length == 56")

space.freeState(s); space.freeState(s2)


print("\n" + "=" * 60)
print("SECTION 6: distance — Compound Distance")
print("=" * 60)
print("""
distance(state1, state2)
    d = 1.0 × euclidean((x1,y1,z1),(x2,y2,z2))
      + 1.0 × acos(|q1·q2|)

Both subspaces have equal weight 1.0 in SE(3).
(Compare SE(2) where yaw weight is 0.5)

C++: space->distance(state1, state2)
""")

a = space.allocState()
b = space.allocState()

# Same state
a.setXYZ(0.0,0.0,0.0); a.rotation().setIdentity()
b.setXYZ(0.0,0.0,0.0); b.rotation().setIdentity()
d = space.distance(a, b)
print(f"-- d(same, same) = {d:.6f}  (should be 0)")
check(approx(d, 0.0), "distance 0 for same state")

# Pure position difference
a.setXYZ(0.0,0.0,0.0); a.rotation().setIdentity()
b.setXYZ(3.0,4.0,0.0); b.rotation().setIdentity()
d = space.distance(a, b)
print(f"-- d((0,0,0), (3,4,0), same rot) = {d:.4f}  (position dist=5.0)")
check(approx(d, 5.0), "pure position distance correct (3-4-5 triangle)")

# Pure rotation difference
a.setXYZ(0.0,0.0,0.0); a.rotation().setIdentity()
b.setXYZ(0.0,0.0,0.0); b.rotation().setAxisAngle(0,0,1,PI/2)
d = space.distance(a, b)
expected_rot = PI/4   # 90° → distance = π/4 in SO(3)
print(f"-- d(same pos, 90° rotation) = {d:.4f}  (expected ≈ {expected_rot:.4f})")
check(approx(d, expected_rot, tol=1e-4), "pure rotation distance correct")

# Combined
a.setXYZ(0.0,0.0,0.0); a.rotation().setIdentity()
b.setXYZ(3.0,4.0,0.0); b.rotation().setAxisAngle(0,0,1,PI/2)
d = space.distance(a, b)
expected = 5.0 + PI/4
print(f"-- d(combined pos+rot) = {d:.4f}  (expected ≈ {expected:.4f})")
check(approx(d, expected, tol=1e-4), "combined distance correct")

# Symmetry
a.setXYZ(1.0,2.0,3.0); a.rotation().setAxisAngle(1,0,0,PI/3)
b.setXYZ(-1.0,0.0,1.0); b.rotation().setAxisAngle(0,1,0,PI/6)
d_ab = space.distance(a, b)
d_ba = space.distance(b, a)
print(f"-- Symmetry: d(a,b)={d_ab:.4f}, d(b,a)={d_ba:.4f}")
check(approx(d_ab, d_ba), "distance is symmetric")

space.freeState(a); space.freeState(b)


print("\n" + "=" * 60)
print("SECTION 7: interpolate")
print("=" * 60)
print("""
interpolate(from, to, t, result)
  Position: linear interpolation
  Rotation: SLERP

  C++: space->interpolate(from, to, t, result);
""")

f = space.allocState()
t_s = space.allocState()
r = space.allocState()

f.setXYZ(0.0, 0.0, 0.0); f.rotation().setIdentity()
t_s.setXYZ(4.0, 0.0, 0.0); t_s.rotation().setAxisAngle(0,0,1,PI/2)

space.interpolate(f, t_s, 0.0, r)
print(f"-- t=0.0: x={r.x:.2f}, qw={r.rotation().w:.4f}")
check(r.x == 0.0 and approx(r.rotation().w, 1.0), "t=0 gives from")

space.interpolate(f, t_s, 1.0, r)
print(f"-- t=1.0: x={r.x:.2f}, qw={r.rotation().w:.4f}")
check(approx(r.x, 4.0), "t=1 gives to position")

space.interpolate(f, t_s, 0.5, r)
print(f"-- t=0.5: x={r.x:.2f}, y={r.y:.2f}, z={r.z:.2f}, qw={r.rotation().w:.4f}")
check(approx(r.x, 2.0), "t=0.5 gives midpoint position")
check(space.satisfiesBounds(r), "midpoint satisfies bounds")

# Quarter steps
print(f"\n-- Quarter steps from (0,0,0,identity) to (4,0,0,90°Z): --")
for step_t in [0.0, 0.25, 0.5, 0.75, 1.0]:
    space.interpolate(f, t_s, step_t, r)
    print(f"  t={step_t}: x={r.x:.2f}, qw={r.rotation().w:.4f}")
    check(space.satisfiesBounds(r), f"t={step_t} state in bounds")

space.freeState(f); space.freeState(t_s); space.freeState(r)


print("\n" + "=" * 60)
print("SECTION 8: Sampling")
print("=" * 60)
print("""
sampleUniform(state)
    Samples (x,y,z) uniformly from bounds AND rotation from SO(3).
    C++: space->allocDefaultStateSampler()->sampleUniform(state);

sampleUniformNear(state, near, distance)
    Samples near a given state.
    C++: space->allocDefaultStateSampler()->sampleUniformNear(...);

sampleGaussian(state, mean, stdDev)
    Samples from Gaussian around mean.
    C++: space->allocDefaultStateSampler()->sampleGaussian(...);
""")

ss = space.allocState()
near_s = space.allocState()
near_s.setXYZ(0.0, 0.0, 0.0); near_s.rotation().setIdentity()

print("-- sampleUniform() × 5 --")
for i in range(5):
    space.sampleUniform(ss)
    valid = space.satisfiesBounds(ss)
    if i < 3:
        print(f"  x={ss.x:.2f} y={ss.y:.2f} z={ss.z:.2f}  qw={ss.rotation().w:.3f}  valid={valid}")
    check(valid, f"sample {i} satisfies bounds")

print("\n-- sampleUniformNear(near=origin+identity, distance=2.0) × 3 --")
for i in range(3):
    space.sampleUniformNear(ss, near_s, 2.0)
    print(f"  x={ss.x:.2f} y={ss.y:.2f} z={ss.z:.2f}  dist={space.distance(ss, near_s):.3f}")
    check(space.satisfiesBounds(ss), f"near sample {i} in bounds")

print("\n-- sampleGaussian(mean=origin+identity, stdDev=1.0) × 3 --")
for i in range(3):
    space.sampleGaussian(ss, near_s, 1.0)
    print(f"  x={ss.x:.2f} y={ss.y:.2f} z={ss.z:.2f}  valid={space.satisfiesBounds(ss)}")
    check(space.satisfiesBounds(ss), f"Gaussian sample {i} in bounds")

space.freeState(ss); space.freeState(near_s)


print("\n" + "=" * 60)
print("SECTION 9: printState, printSettings, setup, registerProjections")
print("=" * 60)

ps = space.allocState()
ps.setXYZ(1.0, 2.0, -1.0); ps.rotation().setAxisAngle(0,1,0,PI/3)
state_str = space.printState(ps)
print(f"  printState():\n  {state_str.strip()}")
check(len(state_str) > 0, "printState non-empty")
space.freeState(ps)

cfg = space.printSettings()
print(f"\n  printSettings():\n{cfg}")
check(len(cfg) > 0, "printSettings non-empty")

fresh = pyompl.SE3StateSpace()
b2 = pyompl.RealVectorBounds(3); b2.setLow(-1.0); b2.setHigh(1.0)
fresh.setBounds(b2); fresh.setup()
print("  setup() completed"); check(True, "setup() runs cleanly")
fresh.registerProjections()
print("  registerProjections() completed"); check(True, "registerProjections() runs cleanly")


print("\n" + "=" * 60)
print("SECTION 10: Real-world — 6-DOF Robot Arm End-Effector")
print("=" * 60)
print("""
Use case: plan a path for the tip of a robot arm from
a home pose to a target pose, respecting workspace limits.
""")

arm_space = pyompl.SE3StateSpace()
arm_space.setName("ArmEndEffector")
arm_bounds = pyompl.RealVectorBounds(3)
arm_bounds.setLow(0, -0.8); arm_bounds.setHigh(0, 0.8)   # x: -0.8 to 0.8 m
arm_bounds.setLow(1, -0.8); arm_bounds.setHigh(1, 0.8)   # y: -0.8 to 0.8 m
arm_bounds.setLow(2,  0.0); arm_bounds.setHigh(2, 1.2)   # z: 0 to 1.2 m
arm_space.setBounds(arm_bounds)
arm_space.setup()

home = arm_space.allocState()
home.setXYZ(0.0, 0.0, 0.5)
home.rotation().setIdentity()
print(f"  Home: ({home.x}, {home.y}, {home.z}), identity rotation")

target = arm_space.allocState()
target.setXYZ(0.5, 0.3, 0.8)
target.rotation().setAxisAngle(0, 1, 0, PI/4)
print(f"  Target: ({target.x}, {target.y}, {target.z}), 45° around Y")

dist = arm_space.distance(home, target)
print(f"\n  End-effector distance: {dist:.4f}")
check(arm_space.satisfiesBounds(home),   "home pose in workspace")
check(arm_space.satisfiesBounds(target), "target pose in workspace")

wp = arm_space.allocState()
print(f"\n  Interpolated waypoints:")
for step_t in [0.0, 0.2, 0.4, 0.6, 0.8, 1.0]:
    arm_space.interpolate(home, target, step_t, wp)
    print(f"  t={step_t}: ({wp.x:.2f},{wp.y:.2f},{wp.z:.2f}) qw={wp.rotation().w:.3f}")
    check(arm_space.satisfiesBounds(wp), f"waypoint t={step_t} in workspace")

arm_space.freeState(home)
arm_space.freeState(target)
arm_space.freeState(wp)


print("\n" + "=" * 60)
print("ALL SECTIONS COMPLETE")
print("=" * 60)
print("""
What we covered:

  SE3State:
    .x .y .z (properties), getX/getY/getZ, setX/setY/setZ, setXYZ
    .rotation() → SO3State reference
    rotation().setIdentity(), rotation().setAxisAngle()
    rotation().x .y .z .w
    repr

  SE3StateSpace:
    constructor, setName/getName
    setBounds, getBounds
    getDimension (=6), getMaximumExtent, getMeasure
    satisfiesBounds, enforceBounds
    copyState, cloneState, equalStates
    copyToReals (7 elements), copyFromReals
    getSerializationLength (=56)
    distance (weighted: position + rotation, both weight=1.0)
    interpolate (linear position + SLERP rotation)
    allocState, freeState
    sampleUniform, sampleUniformNear, sampleGaussian
    printState, printSettings, setup, registerProjections
    getSubspaceCount, getSubspaceWeight, isLocked

Key concepts:
  - SE(3) = R^3 × SO(3): position + 3D orientation
  - Both subspaces have weight=1.0 (unlike SE(2) where yaw=0.5)
  - rotation() returns a live SO3State reference — modify directly
  - SLERP for rotation, linear for position
  - 7 values total: x,y,z + qx,qy,qz,qw

Next: examples/06_time_state_space.py
""")