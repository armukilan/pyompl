"""
===============================================================
pyompl Example 04 — SE2StateSpace
===============================================================

WHAT IS SE(2)?
--------------
SE(2) = Special Euclidean Group in 2 dimensions.
It represents the COMPLETE POSE of a rigid body in a 2D plane:
  x   — horizontal position
  y   — vertical position
  yaw — heading (orientation angle)

Think of it as: where is the robot AND which way is it facing?

REAL-WORLD EXAMPLES:
  - A wheeled robot on a floor (x, y position + heading)
  - A car (position + steering direction)
  - A boat on water
  - Any mobile base that translates and rotates in 2D

INTERNAL STRUCTURE — COMPOUND STATE SPACE:
-------------------------------------------
SE(2) is composed of two subspaces:
  Subspace 0: RealVectorStateSpace(2)  weight=1.0  → stores (x, y)
  Subspace 1: SO2StateSpace()           weight=0.5  → stores yaw

This compound structure means:
  - Position is handled by Euclidean geometry (flat R^2)
  - Orientation is handled by SO(2) geometry (circular topology)
  - Each component uses the correct operations for its type

COMPOUND DISTANCE:
  d = 1.0 × euclidean(p1, p2) + 0.5 × arc_distance(yaw1, yaw2)

COMPOUND INTERPOLATION:
  position: linear interpolation
  yaw:      arc interpolation (shortest path on circle)

C++ HEADER: ompl/base/spaces/SE2StateSpace.h
C++ CLASS:  ompl::base::SE2StateSpace
INHERITS:   CompoundStateSpace → StateSpace

RUN THIS FILE:
    python examples/04_se2_state_space.py
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
print("SECTION 1: SE2StateSpace — Construction and Setup")
print("=" * 60)
print("""
SETUP SEQUENCE (always follow this order):
  1. Create the space
  2. Create RealVectorBounds for the workspace (x, y limits)
  3. Call space.setBounds(bounds)
  4. Call space.setup()

Yaw bounds are NOT set manually — SO(2) handles wrapping to (-π, π].

C++:
  auto space = std::make_shared<SE2StateSpace>();
  RealVectorBounds bounds(2);
  bounds.setLow(-10.0);
  bounds.setHigh(10.0);
  space->setBounds(bounds);
  space->setup();
""")

# Create space
space = pyompl.SE2StateSpace()

# Set position bounds (robot operates in 20×20 metre room)
bounds = pyompl.RealVectorBounds(2)
bounds.setLow(-10.0)
bounds.setHigh(10.0)
space.setBounds(bounds)
space.setup()

print(f"  getName()          = {space.getName()}")
print(f"  getDimension()     = {space.getDimension()}  (x + y + yaw = 3)")
print(f"  getMaximumExtent() = {space.getMaximumExtent():.4f}")
print(f"  getMeasure()       = {space.getMeasure():.4f}")

check(space.getDimension() == 3, "getDimension() == 3")

# setName
space.setName("RobotPose")
print(f"\n  After setName: {space.getName()}")
check("RobotPose" in space.getName(), "getName works")

# getBounds
b = space.getBounds()
print(f"\n  getBounds().getLow()  = {b.getLow()}")
print(f"  getBounds().getHigh() = {b.getHigh()}")
check(b.getLow() == [-10.0, -10.0], "position lower bounds correct")
check(b.getHigh() == [10.0, 10.0],  "position upper bounds correct")


print("\n" + "=" * 60)
print("SECTION 2: SE2State — Position and Orientation")
print("=" * 60)
print("""
An SE2State has THREE components:
  state.x    — horizontal position
  state.y    — vertical position
  state.yaw  — heading in radians, range (-π, π]

YAW CONVENTION:
  0.0   = facing East (positive X direction)
  π/2   = facing North (positive Y direction)
  π/-π  = facing West
  -π/2  = facing South

Access methods (equivalent C++ shown):
  state.x         ← state->getX()
  state.y         ← state->getY()
  state.yaw       ← state->getYaw()
  state.x = v     ← state->setX(v)
  state.y = v     ← state->setY(v)
  state.yaw = v   ← state->setYaw(v)
  state.setXY(x,y)← state->setXY(x, y)
""")

print("-- allocState() --")
s1 = space.allocState()
s2 = space.allocState()
check(s1 is not None, "allocState returns non-None")

# Set individual components
print("\n-- Setting x, y, yaw individually --")
s1.x   = 3.0
s1.y   = 4.0
s1.yaw = PI / 4   # 45°

print(f"  s1.x   = {s1.x}")
print(f"  s1.y   = {s1.y}")
print(f"  s1.yaw = {s1.yaw:.6f} rad = {math.degrees(s1.yaw):.1f}°")
check(s1.x == 3.0, "x set correctly")
check(s1.y == 4.0, "y set correctly")
check(approx(s1.yaw, PI/4), "yaw set correctly")

# getX, getY, getYaw explicit methods
print("\n-- getX(), getY(), getYaw() explicit methods --")
print(f"  getX()   = {s1.getX()}")
print(f"  getY()   = {s1.getY()}")
print(f"  getYaw() = {s1.getYaw():.6f}")
check(s1.getX() == s1.x, "getX() == .x property")
check(s1.getY() == s1.y, "getY() == .y property")

# setX, setY, setYaw explicit methods
print("\n-- setX(), setY(), setYaw() explicit methods --")
s1.setX(5.0)
s1.setY(6.0)
s1.setYaw(-PI/2)
print(f"  After setX(5), setY(6), setYaw(-π/2):")
print(f"  x={s1.x}, y={s1.y}, yaw={s1.yaw:.4f}")
check(s1.x == 5.0 and s1.y == 6.0, "setX/setY work")
check(approx(s1.yaw, -PI/2), "setYaw works")

# setXY — convenience
print("\n-- setXY(x, y) — set both at once --")
s1.setXY(1.0, 2.0)
print(f"  After setXY(1, 2): x={s1.x}, y={s1.y}, yaw={s1.yaw:.4f}")
check(s1.x == 1.0 and s1.y == 2.0, "setXY sets both correctly")
check(approx(s1.yaw, -PI/2), "setXY does not change yaw")

# repr
print(f"\n  repr: {repr(s1)}")

space.freeState(s1)
space.freeState(s2)


print("\n" + "=" * 60)
print("SECTION 3: satisfiesBounds and enforceBounds")
print("=" * 60)
print("""
satisfiesBounds(state)
    Returns True if x AND y are within position bounds
    AND yaw is in (-π, π].
    C++: space->satisfiesBounds(state)

enforceBounds(state)
    Clamps x, y to position bounds.
    Wraps yaw to (-π, π].
    C++: space->enforceBounds(state)
""")

s = space.allocState()

# Inside bounds
s.x, s.y, s.yaw = 5.0, 5.0, 1.0
print(f"-- (5,5,1.0):  satisfiesBounds = {space.satisfiesBounds(s)}")
check(space.satisfiesBounds(s) is True, "in-bounds state satisfies bounds")

# x out of bounds
s.x, s.y, s.yaw = 15.0, 5.0, 1.0
print(f"-- (15,5,1.0): satisfiesBounds = {space.satisfiesBounds(s)}")
check(space.satisfiesBounds(s) is False, "x=15 out of bounds")

# yaw out of range
s.x, s.y, s.yaw = 5.0, 5.0, 5.0   # yaw > π
print(f"-- (5,5,5.0):  satisfiesBounds = {space.satisfiesBounds(s)}")
check(space.satisfiesBounds(s) is False, "yaw=5.0 out of range")

# enforceBounds — clamp x, wrap yaw
s.x, s.y, s.yaw = 15.0, -20.0, 5.0
print(f"\n-- Before enforceBounds: x={s.x}, y={s.y}, yaw={s.yaw:.4f}")
space.enforceBounds(s)
print(f"   After  enforceBounds: x={s.x}, y={s.y}, yaw={s.yaw:.4f}")
check(s.x == 10.0, "x clamped to 10.0")
check(s.y == -10.0, "y clamped to -10.0")
check(space.satisfiesBounds(s), "state satisfies bounds after enforceBounds")

space.freeState(s)


print("\n" + "=" * 60)
print("SECTION 4: copyState, cloneState, equalStates")
print("=" * 60)

src = space.allocState()
dst = space.allocState()
src.x, src.y, src.yaw = 3.0, 4.0, PI/3

print("-- copyState(dst, src) --")
space.copyState(dst, src)
print(f"  src: ({src.x}, {src.y}, {src.yaw:.4f})")
print(f"  dst: ({dst.x}, {dst.y}, {dst.yaw:.4f})")
check(dst.x == src.x and dst.y == src.y and approx(dst.yaw, src.yaw),
      "copyState copies all 3 components")

print("\n-- cloneState(src) --")
clone = space.cloneState(src)
check(clone.x == src.x and approx(clone.yaw, src.yaw), "cloneState correct")
space.freeState(clone)

print("\n-- equalStates() --")
e1 = space.allocState(); e1.x, e1.y, e1.yaw = 1.0, 2.0, 0.5
e2 = space.allocState(); e2.x, e2.y, e2.yaw = 1.0, 2.0, 0.5
e3 = space.allocState(); e3.x, e3.y, e3.yaw = 1.0, 2.0, 0.6

print(f"  equalStates(same, same)      = {space.equalStates(e1, e2)}")
print(f"  equalStates(diff yaw)        = {space.equalStates(e1, e3)}")
check(space.equalStates(e1, e2) is True,  "identical states are equal")
check(space.equalStates(e1, e3) is False, "different yaw → not equal")

space.freeState(src)
space.freeState(dst)
space.freeState(e1)
space.freeState(e2)
space.freeState(e3)


print("\n" + "=" * 60)
print("SECTION 5: copyToReals, copyFromReals, getSerializationLength")
print("=" * 60)
print("""
copyToReals → returns [x, y, yaw] as 3-element list
copyFromReals ← sets state from [x, y, yaw]
getSerializationLength → 24 bytes (3 × 8)
""")

s = space.allocState()
s.x, s.y, s.yaw = 7.0, -3.0, PI/6

reals = space.copyToReals(s)
print(f"  copyToReals: {[round(v, 4) for v in reals]}")
check(len(reals) == 3, "copyToReals returns 3-element list")
check(approx(reals[0], 7.0), "reals[0] == x")
check(approx(reals[1], -3.0), "reals[1] == y")
check(approx(reals[2], PI/6), "reals[2] == yaw")

s2 = space.allocState()
space.copyFromReals(s2, reals)
print(f"  copyFromReals: x={s2.x:.4f} y={s2.y:.4f} yaw={s2.yaw:.4f}")
check(approx(s2.x, s.x) and approx(s2.yaw, s.yaw), "copyFromReals restores state")

print(f"\n  getSerializationLength() = {space.getSerializationLength()} bytes")
check(space.getSerializationLength() == 24, "serialization length == 24")

space.freeState(s)
space.freeState(s2)


print("\n" + "=" * 60)
print("SECTION 6: distance — Compound Weighted Distance")
print("=" * 60)
print("""
distance(state1, state2)
    Compound weighted distance:
      d = 1.0 × euclidean(p1, p2) + 0.5 × arc_distance(yaw1, yaw2)

    C++: space->distance(state1, state2)

    WHY WEIGHTED?
      Position and orientation are in different units (metres vs radians).
      The weights (1.0 for position, 0.5 for yaw) balance their contributions.
      These weights were set in the SE2StateSpace constructor.

    SUBSPACE WEIGHTS:
      getSubspaceWeight(0) = 1.0  (position)
      getSubspaceWeight(1) = 0.5  (yaw)
""")

print("-- Subspace structure --")
print(f"  getSubspaceCount()       = {space.getSubspaceCount()}")
print(f"  getSubspaceWeight(0)     = {space.getSubspaceWeight(0)}  (position)")
print(f"  getSubspaceWeight(1)     = {space.getSubspaceWeight(1)}  (yaw)")
print(f"  isLocked()               = {space.isLocked()}")
check(space.getSubspaceCount() == 2, "2 subspaces")
check(space.getSubspaceWeight(0) == 1.0, "position weight == 1.0")
check(space.getSubspaceWeight(1) == 0.5, "yaw weight == 0.5")
check(space.isLocked() is True, "SE2 space is locked")

a = space.allocState()
b = space.allocState()

# Same state → distance = 0
a.x, a.y, a.yaw = 0.0, 0.0, 0.0
b.x, b.y, b.yaw = 0.0, 0.0, 0.0
d = space.distance(a, b)
print(f"\n-- d(same, same) = {d:.6f}  (should be 0)")
check(approx(d, 0.0), "distance 0 for same state")

# Pure position difference, same yaw
a.x, a.y, a.yaw = 0.0, 0.0, 0.0
b.x, b.y, b.yaw = 3.0, 4.0, 0.0   # euclidean dist = 5
d = space.distance(a, b)
print(f"\n-- d((0,0,0°), (3,4,0°)) = {d:.6f}")
print(f"  Position dist = 5.0 × weight 1.0 = 5.0")
print(f"  Yaw dist      = 0.0 × weight 0.5 = 0.0")
print(f"  Total = 5.0 (approx)")
check(approx(d, 5.0, tol=1e-4), "pure position distance correct")

# Pure yaw difference, same position
a.x, a.y, a.yaw = 0.0, 0.0, 0.0
b.x, b.y, b.yaw = 0.0, 0.0, PI/2   # 90° yaw diff
yaw_arc = PI/2
d = space.distance(a, b)
expected = 0.5 * yaw_arc
print(f"\n-- d((0,0,0°), (0,0,90°)) = {d:.6f}")
print(f"  Position dist = 0.0 × 1.0 = 0.0")
print(f"  Yaw dist      = π/2 × 0.5 = {expected:.4f}")
check(approx(d, expected, tol=1e-4), "pure yaw distance correct")

# Combined
a.x, a.y, a.yaw = 0.0, 0.0, 0.0
b.x, b.y, b.yaw = 3.0, 4.0, PI/2
d = space.distance(a, b)
expected = 5.0 + 0.5 * (PI/2)
print(f"\n-- d((0,0,0°), (3,4,90°)) = {d:.6f}  (expected ≈ {expected:.4f})")
check(approx(d, expected, tol=1e-4), "combined position+yaw distance correct")

# Yaw wrapping in distance (SO2 part handles this)
a.x, a.y, a.yaw =  0.0, 0.0,  3.0  # near π
b.x, b.y, b.yaw =  0.0, 0.0, -3.0  # near -π
# Short yaw arc ≈ 0.28, not 6.0
d = space.distance(a, b)
print(f"\n-- Yaw wrapping: d((0,0,3.0), (0,0,-3.0)) = {d:.4f}")
print(f"  Short yaw arc ≈ {min(6.0, 2*PI-6.0):.4f} × 0.5 = {0.5*min(6.0,2*PI-6.0):.4f}")
check(d < 1.0, "yaw wrapping gives small distance, not 3.0")

# Symmetry
a.x, a.y, a.yaw = 1.0, 2.0, 0.5
b.x, b.y, b.yaw = 4.0, 6.0, 1.5
d_ab = space.distance(a, b)
d_ba = space.distance(b, a)
print(f"\n-- Symmetry: d(a,b)={d_ab:.4f}, d(b,a)={d_ba:.4f}")
check(approx(d_ab, d_ba), "distance is symmetric")

space.freeState(a)
space.freeState(b)


print("\n" + "=" * 60)
print("SECTION 7: interpolate")
print("=" * 60)
print("""
interpolate(from, to, t, result)
    Interpolates position LINEARLY and yaw via SHORTEST ARC.

    result.x   = from.x   + t*(to.x   - from.x)
    result.y   = from.y   + t*(to.y   - from.y)
    result.yaw = SO2 arc interpolation at t

    C++: space->interpolate(from, to, t, result);
""")

f = space.allocState()
t_s = space.allocState()
r = space.allocState()

f.x, f.y, f.yaw = 0.0, 0.0, 0.0
t_s.x, t_s.y, t_s.yaw = 10.0, 10.0, PI/2

space.interpolate(f, t_s, 0.0, r)
print(f"-- t=0.0: x={r.x:.2f}, y={r.y:.2f}, yaw={r.yaw:.4f}")
check(r.x == 0.0 and r.y == 0.0, "t=0 gives from position")

space.interpolate(f, t_s, 1.0, r)
print(f"-- t=1.0: x={r.x:.2f}, y={r.y:.2f}, yaw={r.yaw:.4f}")
check(approx(r.x, 10.0) and approx(r.y, 10.0), "t=1 gives to position")

space.interpolate(f, t_s, 0.5, r)
print(f"-- t=0.5: x={r.x:.2f}, y={r.y:.2f}, yaw={r.yaw:.4f}")
check(approx(r.x, 5.0) and approx(r.y, 5.0), "t=0.5 gives midpoint position")
check(approx(r.yaw, PI/4, tol=1e-4), "t=0.5 gives midpoint yaw")
check(space.satisfiesBounds(r), "midpoint state within bounds")

# Yaw wrapping during interpolation
f.x, f.y, f.yaw   =  0.0, 0.0,  3.0   # near π
t_s.x, t_s.y, t_s.yaw =  0.0, 0.0, -3.0  # near -π
space.interpolate(f, t_s, 0.5, r)
print(f"\n-- Yaw wrap: from=3.0, to=-3.0, t=0.5 → yaw={r.yaw:.4f}")
print(f"  Yaw goes the short way (through ±π), not the long way (through 0)")
check(abs(r.yaw) > PI/2, "yaw midpoint is near ±π, not near 0")

print("\n-- Quarter steps: (0,0,0°) → (8,6,90°) --")
f.x, f.y, f.yaw   = 0.0, 0.0, 0.0
t_s.x, t_s.y, t_s.yaw = 8.0, 6.0, PI/2
for step_t in [0.0, 0.25, 0.5, 0.75, 1.0]:
    space.interpolate(f, t_s, step_t, r)
    print(f"  t={step_t}: ({r.x:.1f}, {r.y:.1f}, {math.degrees(r.yaw):.0f}°)")
    check(space.satisfiesBounds(r), f"interpolated state at t={step_t} in bounds")

space.freeState(f)
space.freeState(t_s)
space.freeState(r)


# print("\n" + "=" * 60)
# print("SECTION 8: Sampler")
# print("=" * 60)
# print("""
# allocDefaultStateSampler() returns a CompoundStateSampler that
# independently samples:
#   (x, y) uniformly from the position bounds
#   yaw    uniformly from (-π, π]
# """)

# sampler = space.allocDefaultStateSampler()
# print(f"  Sampler type: {type(sampler).__name__}")

# ss = space.allocState()
# print("\n-- sampleUniform() × 5 --")
# for i in range(5):
#     sampler.sampleUniform(ss)
#     valid = space.satisfiesBounds(ss)
#     if i < 3:
#         print(f"  x={ss.x:.2f}, y={ss.y:.2f}, yaw={math.degrees(ss.yaw):.1f}°  valid={valid}")
#     check(valid, f"sample {i} satisfies bounds")

# space.freeState(ss)
print("\n" + "=" * 60)
print("SECTION 8: Sampler")
print("=" * 60)
print("""
SE2StateSpace provides sampleUniform, sampleUniformNear, sampleGaussian
directly on the space object. These internally use the CompoundStateSampler
which samples (x,y) from position bounds and yaw from (-π, π].
""")

ss = space.allocState()

print("-- sampleUniform() x 5 --")
for i in range(5):
    space.sampleUniform(ss)
    valid = space.satisfiesBounds(ss)
    if i < 3:
        print(f"  x={ss.x:.2f}, y={ss.y:.2f}, yaw={math.degrees(ss.yaw):.1f}  valid={valid}")
    check(valid, f"sample {i} satisfies bounds")

near_s = space.allocState()
near_s.x, near_s.y, near_s.yaw = 0.0, 0.0, 0.0

print("\n-- sampleUniformNear(near=(0,0,0), distance=2.0) x 3 --")
for i in range(3):
    space.sampleUniformNear(ss, near_s, 2.0)
    print(f"  x={ss.x:.2f}, y={ss.y:.2f}, yaw={math.degrees(ss.yaw):.1f}")
    check(space.satisfiesBounds(ss), f"near sample {i} in bounds")

print("\n-- sampleGaussian(mean=(0,0,0), stdDev=1.0) x 3 --")
for i in range(3):
    space.sampleGaussian(ss, near_s, 1.0)
    print(f"  x={ss.x:.2f}, y={ss.y:.2f}, yaw={math.degrees(ss.yaw):.1f}")
    check(space.satisfiesBounds(ss), f"Gaussian sample {i} in bounds")

space.freeState(ss)
space.freeState(near_s)


print("\n" + "=" * 60)
print("SECTION 9: printState, printSettings")
print("=" * 60)

ps = space.allocState()
ps.x, ps.y, ps.yaw = 2.5, -3.0, PI/3
state_str = space.printState(ps)
print(f"  printState(): '{state_str.strip()}'")
check(len(state_str) > 0, "printState returns non-empty string")
space.freeState(ps)

cfg = space.printSettings()
print(f"\n  printSettings():\n{cfg}")
check(len(cfg) > 0, "printSettings returns non-empty string")


print("\n" + "=" * 60)
print("SECTION 10: Real-world — Mobile Robot Path")
print("=" * 60)
print("""
Use case: a differential-drive robot navigating through a warehouse.
  - 40×40 metre warehouse floor
  - Robot starts at loading dock (south-west corner, facing East)
  - Goal: shelf at north-east corner, facing North
""")

warehouse = pyompl.SE2StateSpace()
warehouse.setName("WarehouseFloor")
wb = pyompl.RealVectorBounds(2)
wb.setLow(0.0)
wb.setHigh(40.0)
warehouse.setBounds(wb)
warehouse.setup()

start = warehouse.allocState()
start.x, start.y, start.yaw = 1.0, 1.0, 0.0   # facing East

goal = warehouse.allocState()
goal.x, goal.y, goal.yaw = 39.0, 39.0, PI/2    # facing North

dist = warehouse.distance(start, goal)
print(f"  Start: ({start.x}, {start.y}, {math.degrees(start.yaw):.0f}°)")
print(f"  Goal:  ({goal.x}, {goal.y}, {math.degrees(goal.yaw):.0f}°)")
print(f"  Compound distance: {dist:.4f}")
check(warehouse.satisfiesBounds(start), "start is within warehouse")
check(warehouse.satisfiesBounds(goal),  "goal is within warehouse")

wp = warehouse.allocState()
print(f"\n  Straight-line waypoints:")
for step_t in [0.0, 0.2, 0.4, 0.6, 0.8, 1.0]:
    warehouse.interpolate(start, goal, step_t, wp)
    print(f"  t={step_t}: ({wp.x:.1f}, {wp.y:.1f}, {math.degrees(wp.yaw):.0f}°)")
    check(warehouse.satisfiesBounds(wp), f"waypoint t={step_t} in bounds")

warehouse.freeState(start)
warehouse.freeState(goal)
warehouse.freeState(wp)


print("\n" + "=" * 60)
print("ALL SECTIONS COMPLETE")
print("=" * 60)
print("""
What we covered:

  SE2State:
    .x .y .yaw (properties)
    getX, getY, getYaw, setX, setY, setYaw, setXY
    repr

  SE2StateSpace:
    constructor, setName/getName
    setBounds, getBounds
    getDimension, getMaximumExtent, getMeasure
    satisfiesBounds, enforceBounds
    copyState, cloneState, equalStates
    copyToReals, copyFromReals
    getSerializationLength
    distance  (weighted compound: position + yaw)
    interpolate (linear position + arc yaw)
    allocState, freeState
    allocDefaultStateSampler
    printState, printSettings
    setup, registerProjections
    getSubspaceCount, getSubspace, getSubspaceWeight, isLocked

Key concepts:
  - SE(2) = R^2 × SO(2): position + orientation
  - Compound space: each subspace uses its own geometry
  - Distance is WEIGHTED: 1.0×position + 0.5×yaw
  - Yaw wraps around (SO(2) handles angle topology)
  - setBounds only sets position bounds (yaw is always bounded)

Next: examples/05_space_information.py
""")