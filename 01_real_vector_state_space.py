# """
# ===============================================================
# pyompl Example 01 — RealVectorStateSpace
# ===============================================================

# WHAT IS THIS?
# -------------
# In OMPL, a "State Space" defines the space of all possible
# configurations a robot can be in. The simplest one is
# RealVectorStateSpace — a flat n-dimensional space like R^2 or R^3.

# Think of it as: if your robot has 3 joints, each joint has a
# position. The state space is the space of all joint positions.
# A "state" is one specific configuration (one point in that space).

# C++ EQUIVALENT (what pyompl wraps):
#     #include <ompl/base/spaces/RealVectorStateSpace.h>
#     using namespace ompl::base;

# RUN THIS FILE:
#     python examples/01_real_vector_state_space.py
# ===============================================================
# """

# import math
# import pyompl

# print("=" * 60)
# print("PART 1: RealVectorBounds")
# print("=" * 60)

# """
# WHAT IS RealVectorBounds?
# --------------------------
# Before you can use a state space, you must define its boundaries —
# i.e., the minimum and maximum value each dimension can take.

# For a 2D robot in a room from -5 to 5 meters on both axes:
#     low  = [-5, -5]
#     high = [ 5,  5]

# C++ equivalent:
#     ompl::base::RealVectorBounds bounds(2);
#     bounds.setLow(-5.0);
#     bounds.setHigh(5.0);
# """

# # Create bounds for a 2-dimensional space
# bounds_2d = pyompl.RealVectorBounds(2)

# # Set all lower bounds to -5.0 (same value for all dims)
# # C++: bounds.setLow(-5.0);
# bounds_2d.setLow(-5.0)

# # Set all upper bounds to 5.0 (same value for all dims)
# # C++: bounds.setHigh(5.0);
# bounds_2d.setHigh(5.0)

# print(f"Lower bounds: {bounds_2d.getLow()}")   # [-5.0, -5.0]
# print(f"Upper bounds: {bounds_2d.getHigh()}")  # [ 5.0,  5.0]
# print(f"Dimensions:   {bounds_2d.getDimension()}")  # 2

# # You can also set bounds per dimension individually
# # Useful when each joint has a different range
# # C++: bounds.setLow(0, -3.14);  bounds.setLow(1, 0.0);
# bounds_joint = pyompl.RealVectorBounds(2)
# bounds_joint.setLow(0, -3.14)   # joint 0: -pi to pi
# bounds_joint.setLow(1,  0.0)    # joint 1: 0 to 2
# bounds_joint.setHigh(0, 3.14)
# bounds_joint.setHigh(1, 2.0)

# print(f"\nPer-dimension lower bounds: {bounds_joint.getLow()}")   # [-3.14, 0.0]
# print(f"Per-dimension upper bounds: {bounds_joint.getHigh()}")   # [ 3.14, 2.0]

# # check() validates the bounds — raises exception if low >= high
# # C++: bounds.check();
# try:
#     bounds_2d.check()
#     print("\nbounds_2d.check() passed — bounds are valid")
# except Exception as e:
#     print(f"bounds_2d.check() failed: {e}")

# # Example of invalid bounds
# bad_bounds = pyompl.RealVectorBounds(2)
# bad_bounds.setLow(5.0)   # low > high — this is wrong
# bad_bounds.setHigh(-5.0)
# try:
#     bad_bounds.check()
# except Exception as e:
#     print(f"bad_bounds.check() raised: {type(e).__name__} (expected)")


# print("\n" + "=" * 60)
# print("PART 2: Creating a RealVectorStateSpace")
# print("=" * 60)

# """
# WHAT IS RealVectorStateSpace?
# ------------------------------
# This is the state space itself. It represents R^n — an n-dimensional
# continuous space. You create it, set its bounds, and then use it
# to allocate states, compute distances, interpolate, etc.

# Think of it as the "coordinate system" your robot lives in.

# C++ equivalent:
#     auto space = std::make_shared<ompl::base::RealVectorStateSpace>(3);
#     space->setBounds(bounds);
# """

# # Create a 3D state space (e.g., a drone at position x, y, z)
# # C++: auto space = std::make_shared<RealVectorStateSpace>(3);
# space = pyompl.RealVectorStateSpace(3)

# # Set bounds: drone can fly in a 10x10x10 meter cube
# bounds_3d = pyompl.RealVectorBounds(3)
# bounds_3d.setLow(0.0)
# bounds_3d.setHigh(10.0)

# # C++: space->setBounds(bounds);
# space.setBounds(bounds_3d)

# # Get the number of dimensions
# # C++: space->getDimension()
# print(f"Dimensions: {space.getDimension()}")  # 3

# # Give the space a name (optional, useful for debugging)
# # C++: space->setName("DroneSpace");
# space.setName("DroneSpace")
# print(f"Space name: {space.getName()}")  # DroneSpace

# # Get the maximum possible distance between any two states
# # For a cube [0,10]^3, it's the diagonal: 10 * sqrt(3) ≈ 17.32
# # C++: space->getMaximumExtent()
# max_extent = space.getMaximumExtent()
# print(f"Max extent: {max_extent:.4f}")           # ≈ 17.3205
# print(f"Expected:   {10.0 * math.sqrt(3):.4f}") # 17.3205

# # Get bounds back from the space
# # C++: space->getBounds()
# retrieved_bounds = space.getBounds()
# print(f"Retrieved low:  {retrieved_bounds.getLow()}")   # [0.0, 0.0, 0.0]
# print(f"Retrieved high: {retrieved_bounds.getHigh()}")  # [10.0, 10.0, 10.0]

# # repr
# print(f"repr: {repr(space)}")  # <RealVectorStateSpace dim=3>


# print("\n" + "=" * 60)
# print("PART 3: Allocating and Using States")
# print("=" * 60)

# """
# WHAT IS A STATE?
# -----------------
# A state is one specific configuration — one point in the state space.
# In RealVectorStateSpace, a state is just an array of doubles.

# You MUST allocate states through the space (not with 'new' or similar).
# The space manages memory. Always free states when done.

# C++ equivalent:
#     ompl::base::State *state = space->allocState();
#     state->as<RealVectorStateSpace::StateType>()->values[0] = 5.0;
#     space->freeState(state);
# """

# # Allocate a new state
# # C++: auto *s = space->allocState()->as<RealVectorStateSpace::StateType>();
# s1 = space.allocState()

# # Set values using index (like array access)
# # C++: s->values[0] = 5.0; s->values[1] = 3.0; s->values[2] = 7.0;
# s1[0] = 5.0   # x = 5
# s1[1] = 3.0   # y = 3
# s1[2] = 7.0   # z = 7

# print(f"s1[0] = {s1[0]}")  # 5.0
# print(f"s1[1] = {s1[1]}")  # 3.0
# print(f"s1[2] = {s1[2]}")  # 7.0

# # Allocate a second state
# s2 = space.allocState()
# s2[0] = 1.0
# s2[1] = 1.0
# s2[2] = 1.0


# print("\n" + "=" * 60)
# print("PART 4: Distance Between States")
# print("=" * 60)

# """
# WHAT IS distance()?
# --------------------
# Computes the Euclidean distance between two states.
# For RealVectorStateSpace this is the standard L2 norm.

# Formula: sqrt( (x2-x1)^2 + (y2-y1)^2 + (z2-z1)^2 )

# C++ equivalent:
#     double d = space->distance(s1, s2);
# """

# # s1 = (5, 3, 7), s2 = (1, 1, 1)
# # distance = sqrt((5-1)^2 + (3-1)^2 + (7-1)^2) = sqrt(16+4+36) = sqrt(56)
# dist = space.distance(s1, s2)
# expected = math.sqrt((5-1)**2 + (3-1)**2 + (7-1)**2)
# print(f"distance(s1, s2) = {dist:.6f}")      # 7.483314
# print(f"expected         = {expected:.6f}")  # 7.483314


# print("\n" + "=" * 60)
# print("PART 5: Interpolation Between States")
# print("=" * 60)

# """
# WHAT IS interpolate()?
# -----------------------
# Given two states 'from' and 'to', compute an intermediate state
# at fraction t (where t=0 gives 'from', t=1 gives 'to', t=0.5 is midpoint).

# This is used by planners to move along a straight line between states.

# C++ equivalent:
#     ompl::base::State *result = space->allocState();
#     space->interpolate(from, to, t, result);
# """

# result = space.allocState()

# # t=0.0 → should equal s1
# space.interpolate(s1, s2, 0.0, result)
# print(f"t=0.0: ({result[0]:.1f}, {result[1]:.1f}, {result[2]:.1f})")  # (5, 3, 7)

# # t=1.0 → should equal s2
# space.interpolate(s1, s2, 1.0, result)
# print(f"t=1.0: ({result[0]:.1f}, {result[1]:.1f}, {result[2]:.1f})")  # (1, 1, 1)

# # t=0.5 → midpoint
# space.interpolate(s1, s2, 0.5, result)
# print(f"t=0.5: ({result[0]:.1f}, {result[1]:.1f}, {result[2]:.1f})")  # (3, 2, 4)

# # t=0.25 → 25% of the way from s1 to s2
# space.interpolate(s1, s2, 0.25, result)
# print(f"t=0.25: ({result[0]:.2f}, {result[1]:.2f}, {result[2]:.2f})")  # (4, 2.5, 5.5)


# print("\n" + "=" * 60)
# print("PART 6: Checking and Enforcing Bounds")
# print("=" * 60)

# """
# WHAT IS satisfiesBounds()?
# ---------------------------
# Returns True if a state is within the defined bounds, False otherwise.
# Planners use this to check if a sampled state is valid.

# C++ equivalent:
#     bool ok = space->satisfiesBounds(state);

# WHAT IS enforceBounds()?
# -------------------------
# Clamps the state values to lie within the bounds.
# If s[0] = 15.0 and high[0] = 10.0, it becomes 10.0.

# C++ equivalent:
#     space->enforceBounds(state);
# """

# # Create a state inside bounds
# s_inside = space.allocState()
# s_inside[0] = 5.0
# s_inside[1] = 5.0
# s_inside[2] = 5.0
# print(f"Inside bounds?  {space.satisfiesBounds(s_inside)}")  # True

# # Create a state outside bounds
# s_outside = space.allocState()
# s_outside[0] = 15.0   # > 10 (out of bounds)
# s_outside[1] =  5.0
# s_outside[2] = -1.0   # < 0  (out of bounds)
# print(f"Outside bounds? {space.satisfiesBounds(s_outside)}")  # False

# # Enforce bounds — clamps to [0, 10]
# space.enforceBounds(s_outside)
# print(f"After enforceBounds: ({s_outside[0]:.1f}, {s_outside[1]:.1f}, {s_outside[2]:.1f})")
# # (10.0, 5.0, 0.0)
# print(f"Now in bounds?  {space.satisfiesBounds(s_outside)}")  # True


# print("\n" + "=" * 60)
# print("PART 7: Freeing States (Memory Management)")
# print("=" * 60)

# """
# WHAT IS freeState()?
# ---------------------
# States allocated with allocState() must be freed when you're done.
# This is equivalent to C++ delete, but done through the space.

# In C++ OMPL, this is critical. In pyompl, the Python GC helps,
# but explicitly freeing is good practice and matches C++ behavior.

# C++ equivalent:
#     space->freeState(s1);
# """

# # Free all the states we allocated
# space.freeState(s1)
# space.freeState(s2)
# space.freeState(result)
# space.freeState(s_inside)
# space.freeState(s_outside)
# print("All states freed successfully.")


# print("\n" + "=" * 60)
# print("PART 8: Real-world usage pattern")
# print("=" * 60)

# """
# HOW YOU'D ACTUALLY USE THIS:
# ------------------------------
# In a real motion planning scenario:
# 1. Create the space with correct dimensions for your robot
# 2. Set bounds to your robot's joint limits or workspace limits
# 3. Pass the space to SpaceInformation (next example)
# 4. SpaceInformation + ProblemDefinition → Planner → solve()

# This example just shows Step 1 and 2.
# """

# # Example: a 6-DOF robot arm
# # Each joint has different angle limits (in radians)
# dof = 6
# arm_space = pyompl.RealVectorStateSpace(dof)
# arm_space.setName("RobotArm6DOF")

# joint_limits_low  = [-3.14, -1.57, -3.14, -3.14, -1.57, -3.14]
# joint_limits_high = [ 3.14,  1.57,  3.14,  3.14,  1.57,  3.14]

# arm_bounds = pyompl.RealVectorBounds(dof)
# for i in range(dof):
#     arm_bounds.setLow(i,  joint_limits_low[i])
#     arm_bounds.setHigh(i, joint_limits_high[i])

# arm_space.setBounds(arm_bounds)

# print(f"Robot arm space: {arm_space.getName()}, DOF={arm_space.getDimension()}")
# print(f"Joint low  limits: {[round(v, 2) for v in arm_bounds.getLow()]}")
# print(f"Joint high limits: {[round(v, 2) for v in arm_bounds.getHigh()]}")

# # Allocate a "home" configuration (all joints at 0)
# home = arm_space.allocState()
# for i in range(dof):
#     home[i] = 0.0

# # Allocate a "goal" configuration
# goal_config = arm_space.allocState()
# goal_config[0] =  1.5
# goal_config[1] =  0.5
# goal_config[2] = -1.0
# goal_config[3] =  2.0
# goal_config[4] =  0.3
# goal_config[5] = -0.7

# dist_home_to_goal = arm_space.distance(home, goal_config)
# print(f"\nDistance from home to goal config: {dist_home_to_goal:.4f}")

# # Check if goal is within joint limits
# print(f"Goal config in bounds: {arm_space.satisfiesBounds(goal_config)}")

# arm_space.freeState(home)
# arm_space.freeState(goal_config)

# print("\n" + "=" * 60)
# print("Done! All examples completed successfully.")
# print("Next: examples/02_se2_state_space.py")
# print("=" * 60)

"""
===============================================================
pyompl Example 01 — RealVectorBounds + RealVectorStateSpace
===============================================================

WHAT IS OMPL?
-------------
OMPL (Open Motion Planning Library) is a C++ library for robot
motion planning. It provides state spaces, samplers, and planners.
pyompl exposes all of this to Python via pybind11.

WHAT IS A STATE SPACE?
-----------------------
A state space defines all possible configurations a robot can be in.
For a robot arm with 3 joints, the state space is 3-dimensional —
one number per joint angle.

WHAT IS RealVectorStateSpace?
------------------------------
The simplest state space: R^n, i.e., n real numbers.
- A point robot in 2D:    dim=2, state = (x, y)
- A drone in 3D:          dim=3, state = (x, y, z)
- A 6-DOF robot arm:      dim=6, state = (j1, j2, j3, j4, j5, j6)

C++ HEADER: ompl/base/spaces/RealVectorStateSpace.h
C++ CLASS:  ompl::base::RealVectorStateSpace

RUN THIS FILE:
    python examples/01_real_vector_state_space.py
===============================================================
"""

import math
import pyompl

PASS = "  PASS"
FAIL = "  FAIL"

def check(condition, msg):
    print(f"  {'PASS' if condition else 'FAIL'} — {msg}")

print("=" * 60)
print("SECTION 1: RealVectorBounds")
print("=" * 60)
print("""
WHAT IS RealVectorBounds?
--------------------------
Before using a state space you must define its boundaries —
the minimum and maximum value each dimension can take.

C++ header: ompl/base/spaces/RealVectorBounds.h
C++ class:  ompl::base::RealVectorBounds

Members:
  low  — std::vector<double>  (public, lower bound per dim)
  high — std::vector<double>  (public, upper bound per dim)
""")

# ---------------------------------------------------------------
# Constructor
# C++: ompl::base::RealVectorBounds bounds(3);
# ---------------------------------------------------------------
print("-- Constructor --")
b3 = pyompl.RealVectorBounds(3)
print(f"  RealVectorBounds(3) created")
print(f"  getDimension() = {b3.getDimension()}")
check(b3.getDimension() == 3, "getDimension() == 3")

# ---------------------------------------------------------------
# setLow(value) — set ALL lower bounds to same value
# C++: bounds.setLow(-5.0);
# ---------------------------------------------------------------
print("\n-- setLow(value) — set all lower bounds uniformly --")
b3.setLow(-5.0)
print(f"  After setLow(-5.0): getLow() = {b3.getLow()}")
check(b3.getLow() == [-5.0, -5.0, -5.0], "all lower bounds = -5.0")

# ---------------------------------------------------------------
# setHigh(value) — set ALL upper bounds to same value
# C++: bounds.setHigh(5.0);
# ---------------------------------------------------------------
print("\n-- setHigh(value) — set all upper bounds uniformly --")
b3.setHigh(5.0)
print(f"  After setHigh(5.0): getHigh() = {b3.getHigh()}")
check(b3.getHigh() == [5.0, 5.0, 5.0], "all upper bounds = 5.0")

# ---------------------------------------------------------------
# setLow(index, value) — set lower bound for one dimension
# C++: bounds.setLow(0, -3.14);
# ---------------------------------------------------------------
print("\n-- setLow(index, value) — per-dimension lower bound --")
b3.setLow(0, -3.14)
print(f"  After setLow(0, -3.14): getLow() = {[round(x,4) for x in b3.getLow()]}")
check(abs(b3.getLow()[0] - (-3.14)) < 1e-9, "low[0] == -3.14")
check(b3.getLow()[1] == -5.0, "low[1] still == -5.0")

# ---------------------------------------------------------------
# setHigh(index, value) — set upper bound for one dimension
# C++: bounds.setHigh(2, 2.0);
# ---------------------------------------------------------------
print("\n-- setHigh(index, value) — per-dimension upper bound --")
b3.setHigh(2, 2.0)
print(f"  After setHigh(2, 2.0): getHigh() = {b3.getHigh()}")
check(b3.getHigh()[2] == 2.0, "high[2] == 2.0")

# ---------------------------------------------------------------
# resize(size) — change number of dimensions
# C++: bounds.resize(5);
# ---------------------------------------------------------------
print("\n-- resize(size) — change number of dimensions --")
b_resize = pyompl.RealVectorBounds(2)
b_resize.setLow(-1.0)
b_resize.setHigh(1.0)
b_resize.resize(4)
print(f"  After resize(4): getDimension() = {b_resize.getDimension()}")
check(b_resize.getDimension() == 4, "getDimension() == 4 after resize")

# ---------------------------------------------------------------
# getVolume() — product of (high[i] - low[i])
# C++: bounds.getVolume()
# ---------------------------------------------------------------
print("\n-- getVolume() — volume of the bounded region --")
bv = pyompl.RealVectorBounds(3)
bv.setLow(0.0)
bv.setHigh(2.0)
vol = bv.getVolume()
print(f"  Bounds [0,2]^3, getVolume() = {vol}")
check(abs(vol - 8.0) < 1e-9, "volume of [0,2]^3 == 8.0")

# ---------------------------------------------------------------
# getDifference() — high[i] - low[i] per dimension
# C++: bounds.getDifference()
# ---------------------------------------------------------------
print("\n-- getDifference() — high[i] - low[i] per dimension --")
diff = bv.getDifference()
print(f"  getDifference() = {diff}")
check(diff == [2.0, 2.0, 2.0], "difference == [2,2,2]")

# ---------------------------------------------------------------
# check() — validate bounds
# C++: bounds.check()
# Throws if low[i] >= high[i]
# ---------------------------------------------------------------
print("\n-- check() — validate bounds --")
b_valid = pyompl.RealVectorBounds(2)
b_valid.setLow(-1.0)
b_valid.setHigh(1.0)
try:
    b_valid.check()
    print("  Valid bounds: check() passed")
    check(True, "valid bounds pass check()")
except Exception as e:
    check(False, f"valid bounds should not raise: {e}")

b_bad = pyompl.RealVectorBounds(2)
b_bad.setLow(5.0)
b_bad.setHigh(-5.0)   # low > high — invalid
try:
    b_bad.check()
    check(False, "invalid bounds should have raised")
except Exception:
    print("  Invalid bounds (low > high): check() raised as expected")
    check(True, "invalid bounds raise exception")


print("\n" + "=" * 60)
print("SECTION 2: Creating a RealVectorStateSpace")
print("=" * 60)
print("""
C++ header: ompl/base/spaces/RealVectorStateSpace.h
C++ class:  ompl::base::RealVectorStateSpace

Inherits from: StateSpace (ompl::base::StateSpace)

A RealVectorStateSpace is the coordinate system your robot lives in.
You create it, configure bounds, then pass it to SpaceInformation.
""")

# ---------------------------------------------------------------
# Constructor
# C++: auto space = std::make_shared<RealVectorStateSpace>(3);
# ---------------------------------------------------------------
print("-- Constructor --")
space = pyompl.RealVectorStateSpace(3)
print(f"  RealVectorStateSpace(3) created")
print(f"  getDimension() = {space.getDimension()}")
check(space.getDimension() == 3, "getDimension() == 3")

# ---------------------------------------------------------------
# setName / getName
# C++: space->setName("DroneSpace");  space->getName();
# ---------------------------------------------------------------
print("\n-- setName / getName --")
space.setName("DroneSpace")
print(f"  getName() = {space.getName()}")
check("DroneSpace" in space.getName(), "getName() contains 'DroneSpace'")

# ---------------------------------------------------------------
# setBounds(bounds) — from RealVectorBounds object
# C++: space->setBounds(bounds);
# ---------------------------------------------------------------
print("\n-- setBounds(bounds) — set bounds from RealVectorBounds --")
bounds = pyompl.RealVectorBounds(3)
bounds.setLow(0.0)
bounds.setHigh(10.0)
space.setBounds(bounds)
print(f"  setBounds([0,10]^3) done")

# ---------------------------------------------------------------
# setBounds(low, high) — scalar shortcut
# C++: space->setBounds(-1.0, 1.0);
# ---------------------------------------------------------------
print("\n-- setBounds(low, high) — scalar shortcut --")
space2 = pyompl.RealVectorStateSpace(2)
space2.setBounds(-1.0, 1.0)
b2 = space2.getBounds()
print(f"  getBounds().getLow()  = {b2.getLow()}")
print(f"  getBounds().getHigh() = {b2.getHigh()}")
check(b2.getLow() == [-1.0, -1.0], "scalar setBounds low correct")
check(b2.getHigh() == [1.0, 1.0],  "scalar setBounds high correct")

# ---------------------------------------------------------------
# getBounds()
# C++: space->getBounds()
# ---------------------------------------------------------------
print("\n-- getBounds() — retrieve current bounds --")
b = space.getBounds()
print(f"  getLow()  = {b.getLow()}")
print(f"  getHigh() = {b.getHigh()}")
check(b.getLow()  == [0.0,  0.0,  0.0],  "low  == [0,0,0]")
check(b.getHigh() == [10.0, 10.0, 10.0], "high == [10,10,10]")

# ---------------------------------------------------------------
# getMaximumExtent()
# C++: space->getMaximumExtent()
# For [0,10]^3: diagonal = 10 * sqrt(3)
# ---------------------------------------------------------------
print("\n-- getMaximumExtent() — max possible distance between states --")
ext = space.getMaximumExtent()
expected_ext = 10.0 * math.sqrt(3)
print(f"  getMaximumExtent() = {ext:.6f}")
print(f"  Expected (10*sqrt(3)) = {expected_ext:.6f}")
check(abs(ext - expected_ext) < 1e-9, "getMaximumExtent() == 10*sqrt(3)")

# ---------------------------------------------------------------
# getMeasure()
# C++: space->getMeasure()
# = product of (high[i] - low[i]) = 10^3 = 1000
# ---------------------------------------------------------------
print("\n-- getMeasure() — volume of state space --")
measure = space.getMeasure()
print(f"  getMeasure() = {measure}")
check(abs(measure - 1000.0) < 1e-9, "getMeasure() == 1000 for [0,10]^3")

# ---------------------------------------------------------------
# printSettings()
# C++: space->printSettings(std::cout)
# ---------------------------------------------------------------
print("\n-- printSettings() — space configuration as string --")
settings = space.printSettings()
print(f"  printSettings() output:\n{settings}")
check(len(settings) > 0, "printSettings() returns non-empty string")

# ---------------------------------------------------------------
# __repr__
# ---------------------------------------------------------------
print("\n-- repr --")
r = repr(space)
print(f"  repr(space) = {r}")
check("3" in r, "repr contains dim=3")


print("\n" + "=" * 60)
print("SECTION 3: addDimension — building space incrementally")
print("=" * 60)
print("""
Instead of specifying dimensions in the constructor, you can
start with dim=0 and add dimensions one by one.
This is useful when dimensions have different bounds or names.

C++: space->addDimension(minBound, maxBound);
C++: space->addDimension(name, minBound, maxBound);
After adding all dimensions: space->setup();
""")

dynamic_space = pyompl.RealVectorStateSpace(0)
dynamic_space.addDimension("x",     -10.0, 10.0)
dynamic_space.addDimension("y",     -10.0, 10.0)
dynamic_space.addDimension("theta", -3.14, 3.14)
dynamic_space.setup()

print(f"  After 3 addDimension calls: getDimension() = {dynamic_space.getDimension()}")
check(dynamic_space.getDimension() == 3, "getDimension() == 3")

# ---------------------------------------------------------------
# getDimensionName / getDimensionIndex / setDimensionName
# C++: space->getDimensionName(0)
# C++: space->getDimensionIndex("x")
# C++: space->setDimensionName(0, "px")
# ---------------------------------------------------------------
print("\n-- getDimensionName / getDimensionIndex / setDimensionName --")
name0 = dynamic_space.getDimensionName(0)
idx   = dynamic_space.getDimensionIndex("y")
print(f"  getDimensionName(0) = '{name0}'")
print(f"  getDimensionIndex('y') = {idx}")
check(name0 == "x", "getDimensionName(0) == 'x'")
check(idx   == 1,   "getDimensionIndex('y') == 1")

dynamic_space.setDimensionName(0, "px")
print(f"  After setDimensionName(0, 'px'): getDimensionName(0) = '{dynamic_space.getDimensionName(0)}'")
check(dynamic_space.getDimensionName(0) == "px", "setDimensionName works")

missing_idx = dynamic_space.getDimensionIndex("nonexistent")
print(f"  getDimensionIndex('nonexistent') = {missing_idx}")
check(missing_idx == -1, "missing dimension returns -1")


print("\n" + "=" * 60)
print("SECTION 4: State Allocation and Values")
print("=" * 60)
print("""
WHAT IS A STATE?
-----------------
A state is one specific configuration — one point in the space.
In RealVectorStateSpace, a state is just an array of doubles
(one per dimension).

You MUST allocate states through the space. The space manages
memory. Always free states when done.

C++ allocate: space->allocState()->as<RealVectorStateSpace::StateType>()
C++ free:     space->freeState(state);
C++ access:   state->values[0]  or  (*state)[0]

In Python: state[0], state[1], ...
""")

# ---------------------------------------------------------------
# allocState() / freeState()
# C++: auto *s = space->allocState()->as<RealVectorStateSpace::StateType>();
# ---------------------------------------------------------------
print("-- allocState() / freeState() --")
s1 = space.allocState()
s2 = space.allocState()
print(f"  Allocated s1 and s2")
check(s1 is not None, "allocState() returns non-None")

# ---------------------------------------------------------------
# __setitem__ / __getitem__
# C++: s->values[0] = 5.0;  double v = s->values[0];
# ---------------------------------------------------------------
print("\n-- state[i] = value  /  state[i] read --")
s1[0] = 3.0
s1[1] = 7.0
s1[2] = 1.5
print(f"  s1 = ({s1[0]}, {s1[1]}, {s1[2]})")
check(s1[0] == 3.0, "s1[0] == 3.0")
check(s1[1] == 7.0, "s1[1] == 7.0")
check(s1[2] == 1.5, "s1[2] == 1.5")

s2[0] = 0.0
s2[1] = 0.0
s2[2] = 0.0

# ---------------------------------------------------------------
# copyState(destination, source)
# C++: space->copyState(dst, src);
# Copies all values from src into dst. Memory must not overlap.
# ---------------------------------------------------------------
print("\n-- copyState(destination, source) --")
s_copy = space.allocState()
space.copyState(s_copy, s1)
print(f"  After copyState(s_copy, s1): s_copy = ({s_copy[0]}, {s_copy[1]}, {s_copy[2]})")
check(s_copy[0] == s1[0] and s_copy[1] == s1[1] and s_copy[2] == s1[2],
      "copyState copies all values correctly")
space.freeState(s_copy)

# ---------------------------------------------------------------
# cloneState(source)
# C++: space->cloneState(src)->as<RealVectorStateSpace::StateType>()
# Allocates a new state and copies src into it.
# ---------------------------------------------------------------
print("\n-- cloneState(source) — allocate + copy in one call --")
s_clone = space.cloneState(s1)
print(f"  After cloneState(s1): s_clone = ({s_clone[0]}, {s_clone[1]}, {s_clone[2]})")
check(s_clone[0] == s1[0], "cloneState copies values correctly")
space.freeState(s_clone)

# ---------------------------------------------------------------
# copyToReals(source) — extract state as Python list
# C++: std::vector<double> reals; space->copyToReals(reals, state);
# ---------------------------------------------------------------
print("\n-- copyToReals(source) — state → Python list --")
reals = space.copyToReals(s1)
print(f"  copyToReals(s1) = {reals}")
check(reals == [3.0, 7.0, 1.5], "copyToReals returns correct list")

# ---------------------------------------------------------------
# copyFromReals(destination, reals) — set state from Python list
# C++: space->copyFromReals(state, reals);
# ---------------------------------------------------------------
print("\n-- copyFromReals(destination, reals) — Python list → state --")
s_from_reals = space.allocState()
space.copyFromReals(s_from_reals, [9.0, 8.0, 7.0])
print(f"  After copyFromReals([9,8,7]): ({s_from_reals[0]}, {s_from_reals[1]}, {s_from_reals[2]})")
check(s_from_reals[0] == 9.0, "copyFromReals sets state[0] correctly")
check(s_from_reals[1] == 8.0, "copyFromReals sets state[1] correctly")
check(s_from_reals[2] == 7.0, "copyFromReals sets state[2] correctly")
space.freeState(s_from_reals)

# ---------------------------------------------------------------
# printState(state) — state as string
# C++: space->printState(state, std::cout);
# ---------------------------------------------------------------
print("\n-- printState(state) — state as string --")
state_str = space.printState(s1)
print(f"  printState(s1) = '{state_str.strip()}'")
check(len(state_str) > 0, "printState returns non-empty string")

# ---------------------------------------------------------------
# getSerializationLength()
# C++: space->getSerializationLength()
# = dim * sizeof(double) = 3 * 8 = 24 bytes
# ---------------------------------------------------------------
print("\n-- getSerializationLength() --")
serial_len = space.getSerializationLength()
print(f"  getSerializationLength() = {serial_len} bytes")
check(serial_len == 3 * 8, "serialization length == dim * 8")


print("\n" + "=" * 60)
print("SECTION 5: Bounds Checking and Enforcement")
print("=" * 60)
print("""
satisfiesBounds(state)
    Returns True if all state values are within [low[i], high[i]].
    C++: space->satisfiesBounds(state)

enforceBounds(state)
    Clamps state values to [low[i], high[i]] in place.
    C++: space->enforceBounds(state)
""")

# space has bounds [0, 10]^3
s_in  = space.allocState()
s_out = space.allocState()

s_in[0], s_in[1], s_in[2]   = 5.0,  5.0,  5.0   # inside
s_out[0], s_out[1], s_out[2] = 15.0, 5.0, -1.0   # outside

print("-- satisfiesBounds() --")
print(f"  s_in  = (5,5,5):      satisfiesBounds = {space.satisfiesBounds(s_in)}")
print(f"  s_out = (15,5,-1):    satisfiesBounds = {space.satisfiesBounds(s_out)}")
check(space.satisfiesBounds(s_in)  is True,  "in-bounds state satisfies bounds")
check(space.satisfiesBounds(s_out) is False, "out-of-bounds state fails bounds check")

print("\n-- enforceBounds() --")
space.enforceBounds(s_out)
print(f"  After enforceBounds: ({s_out[0]}, {s_out[1]}, {s_out[2]})")
check(s_out[0] == 10.0, "x clamped to 10.0")
check(s_out[1] ==  5.0, "y unchanged at 5.0")
check(s_out[2] ==  0.0, "z clamped to 0.0")
check(space.satisfiesBounds(s_out), "state now satisfies bounds after enforceBounds")

space.freeState(s_in)
space.freeState(s_out)


print("\n" + "=" * 60)
print("SECTION 6: Distance")
print("=" * 60)
print("""
distance(state1, state2)
    Returns the Euclidean (L2) distance between two states.
    Formula: sqrt(sum((s2[i] - s1[i])^2))
    C++: space->distance(state1, state2)

This is the metric used by planners to measure how far apart
two configurations are. For a robot arm, it's the joint-space
distance; for a point robot it's physical distance.
""")

sa = space.allocState()
sb = space.allocState()

# Test 1: same state → distance = 0
sa[0], sa[1], sa[2] = 5.0, 5.0, 5.0
sb[0], sb[1], sb[2] = 5.0, 5.0, 5.0
d0 = space.distance(sa, sb)
print(f"-- distance(same, same) = {d0}")
check(d0 == 0.0, "distance between same states == 0")

# Test 2: known distance
sa[0], sa[1], sa[2] = 0.0, 0.0, 0.0
sb[0], sb[1], sb[2] = 3.0, 4.0, 0.0
d1 = space.distance(sa, sb)
print(f"-- distance((0,0,0), (3,4,0)) = {d1:.6f}")
check(abs(d1 - 5.0) < 1e-9, "distance == 5.0 (3-4-5 triangle)")

# Test 3: symmetry — distance(a,b) == distance(b,a)
sa[0], sa[1], sa[2] = 1.0, 2.0, 3.0
sb[0], sb[1], sb[2] = 4.0, 6.0, 3.0
d_ab = space.distance(sa, sb)
d_ba = space.distance(sb, sa)
print(f"-- distance(a,b) = {d_ab:.6f}, distance(b,a) = {d_ba:.6f}")
check(abs(d_ab - d_ba) < 1e-12, "distance is symmetric")

# Test 4: diagonal of [0,10]^3
sa[0], sa[1], sa[2] = 0.0, 0.0, 0.0
sb[0], sb[1], sb[2] = 10.0, 10.0, 10.0
d_diag = space.distance(sa, sb)
print(f"-- distance((0,0,0),(10,10,10)) = {d_diag:.6f} (expected {10*math.sqrt(3):.6f})")
check(abs(d_diag - 10.0 * math.sqrt(3)) < 1e-9, "diagonal distance correct")

space.freeState(sa)
space.freeState(sb)


print("\n" + "=" * 60)
print("SECTION 7: equalStates")
print("=" * 60)
print("""
equalStates(state1, state2)
    Returns True if both states have identical values.
    C++: space->equalStates(state1, state2)

Note: This is exact floating-point equality, not approximate.
""")

se1 = space.allocState()
se2 = space.allocState()
se3 = space.allocState()

se1[0], se1[1], se1[2] = 1.0, 2.0, 3.0
se2[0], se2[1], se2[2] = 1.0, 2.0, 3.0   # identical to se1
se3[0], se3[1], se3[2] = 1.0, 2.0, 4.0   # differs in last dim

print(f"  equalStates(se1, se2) = {space.equalStates(se1, se2)}")
print(f"  equalStates(se1, se3) = {space.equalStates(se1, se3)}")
check(space.equalStates(se1, se2) is True,  "identical states are equal")
check(space.equalStates(se1, se3) is False, "different states are not equal")

space.freeState(se1)
space.freeState(se2)
space.freeState(se3)


print("\n" + "=" * 60)
print("SECTION 8: Interpolation")
print("=" * 60)
print("""
interpolate(from, to, t, result)
    Computes the state at fraction t in [0,1] between 'from' and 'to'.
    t=0.0 → result == from
    t=1.0 → result == to
    t=0.5 → result == midpoint

    Formula: result[i] = from[i] + t * (to[i] - from[i])

    C++: space->interpolate(from, to, t, result);

    Used by planners to move along straight-line paths in state space.
    'result' must be a pre-allocated state.
""")

si_from   = space.allocState()
si_to     = space.allocState()
si_result = space.allocState()

si_from[0], si_from[1], si_from[2] = 0.0, 0.0, 0.0
si_to[0],   si_to[1],   si_to[2]   = 10.0, 10.0, 10.0

# t=0.0 → should equal 'from'
space.interpolate(si_from, si_to, 0.0, si_result)
print(f"-- t=0.0: ({si_result[0]}, {si_result[1]}, {si_result[2]})")
check(si_result[0] == 0.0, "t=0.0 gives from[0]")

# t=1.0 → should equal 'to'
space.interpolate(si_from, si_to, 1.0, si_result)
print(f"-- t=1.0: ({si_result[0]}, {si_result[1]}, {si_result[2]})")
check(si_result[0] == 10.0, "t=1.0 gives to[0]")

# t=0.5 → midpoint
space.interpolate(si_from, si_to, 0.5, si_result)
print(f"-- t=0.5: ({si_result[0]}, {si_result[1]}, {si_result[2]})")
check(si_result[0] == 5.0, "t=0.5 gives midpoint")

# t=0.25
space.interpolate(si_from, si_to, 0.25, si_result)
print(f"-- t=0.25: ({si_result[0]}, {si_result[1]}, {si_result[2]})")
check(si_result[0] == 2.5, "t=0.25 gives 25% of the way")

# Non-trivial values
si_from[0], si_from[1], si_from[2] = 2.0, 4.0, 6.0
si_to[0],   si_to[1],   si_to[2]   = 4.0, 8.0, 10.0
space.interpolate(si_from, si_to, 0.5, si_result)
print(f"-- from=(2,4,6) to=(4,8,10) t=0.5: ({si_result[0]}, {si_result[1]}, {si_result[2]})")
check(si_result[0] == 3.0 and si_result[1] == 6.0 and si_result[2] == 8.0,
      "interpolate midpoint correct for non-trivial values")

space.freeState(si_from)
space.freeState(si_to)
space.freeState(si_result)


print("\n" + "=" * 60)
print("SECTION 9: State Sampler")
print("=" * 60)
print("""
allocDefaultStateSampler() / allocStateSampler()
    Returns a RealVectorStateSampler for this space.
    C++: space->allocDefaultStateSampler()

Three sampling methods:

sampleUniform(state)
    Fills state with uniformly random values within the bounds.
    C++: sampler->sampleUniform(state);

sampleUniformNear(state, near, distance)
    Fills state with values from [near[i]-distance, near[i]+distance].
    Clamped to space bounds.
    C++: sampler->sampleUniformNear(state, near, distance);

sampleGaussian(state, mean, stdDev)
    Fills state with Gaussian-distributed values centred at 'mean'.
    Values outside bounds are clamped.
    C++: sampler->sampleGaussian(state, mean, stdDev);
""")

sampler = space.allocDefaultStateSampler()
print(f"  allocDefaultStateSampler() returned: {type(sampler).__name__}")
check(sampler is not None, "sampler is not None")

# sampleUniform
ss = space.allocState()
sampler.sampleUniform(ss)
print(f"\n-- sampleUniform: ({ss[0]:.3f}, {ss[1]:.3f}, {ss[2]:.3f})")
check(space.satisfiesBounds(ss), "uniform sample is within bounds")

# sampleUniformNear
near_state = space.allocState()
near_state[0], near_state[1], near_state[2] = 5.0, 5.0, 5.0
sampler.sampleUniformNear(ss, near_state, 1.0)
print(f"\n-- sampleUniformNear(near=(5,5,5), dist=1.0): ({ss[0]:.3f}, {ss[1]:.3f}, {ss[2]:.3f})")
check(space.satisfiesBounds(ss), "near-uniform sample is within bounds")

# sampleGaussian
mean_state = space.allocState()
mean_state[0], mean_state[1], mean_state[2] = 5.0, 5.0, 5.0
sampler.sampleGaussian(ss, mean_state, 1.0)
print(f"\n-- sampleGaussian(mean=(5,5,5), stdDev=1.0): ({ss[0]:.3f}, {ss[1]:.3f}, {ss[2]:.3f})")
check(space.satisfiesBounds(ss), "Gaussian sample is within bounds (clamped)")

space.freeState(ss)
space.freeState(near_state)
space.freeState(mean_state)


print("\n" + "=" * 60)
print("SECTION 10: setup() and registerProjections()")
print("=" * 60)
print("""
setup()
    Finalises the space. Computes segment lengths, registers
    default projections, builds value location maps.
    Called automatically by SpaceInformation — but you can
    call it manually if using the space standalone.
    C++: space->setup();

registerProjections()
    Registers the default projection for this space.
    A projection maps states to R^k for use in KPIECE planners.
    Called automatically by setup().
    C++: space->registerProjections();
""")

setup_space = pyompl.RealVectorStateSpace(2)
setup_space.setBounds(-5.0, 5.0)
setup_space.setup()
print("  setup() called successfully")
check(True, "setup() completes without error")

setup_space.registerProjections()
print("  registerProjections() called successfully")
check(True, "registerProjections() completes without error")


print("\n" + "=" * 60)
print("SECTION 11: Real-world usage pattern — 6-DOF robot arm")
print("=" * 60)
print("""
In a real motion planning scenario:
  Step 1: Create state space with correct DOF
  Step 2: Set joint limits as bounds
  Step 3: Pass to SpaceInformation (next example)
  Step 4: Define start/goal states
  Step 5: Run planner

This section covers steps 1, 2, and 4.
""")

dof = 6
arm = pyompl.RealVectorStateSpace(dof)
arm.setName("RobotArm6DOF")

joint_low  = [-3.14, -1.57, -3.14, -3.14, -1.57, -3.14]
joint_high = [ 3.14,  1.57,  3.14,  3.14,  1.57,  3.14]

arm_bounds = pyompl.RealVectorBounds(dof)
for i in range(dof):
    arm_bounds.setLow(i,  joint_low[i])
    arm_bounds.setHigh(i, joint_high[i])
arm.setBounds(arm_bounds)
arm.setup()

print(f"  Space: {arm.getName()}, DOF={arm.getDimension()}")
print(f"  Joint limits low:  {[round(v,2) for v in arm.getBounds().getLow()]}")
print(f"  Joint limits high: {[round(v,2) for v in arm.getBounds().getHigh()]}")

# Home config: all joints at 0
home = arm.allocState()
for i in range(dof):
    home[i] = 0.0

# Goal config
goal = arm.allocState()
vals = [1.5, 0.5, -1.0, 2.0, 0.3, -0.7]
for i in range(dof):
    goal[i] = vals[i]

dist = arm.distance(home, goal)
print(f"\n  Distance from home to goal: {dist:.4f}")
print(f"  Goal within joint limits:   {arm.satisfiesBounds(goal)}")
check(arm.satisfiesBounds(home), "home config satisfies bounds")
check(arm.satisfiesBounds(goal), "goal config satisfies bounds")

# Interpolate from home to goal at t=0.5
mid = arm.allocState()
arm.interpolate(home, goal, 0.5, mid)
mid_vals = [round(mid[i], 3) for i in range(dof)]
print(f"  Midpoint config (t=0.5): {mid_vals}")
check(arm.satisfiesBounds(mid), "midpoint config satisfies bounds")

arm.freeState(home)
arm.freeState(goal)
arm.freeState(mid)

print("\n" + "=" * 60)
print("ALL SECTIONS COMPLETE")
print("=" * 60)
print("""
What we covered:
  RealVectorBounds:
    constructor, setLow, setHigh, resize, getLow, getHigh,
    getDimension, getVolume, getDifference, check

  RealVectorStateSpace:
    constructor, setName, getName, setBounds (two forms),
    getBounds, getDimension, addDimension (two forms),
    getDimensionName, getDimensionIndex, setDimensionName,
    getMaximumExtent, getMeasure, printSettings, repr,
    allocState, freeState, copyState, cloneState,
    copyToReals, copyFromReals, printState,
    getSerializationLength, satisfiesBounds, enforceBounds,
    distance, equalStates, interpolate,
    allocDefaultStateSampler, allocStateSampler,
    sampleUniform, sampleUniformNear, sampleGaussian,
    setup, registerProjections

Next: examples/02_se2_state_space.py
""")