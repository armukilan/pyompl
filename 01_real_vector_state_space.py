"""
===============================================================
pyompl Example 01 — RealVectorStateSpace
===============================================================

WHAT IS THIS?
-------------
In OMPL, a "State Space" defines the space of all possible
configurations a robot can be in. The simplest one is
RealVectorStateSpace — a flat n-dimensional space like R^2 or R^3.

Think of it as: if your robot has 3 joints, each joint has a
position. The state space is the space of all joint positions.
A "state" is one specific configuration (one point in that space).

C++ EQUIVALENT (what pyompl wraps):
    #include <ompl/base/spaces/RealVectorStateSpace.h>
    using namespace ompl::base;

RUN THIS FILE:
    python examples/01_real_vector_state_space.py
===============================================================
"""

import math
import pyompl

print("=" * 60)
print("PART 1: RealVectorBounds")
print("=" * 60)

"""
WHAT IS RealVectorBounds?
--------------------------
Before you can use a state space, you must define its boundaries —
i.e., the minimum and maximum value each dimension can take.

For a 2D robot in a room from -5 to 5 meters on both axes:
    low  = [-5, -5]
    high = [ 5,  5]

C++ equivalent:
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(-5.0);
    bounds.setHigh(5.0);
"""

# Create bounds for a 2-dimensional space
bounds_2d = pyompl.RealVectorBounds(2)

# Set all lower bounds to -5.0 (same value for all dims)
# C++: bounds.setLow(-5.0);
bounds_2d.setLow(-5.0)

# Set all upper bounds to 5.0 (same value for all dims)
# C++: bounds.setHigh(5.0);
bounds_2d.setHigh(5.0)

print(f"Lower bounds: {bounds_2d.getLow()}")   # [-5.0, -5.0]
print(f"Upper bounds: {bounds_2d.getHigh()}")  # [ 5.0,  5.0]
print(f"Dimensions:   {bounds_2d.getDimension()}")  # 2

# You can also set bounds per dimension individually
# Useful when each joint has a different range
# C++: bounds.setLow(0, -3.14);  bounds.setLow(1, 0.0);
bounds_joint = pyompl.RealVectorBounds(2)
bounds_joint.setLow(0, -3.14)   # joint 0: -pi to pi
bounds_joint.setLow(1,  0.0)    # joint 1: 0 to 2
bounds_joint.setHigh(0, 3.14)
bounds_joint.setHigh(1, 2.0)

print(f"\nPer-dimension lower bounds: {bounds_joint.getLow()}")   # [-3.14, 0.0]
print(f"Per-dimension upper bounds: {bounds_joint.getHigh()}")   # [ 3.14, 2.0]

# check() validates the bounds — raises exception if low >= high
# C++: bounds.check();
try:
    bounds_2d.check()
    print("\nbounds_2d.check() passed — bounds are valid")
except Exception as e:
    print(f"bounds_2d.check() failed: {e}")

# Example of invalid bounds
bad_bounds = pyompl.RealVectorBounds(2)
bad_bounds.setLow(5.0)   # low > high — this is wrong
bad_bounds.setHigh(-5.0)
try:
    bad_bounds.check()
except Exception as e:
    print(f"bad_bounds.check() raised: {type(e).__name__} (expected)")


print("\n" + "=" * 60)
print("PART 2: Creating a RealVectorStateSpace")
print("=" * 60)

"""
WHAT IS RealVectorStateSpace?
------------------------------
This is the state space itself. It represents R^n — an n-dimensional
continuous space. You create it, set its bounds, and then use it
to allocate states, compute distances, interpolate, etc.

Think of it as the "coordinate system" your robot lives in.

C++ equivalent:
    auto space = std::make_shared<ompl::base::RealVectorStateSpace>(3);
    space->setBounds(bounds);
"""

# Create a 3D state space (e.g., a drone at position x, y, z)
# C++: auto space = std::make_shared<RealVectorStateSpace>(3);
space = pyompl.RealVectorStateSpace(3)

# Set bounds: drone can fly in a 10x10x10 meter cube
bounds_3d = pyompl.RealVectorBounds(3)
bounds_3d.setLow(0.0)
bounds_3d.setHigh(10.0)

# C++: space->setBounds(bounds);
space.setBounds(bounds_3d)

# Get the number of dimensions
# C++: space->getDimension()
print(f"Dimensions: {space.getDimension()}")  # 3

# Give the space a name (optional, useful for debugging)
# C++: space->setName("DroneSpace");
space.setName("DroneSpace")
print(f"Space name: {space.getName()}")  # DroneSpace

# Get the maximum possible distance between any two states
# For a cube [0,10]^3, it's the diagonal: 10 * sqrt(3) ≈ 17.32
# C++: space->getMaximumExtent()
max_extent = space.getMaximumExtent()
print(f"Max extent: {max_extent:.4f}")           # ≈ 17.3205
print(f"Expected:   {10.0 * math.sqrt(3):.4f}") # 17.3205

# Get bounds back from the space
# C++: space->getBounds()
retrieved_bounds = space.getBounds()
print(f"Retrieved low:  {retrieved_bounds.getLow()}")   # [0.0, 0.0, 0.0]
print(f"Retrieved high: {retrieved_bounds.getHigh()}")  # [10.0, 10.0, 10.0]

# repr
print(f"repr: {repr(space)}")  # <RealVectorStateSpace dim=3>


print("\n" + "=" * 60)
print("PART 3: Allocating and Using States")
print("=" * 60)

"""
WHAT IS A STATE?
-----------------
A state is one specific configuration — one point in the state space.
In RealVectorStateSpace, a state is just an array of doubles.

You MUST allocate states through the space (not with 'new' or similar).
The space manages memory. Always free states when done.

C++ equivalent:
    ompl::base::State *state = space->allocState();
    state->as<RealVectorStateSpace::StateType>()->values[0] = 5.0;
    space->freeState(state);
"""

# Allocate a new state
# C++: auto *s = space->allocState()->as<RealVectorStateSpace::StateType>();
s1 = space.allocState()

# Set values using index (like array access)
# C++: s->values[0] = 5.0; s->values[1] = 3.0; s->values[2] = 7.0;
s1[0] = 5.0   # x = 5
s1[1] = 3.0   # y = 3
s1[2] = 7.0   # z = 7

print(f"s1[0] = {s1[0]}")  # 5.0
print(f"s1[1] = {s1[1]}")  # 3.0
print(f"s1[2] = {s1[2]}")  # 7.0

# Allocate a second state
s2 = space.allocState()
s2[0] = 1.0
s2[1] = 1.0
s2[2] = 1.0


print("\n" + "=" * 60)
print("PART 4: Distance Between States")
print("=" * 60)

"""
WHAT IS distance()?
--------------------
Computes the Euclidean distance between two states.
For RealVectorStateSpace this is the standard L2 norm.

Formula: sqrt( (x2-x1)^2 + (y2-y1)^2 + (z2-z1)^2 )

C++ equivalent:
    double d = space->distance(s1, s2);
"""

# s1 = (5, 3, 7), s2 = (1, 1, 1)
# distance = sqrt((5-1)^2 + (3-1)^2 + (7-1)^2) = sqrt(16+4+36) = sqrt(56)
dist = space.distance(s1, s2)
expected = math.sqrt((5-1)**2 + (3-1)**2 + (7-1)**2)
print(f"distance(s1, s2) = {dist:.6f}")      # 7.483314
print(f"expected         = {expected:.6f}")  # 7.483314


print("\n" + "=" * 60)
print("PART 5: Interpolation Between States")
print("=" * 60)

"""
WHAT IS interpolate()?
-----------------------
Given two states 'from' and 'to', compute an intermediate state
at fraction t (where t=0 gives 'from', t=1 gives 'to', t=0.5 is midpoint).

This is used by planners to move along a straight line between states.

C++ equivalent:
    ompl::base::State *result = space->allocState();
    space->interpolate(from, to, t, result);
"""

result = space.allocState()

# t=0.0 → should equal s1
space.interpolate(s1, s2, 0.0, result)
print(f"t=0.0: ({result[0]:.1f}, {result[1]:.1f}, {result[2]:.1f})")  # (5, 3, 7)

# t=1.0 → should equal s2
space.interpolate(s1, s2, 1.0, result)
print(f"t=1.0: ({result[0]:.1f}, {result[1]:.1f}, {result[2]:.1f})")  # (1, 1, 1)

# t=0.5 → midpoint
space.interpolate(s1, s2, 0.5, result)
print(f"t=0.5: ({result[0]:.1f}, {result[1]:.1f}, {result[2]:.1f})")  # (3, 2, 4)

# t=0.25 → 25% of the way from s1 to s2
space.interpolate(s1, s2, 0.25, result)
print(f"t=0.25: ({result[0]:.2f}, {result[1]:.2f}, {result[2]:.2f})")  # (4, 2.5, 5.5)


print("\n" + "=" * 60)
print("PART 6: Checking and Enforcing Bounds")
print("=" * 60)

"""
WHAT IS satisfiesBounds()?
---------------------------
Returns True if a state is within the defined bounds, False otherwise.
Planners use this to check if a sampled state is valid.

C++ equivalent:
    bool ok = space->satisfiesBounds(state);

WHAT IS enforceBounds()?
-------------------------
Clamps the state values to lie within the bounds.
If s[0] = 15.0 and high[0] = 10.0, it becomes 10.0.

C++ equivalent:
    space->enforceBounds(state);
"""

# Create a state inside bounds
s_inside = space.allocState()
s_inside[0] = 5.0
s_inside[1] = 5.0
s_inside[2] = 5.0
print(f"Inside bounds?  {space.satisfiesBounds(s_inside)}")  # True

# Create a state outside bounds
s_outside = space.allocState()
s_outside[0] = 15.0   # > 10 (out of bounds)
s_outside[1] =  5.0
s_outside[2] = -1.0   # < 0  (out of bounds)
print(f"Outside bounds? {space.satisfiesBounds(s_outside)}")  # False

# Enforce bounds — clamps to [0, 10]
space.enforceBounds(s_outside)
print(f"After enforceBounds: ({s_outside[0]:.1f}, {s_outside[1]:.1f}, {s_outside[2]:.1f})")
# (10.0, 5.0, 0.0)
print(f"Now in bounds?  {space.satisfiesBounds(s_outside)}")  # True


print("\n" + "=" * 60)
print("PART 7: Freeing States (Memory Management)")
print("=" * 60)

"""
WHAT IS freeState()?
---------------------
States allocated with allocState() must be freed when you're done.
This is equivalent to C++ delete, but done through the space.

In C++ OMPL, this is critical. In pyompl, the Python GC helps,
but explicitly freeing is good practice and matches C++ behavior.

C++ equivalent:
    space->freeState(s1);
"""

# Free all the states we allocated
space.freeState(s1)
space.freeState(s2)
space.freeState(result)
space.freeState(s_inside)
space.freeState(s_outside)
print("All states freed successfully.")


print("\n" + "=" * 60)
print("PART 8: Real-world usage pattern")
print("=" * 60)

"""
HOW YOU'D ACTUALLY USE THIS:
------------------------------
In a real motion planning scenario:
1. Create the space with correct dimensions for your robot
2. Set bounds to your robot's joint limits or workspace limits
3. Pass the space to SpaceInformation (next example)
4. SpaceInformation + ProblemDefinition → Planner → solve()

This example just shows Step 1 and 2.
"""

# Example: a 6-DOF robot arm
# Each joint has different angle limits (in radians)
dof = 6
arm_space = pyompl.RealVectorStateSpace(dof)
arm_space.setName("RobotArm6DOF")

joint_limits_low  = [-3.14, -1.57, -3.14, -3.14, -1.57, -3.14]
joint_limits_high = [ 3.14,  1.57,  3.14,  3.14,  1.57,  3.14]

arm_bounds = pyompl.RealVectorBounds(dof)
for i in range(dof):
    arm_bounds.setLow(i,  joint_limits_low[i])
    arm_bounds.setHigh(i, joint_limits_high[i])

arm_space.setBounds(arm_bounds)

print(f"Robot arm space: {arm_space.getName()}, DOF={arm_space.getDimension()}")
print(f"Joint low  limits: {[round(v, 2) for v in arm_bounds.getLow()]}")
print(f"Joint high limits: {[round(v, 2) for v in arm_bounds.getHigh()]}")

# Allocate a "home" configuration (all joints at 0)
home = arm_space.allocState()
for i in range(dof):
    home[i] = 0.0

# Allocate a "goal" configuration
goal_config = arm_space.allocState()
goal_config[0] =  1.5
goal_config[1] =  0.5
goal_config[2] = -1.0
goal_config[3] =  2.0
goal_config[4] =  0.3
goal_config[5] = -0.7

dist_home_to_goal = arm_space.distance(home, goal_config)
print(f"\nDistance from home to goal config: {dist_home_to_goal:.4f}")

# Check if goal is within joint limits
print(f"Goal config in bounds: {arm_space.satisfiesBounds(goal_config)}")

arm_space.freeState(home)
arm_space.freeState(goal_config)

print("\n" + "=" * 60)
print("Done! All examples completed successfully.")
print("Next: examples/02_se2_state_space.py")
print("=" * 60)