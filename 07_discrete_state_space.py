"""
===============================================================
pyompl Example 07 — DiscreteStateSpace
===============================================================

WHAT IS DiscreteStateSpace?
----------------------------
DiscreteStateSpace represents a FINITE SET of integer states.
A state is just ONE integer value in the range [lowerBound, upperBound].

Unlike all other state spaces we have seen (which are continuous),
this one is DISCRETE — there is no "in between" two states.

REAL-WORLD EXAMPLES:
  - Robot gear:        DiscreteStateSpace(1, 5)  → gears 1,2,3,4,5
  - Elevator floor:    DiscreteStateSpace(0, 20) → floors 0-20
  - Traffic light:     DiscreteStateSpace(0, 2)  → RED=0, YELLOW=1, GREEN=2
  - Gripper fingers:   DiscreteStateSpace(0, 5)  → 0-5 fingers gripping
  - Robot mode:        DiscreteStateSpace(0, 3)  → IDLE,MOVE,GRASP,PLACE

KEY PROPERTIES (different from continuous spaces):
  - isDiscrete() returns True
  - States are integers, distance is |v1 - v2|
  - NO wrapping (lowerBound and upperBound are hard walls)
  - Interpolation rounds to nearest integer
  - getMeasure() returns state COUNT (not a geometric volume)

COMBINING WITH CONTINUOUS SPACES:
  Almost always used with CompoundStateSpace:
    (x, y, gear) = RealVectorStateSpace(2) + DiscreteStateSpace(1, 5)

C++ HEADER: ompl/base/spaces/DiscreteStateSpace.h
C++ CLASS:  ompl::base::DiscreteStateSpace
INHERITS:   StateSpace

RUN THIS FILE:
    python examples/07_discrete_state_space.py
===============================================================
"""

import math
import pyompl

def check(condition, msg):
    print(f"  {'PASS' if condition else 'FAIL'} — {msg}")

def approx(a, b, tol=1e-9):
    return abs(a - b) < tol


print("=" * 60)
print("SECTION 1: Construction and Basic Properties")
print("=" * 60)
print("""
Constructor: DiscreteStateSpace(lowerBound, upperBound)
Both bounds are INCLUSIVE.
Total states = upperBound - lowerBound + 1

C++: DiscreteStateSpace space(lowerBound, upperBound);
""")

# Basic space: 5 gears (1 through 5)
gear_space = pyompl.DiscreteStateSpace(1, 5)
gear_space.setName("GearSelector")
gear_space.setup()

print(f"  DiscreteStateSpace(1, 5) — gears 1-5")
print(f"  getName()           = {gear_space.getName()}")
print(f"  getDimension()      = {gear_space.getDimension()}")
print(f"  getLowerBound()     = {gear_space.getLowerBound()}")
print(f"  getUpperBound()     = {gear_space.getUpperBound()}")
print(f"  getStateCount()     = {gear_space.getStateCount()}")
print(f"  getMaximumExtent()  = {gear_space.getMaximumExtent()}")
print(f"  getMeasure()        = {gear_space.getMeasure()}")
print(f"  isDiscrete()        = {gear_space.isDiscrete()}")
print(f"  repr: {repr(gear_space)}")

check(gear_space.getDimension() == 1, "getDimension() == 1")
check(gear_space.getLowerBound() == 1, "lower bound == 1")
check(gear_space.getUpperBound() == 5, "upper bound == 5")
check(gear_space.getStateCount() == 5, "state count == 5")
check(approx(gear_space.getMaximumExtent(), 4.0), "max extent == 4 (5-1)")
check(approx(gear_space.getMeasure(), 5.0), "measure == 5 (count)")
check(gear_space.isDiscrete() is True, "isDiscrete() == True")

# Space with negative bounds
print("\n-- DiscreteStateSpace(-3, 3) — 7 states --")
signed_space = pyompl.DiscreteStateSpace(-3, 3)
signed_space.setup()
print(f"  getStateCount() = {signed_space.getStateCount()}  (states: -3,-2,-1,0,1,2,3)")
check(signed_space.getStateCount() == 7, "state count == 7")
check(signed_space.getLowerBound() == -3, "lower bound == -3")

# Single-state space
print("\n-- DiscreteStateSpace(42, 42) — 1 state --")
single = pyompl.DiscreteStateSpace(42, 42)
# single.setup()
check(single.getStateCount() == 1, "single state space has count 1")
check(approx(single.getMaximumExtent(), 0.0), "max extent == 0 for single state")


print("\n" + "=" * 60)
print("SECTION 2: DiscreteState — The State Object")
print("=" * 60)
print("""
A DiscreteState has ONE field: .value (an integer).

C++ field: state->value
Python:    state.value

Do NOT construct directly — use space.allocState().
""")

s1 = gear_space.allocState()
s2 = gear_space.allocState()

print("-- allocState() --")
check(s1 is not None, "allocState returns non-None")

print("\n-- state.value — read and write --")
s1.value = 3
print(f"  s1.value = 3: {s1.value}")
check(s1.value == 3, ".value == 3")

s1.value = 1
print(f"  s1.value = 1: {s1.value}")
check(s1.value == 1, ".value == 1")

s2.value = 5
print(f"  s2.value = 5: {s2.value}")

print(f"\n  repr(s1) = {repr(s1)}")
print(f"  repr(s2) = {repr(s2)}")


print("\n" + "=" * 60)
print("SECTION 3: setBounds — Changing Bounds After Construction")
print("=" * 60)
print("""
setBounds(lowerBound, upperBound)
    Changes the bounds after construction.
    C++: space->setBounds(lowerBound, upperBound);

    Call setup() after changing bounds.
""")

# Create space and change bounds
dynamic = pyompl.DiscreteStateSpace(0, 3)
dynamic.setup()
print(f"  Initial: [{dynamic.getLowerBound()}, {dynamic.getUpperBound()}], "
      f"count={dynamic.getStateCount()}")

dynamic.setBounds(0, 10)
dynamic.setup()
print(f"  After setBounds(0,10): [{dynamic.getLowerBound()}, {dynamic.getUpperBound()}], "
      f"count={dynamic.getStateCount()}")
check(dynamic.getLowerBound() == 0, "lower bound updated to 0")
check(dynamic.getUpperBound() == 10, "upper bound updated to 10")
check(dynamic.getStateCount() == 11, "state count updated to 11")

dynamic.setBounds(-5, 5)
dynamic.setup()
print(f"  After setBounds(-5,5): [{dynamic.getLowerBound()}, {dynamic.getUpperBound()}], "
      f"count={dynamic.getStateCount()}")
check(dynamic.getStateCount() == 11, "state count still 11")


print("\n" + "=" * 60)
print("SECTION 4: satisfiesBounds and enforceBounds")
print("=" * 60)
print("""
satisfiesBounds(state)
    Returns True if lowerBound <= state.value <= upperBound.
    C++: space->satisfiesBounds(state)

enforceBounds(state)
    Clamps state.value to [lowerBound, upperBound].
    C++: space->enforceBounds(state)

NO WRAPPING: this is NOT like SO2. Values are clamped, not wrapped.
""")

s = gear_space.allocState()   # bounds [1, 5]

print("-- satisfiesBounds() for gear space [1, 5] --")
for val, expected in [(1, True), (3, True), (5, True), (0, False), (6, False), (-1, False)]:
    s.value = val
    result = gear_space.satisfiesBounds(s)
    print(f"  value={val:2d}: satisfiesBounds = {result}")
    check(result == expected, f"value={val} → {expected}")

print("\n-- enforceBounds() for gear space [1, 5] --")
test_cases = [(0, 1), (-5, 1), (6, 5), (10, 5), (3, 3), (1, 1), (5, 5)]
for before, after in test_cases:
    s.value = before
    gear_space.enforceBounds(s)
    print(f"  {before:3d} → {s.value}")
    check(s.value == after, f"enforceBounds({before}) == {after}")

gear_space.freeState(s)


print("\n" + "=" * 60)
print("SECTION 5: copyState, cloneState, equalStates")
print("=" * 60)

s1 = gear_space.allocState(); s1.value = 3
s2 = gear_space.allocState(); s2.value = 1

print("-- copyState(dst, src) --")
gear_space.copyState(s2, s1)
print(f"  src.value=3, after copyState: dst.value={s2.value}")
check(s2.value == 3, "copyState copies integer correctly")

print("\n-- cloneState(src) --")
s1.value = 4
clone = gear_space.cloneState(s1)
print(f"  src.value=4, clone.value={clone.value}")
check(clone.value == 4, "cloneState correct")
gear_space.freeState(clone)

print("\n-- equalStates() --")
e1 = gear_space.allocState(); e1.value = 2
e2 = gear_space.allocState(); e2.value = 2
e3 = gear_space.allocState(); e3.value = 4

print(f"  equalStates(2, 2) = {gear_space.equalStates(e1, e2)}")
print(f"  equalStates(2, 4) = {gear_space.equalStates(e1, e3)}")
check(gear_space.equalStates(e1, e2) is True,  "same value → equal")
check(gear_space.equalStates(e1, e3) is False, "different value → not equal")

gear_space.freeState(s1); gear_space.freeState(s2)
gear_space.freeState(e1); gear_space.freeState(e2); gear_space.freeState(e3)


print("\n" + "=" * 60)
print("SECTION 6: distance — Integer Absolute Difference")
print("=" * 60)
print("""
distance(state1, state2)
    Returns |state1.value - state2.value| as a double.
    C++: space->distance(state1, state2)

    Distance between adjacent states = 1.
    Distance is always a non-negative integer (as a double).
    NO circular topology — 0 and 5 are NOT close to each other.
""")

a = gear_space.allocState()
b = gear_space.allocState()

test_cases = [
    (1, 1, 0.0),
    (1, 5, 4.0),
    (5, 1, 4.0),  # symmetric
    (2, 4, 2.0),
    (3, 3, 0.0),
    (1, 2, 1.0),
]

for v1, v2, expected in test_cases:
    a.value, b.value = v1, v2
    d = gear_space.distance(a, b)
    print(f"  d({v1}, {v2}) = {d:.1f}  (expected {expected})")
    check(approx(d, expected), f"|{v1}-{v2}| == {expected}")

# Max distance = upperBound - lowerBound
a.value, b.value = 1, 5
d_max = gear_space.distance(a, b)
print(f"\n  Max distance d(1, 5) = {d_max:.1f}  (= getMaximumExtent = {gear_space.getMaximumExtent():.1f})")
check(approx(d_max, gear_space.getMaximumExtent()), "max distance == max extent")

gear_space.freeState(a)
gear_space.freeState(b)


print("\n" + "=" * 60)
print("SECTION 7: interpolate — Rounded Linear Interpolation")
print("=" * 60)
print("""
interpolate(from, to, t, result)
    result.value = round(from.value + t * (to.value - from.value))

    C++: space->interpolate(from, to, t, result);

    IMPORTANT: Because states are integers, result is ROUNDED.
    This means you get valid discrete states at every t value.

    Example: from=0, to=5
      t=0.0 → round(0.0) = 0
      t=0.2 → round(1.0) = 1
      t=0.4 → round(2.0) = 2
      t=0.5 → round(2.5) = 2 or 3 (rounding to nearest even / implementation)
      t=0.6 → round(3.0) = 3
      t=0.8 → round(4.0) = 4
      t=1.0 → round(5.0) = 5
""")

space_0_10 = pyompl.DiscreteStateSpace(0, 10)
space_0_10.setup()

f = space_0_10.allocState(); f.value = 0
t_s = space_0_10.allocState(); t_s.value = 10
r = space_0_10.allocState()

print("-- from=0, to=10, quarter steps --")
for step_t in [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]:
    space_0_10.interpolate(f, t_s, step_t, r)
    expected_float = step_t * 10
    print(f"  t={step_t:.1f}: result.value={r.value}  (raw={expected_float:.1f})")
    check(space_0_10.satisfiesBounds(r), f"interpolated state at t={step_t} in bounds")

# Edge cases
print("\n-- t=0 → from, t=1 → to --")
space_0_10.interpolate(f, t_s, 0.0, r)
check(r.value == 0, "t=0 gives from")
space_0_10.interpolate(f, t_s, 1.0, r)
check(r.value == 10, "t=1 gives to")

space_0_10.freeState(f); space_0_10.freeState(t_s); space_0_10.freeState(r)


print("\n" + "=" * 60)
print("SECTION 8: getSerializationLength")
print("=" * 60)

slen = gear_space.getSerializationLength()
print(f"  getSerializationLength() = {slen} bytes  (sizeof(int) = 4)")
check(slen == 4, "serialization length == 4 bytes")


print("\n" + "=" * 60)
print("SECTION 9: printState, printSettings")
print("=" * 60)

ps = gear_space.allocState(); ps.value = 3
print(f"  printState(3): '{gear_space.printState(ps).strip()}'")
check(len(gear_space.printState(ps)) > 0, "printState non-empty")
gear_space.freeState(ps)

cfg = gear_space.printSettings()
print(f"\n  printSettings():\n{cfg}")
check(len(cfg) > 0, "printSettings non-empty")


print("\n" + "=" * 60)
print("SECTION 10: Sampling")
print("=" * 60)
print("""
sampleUniform(state)
    Samples uniformly from [lowerBound, upperBound].
    Every integer equally likely.

sampleUniformNear(state, near, distance)
    Samples from [near.value - floor(distance), near.value + floor(distance)]
    Clamped to bounds.

sampleGaussian(state, mean, stdDev)
    Samples from discrete Gaussian, rounded and clamped.
""")

ss = gear_space.allocState()
near_s = gear_space.allocState(); near_s.value = 3

print("-- sampleUniform() × 10 for [1,5] --")
counts = {1: 0, 2: 0, 3: 0, 4: 0, 5: 0}
for i in range(50):
    gear_space.sampleUniform(ss)
    check(gear_space.satisfiesBounds(ss), f"sample in bounds")
    if ss.value in counts:
        counts[ss.value] += 1
print(f"  Counts over 50 samples: {counts}")
check(all(v > 0 for v in counts.values()), "all 5 gears were sampled")

print("\n-- sampleUniformNear(near=3, distance=1.5) × 5 --")
print("  Valid range: [3-1=2, 3+1=4] (floor(1.5)=1)")
for i in range(5):
    gear_space.sampleUniformNear(ss, near_s, 1.5)
    print(f"  sample {i}: value={ss.value}")
    check(gear_space.satisfiesBounds(ss), f"near sample {i} in bounds")
    check(2 <= ss.value <= 4, f"near sample {i} within distance of 3")

print("\n-- sampleGaussian(mean=3, stdDev=1.0) × 5 --")
for i in range(5):
    gear_space.sampleGaussian(ss, near_s, 1.0)
    print(f"  sample {i}: value={ss.value}")
    check(gear_space.satisfiesBounds(ss), f"Gaussian sample {i} in bounds")

gear_space.freeState(ss)
gear_space.freeState(near_s)


print("\n" + "=" * 60)
print("SECTION 11: Real-world — Robot Mode Switching")
print("=" * 60)
print("""
Use case: a robot with 4 operational modes:
  0 = IDLE
  1 = MOVING
  2 = GRASPING
  3 = PLACING

We plan mode sequences and check valid transitions.
""")

MODE_IDLE     = 0
MODE_MOVING   = 1
MODE_GRASPING = 2
MODE_PLACING  = 3
MODE_NAMES    = {0: "IDLE", 1: "MOVING", 2: "GRASPING", 3: "PLACING"}

mode_space = pyompl.DiscreteStateSpace(0, 3)
mode_space.setName("RobotMode")
mode_space.setup()

print(f"  Mode space: [{mode_space.getLowerBound()}, {mode_space.getUpperBound()}]")
print(f"  Modes: {MODE_NAMES}")

start = mode_space.allocState(); start.value = MODE_IDLE
goal  = mode_space.allocState(); goal.value  = MODE_PLACING

print(f"\n  Start: {MODE_NAMES[start.value]}")
print(f"  Goal:  {MODE_NAMES[goal.value]}")
print(f"  Distance: {mode_space.distance(start, goal):.1f} mode steps")

wp = mode_space.allocState()
print(f"\n  Mode sequence (interpolated):")
for step_t in [0.0, 0.33, 0.67, 1.0]:
    mode_space.interpolate(start, goal, step_t, wp)
    print(f"  t={step_t:.2f}: mode={wp.value} ({MODE_NAMES[wp.value]})")

# Check each mode is valid
for mode_val, name in MODE_NAMES.items():
    wp.value = mode_val
    print(f"  {name} ({mode_val}): satisfiesBounds = {mode_space.satisfiesBounds(wp)}")
    check(mode_space.satisfiesBounds(wp), f"{name} is a valid mode")

# Invalid mode
wp.value = 99
print(f"  Mode 99: satisfiesBounds = {mode_space.satisfiesBounds(wp)}")
check(mode_space.satisfiesBounds(wp) is False, "mode 99 is invalid")
mode_space.enforceBounds(wp)
print(f"  After enforceBounds: {wp.value} ({MODE_NAMES[wp.value]})")
check(wp.value == 3, "clamped to PLACING (max mode)")

mode_space.freeState(start)
mode_space.freeState(goal)
mode_space.freeState(wp)


print("\n" + "=" * 60)
print("ALL SECTIONS COMPLETE")
print("=" * 60)
print("""
What we covered:

  DiscreteState:
    .value (read/write), repr

  DiscreteStateSampler:
    sampleUniform, sampleUniformNear, sampleGaussian

  DiscreteStateSpace:
    constructor (lowerBound, upperBound)
    getLowerBound, getUpperBound, getStateCount
    setBounds (change bounds after construction)
    isDiscrete (always True)
    getDimension (always 1)
    getMaximumExtent (= upperBound - lowerBound)
    getMeasure (= state count)
    satisfiesBounds, enforceBounds (clamping, NOT wrapping)
    copyState, cloneState, equalStates
    getSerializationLength (= 4 bytes)
    distance (= |v1 - v2|, integer steps)
    interpolate (rounded linear)
    allocState, freeState
    sampleUniform, sampleUniformNear, sampleGaussian
    printState, printSettings, setup

Key concepts:
  - Integer states only — no continuous topology
  - NO wrapping — bounds are hard walls
  - distance = |v1 - v2| (counting steps)
  - interpolation rounds to nearest integer
  - isDiscrete() = True (distinguishes from continuous spaces)
  - Almost always combined with continuous spaces via CompoundStateSpace

Next: examples/08_space_time_state_space.py
""")