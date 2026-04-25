"""
===============================================================
pyompl Example 09 — EmptyStateSpace, HybridTimeStateSpace,
                    WrapperStateSpace
===============================================================

THREE STATE SPACES IN ONE FILE:

1. EmptyStateSpace
   A RealVectorStateSpace with dim=0. A structural placeholder.
   Used in multi-level planners. No actual degrees of freedom.

2. HybridTimeStateSpace
   Like TimeStateSpace but adds a JUMP counter.
   For hybrid systems with both continuous dynamics and
   discrete events (bouncing ball, gear changes, contacts).

3. WrapperStateSpace
   A decorator that wraps any other state space.
   Forwards all operations to the inner space transparently.
   Base class for constraint spaces in OMPL.

C++ HEADERS:
  ompl/base/spaces/EmptyStateSpace.h
  ompl/base/spaces/HybridTimeStateSpace.h
  ompl/base/spaces/WrapperStateSpace.h

RUN THIS FILE:
    python examples/09_extra_state_spaces.py
===============================================================
"""

import math
import pyompl

def check(condition, msg):
    print(f"  {'PASS' if condition else 'FAIL'} — {msg}")

def approx(a, b, tol=1e-9):
    return abs(a - b) < tol


# ==============================================================
# PART 1: EmptyStateSpace
# ==============================================================
print("=" * 60)
print("PART 1: EmptyStateSpace")
print("=" * 60)
print("""
WHAT IS EmptyStateSpace?
-------------------------
EmptyStateSpace is a RealVectorStateSpace(0) — zero dimensions.

WHY DOES IT EXIST?
  In multi-level motion planning (e.g. MLRRT, hierarchical planners),
  a robot's configuration space is decomposed into levels. Some levels
  may project onto an empty subspace. EmptyStateSpace is a structural
  placeholder that allows the framework to treat all levels uniformly.

  Example: A 7-DOF arm decomposed into:
    Level 0: first 4 joints → RealVectorStateSpace(4)
    Level 1: remaining 3 joints → RealVectorStateSpace(3)
    Level 2: projection onto base → EmptyStateSpace (no DOF here)

C++ class:   ompl::base::EmptyStateSpace
C++ header:  ompl/base/spaces/EmptyStateSpace.h
Inherits:    RealVectorStateSpace(0) → StateSpace

KEY PROPERTIES:
  getDimension()     = 0   ← no degrees of freedom
  getMaximumExtent() = 0   ← no distance possible
  getMeasure()       = 0   ← no volume
  setup()            = no-op
  getName()          = 'EmptySpace'
""")

# ---------------------------------------------------------------
# Construction — no arguments needed
# C++: EmptyStateSpace space;
# ---------------------------------------------------------------
print("-- Constructor --")
empty = pyompl.EmptyStateSpace()
print(f"  EmptyStateSpace() created")
print(f"  getName()          = {empty.getName()}")
print(f"  getDimension()     = {empty.getDimension()}")
print(f"  getMaximumExtent() = {empty.getMaximumExtent()}")
print(f"  getMeasure()       = {empty.getMeasure()}")
print(f"  repr: {repr(empty)}")

check(empty.getName() == "EmptySpace",    "name == 'EmptySpace'")
check(empty.getDimension() == 0,          "dimension == 0")
check(empty.getMaximumExtent() == 0.0,    "max extent == 0")
check(empty.getMeasure() == 0.0,          "measure == 0")

# setup() is a no-op — does not raise
print("\n-- setup() — no-op --")
empty.setup()
print("  setup() completed (no-op)")
check(True, "setup() does not raise")

print("\n-- Use case: placeholder in compound space --")
# EmptyStateSpace is often combined with real spaces in compound planning
# Here we just verify it integrates with the type system
combined = pyompl.CompoundStateSpace()
real_part = pyompl.RealVectorStateSpace(3)
real_bounds = pyompl.RealVectorBounds(3)
real_bounds.setLow(-1.0); real_bounds.setHigh(1.0)
real_part.setBounds(real_bounds)
combined.addSubspace(real_part, 1.0)
combined.addSubspace(pyompl.EmptyStateSpace(), 0.0)  # weight=0, no contribution
combined.setup()
print(f"  Compound with EmptyStateSpace: dim={combined.getDimension()}")
check(combined.getDimension() == 3, "compound dim == 3 (empty adds 0)")


# ==============================================================
# PART 2: HybridTimeStateSpace
# ==============================================================
print("\n" + "=" * 60)
print("PART 2: HybridTimeStateSpace")
print("=" * 60)
print("""
WHAT IS HybridTimeStateSpace?
------------------------------
Extends TimeStateSpace with a JUMP counter.
A state has TWO components:
  position  — continuous time value (double)
  jumps     — discrete jump counter (unsigned int)

WHAT ARE JUMPS?
  In HYBRID SYSTEMS, a robot alternates between:
    CONTINUOUS dynamics: normal physics (differential equations)
    DISCRETE events:     sudden instantaneous changes (jumps)

  Examples of jumps:
    - A bouncing ball hitting the ground (velocity reverses instantly)
    - A robot foot making/breaking contact with ground
    - A gear shift in a vehicle
    - A valve opening/closing

  The jump counter tracks HOW MANY discrete events have occurred.
  This lets planners reason about WHICH phase of the hybrid system
  the robot is in.

C++ class:   ompl::base::HybridTimeStateSpace
C++ header:  ompl/base/spaces/HybridTimeStateSpace.h
Inherits:    StateSpace

TWO-LEVEL BOUNDING (independent):
  setTimeBounds(min, max)    → bounds on position (time)
  setJumpBounds(min, max)    → bounds on jumps count
  Each can be bounded/unbounded independently.
  Unbounded = default = trivial behaviour (like TimeStateSpace unbounded)

DISTANCE:
  |position1 - position2| + |jumps1 - jumps2|
  Combines continuous time distance with discrete jump distance.

INTERPOLATION:
  position: linear   result.position = from + t*(to - from)
  jumps:    rounded  result.jumps    = round(from + t*(to - from))
""")

print("-- Construction (unbounded) --")
# C++: HybridTimeStateSpace space;
hspace = pyompl.HybridTimeStateSpace()
hspace.setName("BallHybridTime")
hspace.setup()

print(f"  getName()          = {hspace.getName()}")
print(f"  getDimension()     = {hspace.getDimension()}")
print(f"  isTimeBounded()    = {hspace.isTimeBounded()}")
print(f"  areJumpsBounded()  = {hspace.areJumpsBounded()}")
print(f"  getMaximumExtent() = {hspace.getMaximumExtent()}")
print(f"  repr: {repr(hspace)}")

check(hspace.getDimension() == 1,         "dimension == 1")
check(hspace.isTimeBounded() is False,    "time unbounded initially")
check(hspace.areJumpsBounded() is False,  "jumps unbounded initially")
check(approx(hspace.getMaximumExtent(), 1.0), "unbounded extent == 1")

# ---------------------------------------------------------------
# HybridTimeState — position and jumps
# C++: state->position = 2.5;  state->jumps = 3;
# ---------------------------------------------------------------
print("\n-- HybridTimeState — position and jumps fields --")
s1 = hspace.allocState()
s2 = hspace.allocState()

s1.position = 2.5
s1.jumps    = 3
print(f"  s1: position={s1.position}, jumps={s1.jumps}")
check(s1.position == 2.5, "position set correctly")
check(s1.jumps == 3,      "jumps set correctly")

s2.position = 7.0
s2.jumps    = 5
print(f"  s2: position={s2.position}, jumps={s2.jumps}")
print(f"  repr(s1) = {repr(s1)}")

# ---------------------------------------------------------------
# setTimeBounds / setJumpBounds
# C++: space->setTimeBounds(0.0, 10.0);
# C++: space->setJumpBounds(0, 5);
# ---------------------------------------------------------------
print("\n-- setTimeBounds and setJumpBounds --")
hspace.setTimeBounds(0.0, 10.0)
hspace.setJumpBounds(0, 5)

print(f"  After setTimeBounds(0,10) + setJumpBounds(0,5):")
print(f"  isTimeBounded()    = {hspace.isTimeBounded()}")
print(f"  areJumpsBounded()  = {hspace.areJumpsBounded()}")
print(f"  getMinTimeBound()  = {hspace.getMinTimeBound()}")
print(f"  getMaxTimeBound()  = {hspace.getMaxTimeBound()}")
print(f"  getMinJumpsBound() = {hspace.getMinJumpsBound()}")
print(f"  getMaxJumpBound()  = {hspace.getMaxJumpBound()}")

check(hspace.isTimeBounded() is True,  "time bounded after setTimeBounds")
check(hspace.areJumpsBounded() is True, "jumps bounded after setJumpBounds")
check(hspace.getMinTimeBound() == 0.0,  "min time == 0")
check(hspace.getMaxTimeBound() == 10.0, "max time == 10")
check(hspace.getMinJumpsBound() == 0,   "min jumps == 0")
check(hspace.getMaxJumpBound() == 5,    "max jumps == 5")

# ---------------------------------------------------------------
# satisfiesBounds and enforceBounds
# ---------------------------------------------------------------
print("\n-- satisfiesBounds and enforceBounds --")
s = hspace.allocState()

# Inside bounds
s.position, s.jumps = 5.0, 3
print(f"  (5.0, 3): satisfiesBounds = {hspace.satisfiesBounds(s)}")
check(hspace.satisfiesBounds(s) is True, "(5,3) in bounds")

# Outside bounds — position too high
s.position, s.jumps = 15.0, 3
print(f"  (15.0, 3): satisfiesBounds = {hspace.satisfiesBounds(s)}")
check(hspace.satisfiesBounds(s) is False, "(15,3) out of bounds")

# enforceBounds
s.position, s.jumps = 15.0, 8  # both out of bounds
hspace.enforceBounds(s)
print(f"  After enforceBounds on (15.0, 8): position={s.position}, jumps={s.jumps}")
check(s.position == 10.0, "position clamped to 10.0")
check(s.jumps == 5,       "jumps clamped to 5")
check(hspace.satisfiesBounds(s), "now satisfies bounds")

hspace.freeState(s)

# ---------------------------------------------------------------
# distance — |pos1-pos2| + |jumps1-jumps2|
# C++: space->distance(state1, state2)
# ---------------------------------------------------------------
print("\n-- distance() = |position1-position2| + |jumps1-jumps2| --")
s1.position, s1.jumps = 2.0, 1
s2.position, s2.jumps = 5.0, 3
d = hspace.distance(s1, s2)
expected = abs(2.0 - 5.0) + abs(1 - 3)
print(f"  d((2.0,j=1), (5.0,j=3)) = {d:.4f}  (|2-5|+|1-3| = {expected})")
check(approx(d, expected), "distance = pos_diff + jump_diff")

# Same state → distance = 0
s1.position, s1.jumps = 4.0, 2
s2.position, s2.jumps = 4.0, 2
d = hspace.distance(s1, s2)
print(f"  d(same, same) = {d:.4f}  (should be 0)")
check(approx(d, 0.0), "same state has distance 0")

# Only position differs
s1.position, s1.jumps = 1.0, 3
s2.position, s2.jumps = 4.0, 3
d = hspace.distance(s1, s2)
print(f"  d(pos=1,j=3 vs pos=4,j=3) = {d:.4f}  (only position differs = 3.0)")
check(approx(d, 3.0), "only position distance = 3.0")

# Only jumps differ
s1.position, s1.jumps = 5.0, 0
s2.position, s2.jumps = 5.0, 4
d = hspace.distance(s1, s2)
print(f"  d(pos=5,j=0 vs pos=5,j=4) = {d:.4f}  (only jumps differ = 4.0)")
check(approx(d, 4.0), "only jumps distance = 4.0")

# ---------------------------------------------------------------
# equalStates
# ---------------------------------------------------------------
print("\n-- equalStates() — both position AND jumps must match --")
e1 = hspace.allocState(); e1.position, e1.jumps = 3.0, 2
e2 = hspace.allocState(); e2.position, e2.jumps = 3.0, 2
e3 = hspace.allocState(); e3.position, e3.jumps = 3.0, 3  # diff jumps

print(f"  equalStates((3.0,j=2),(3.0,j=2)) = {hspace.equalStates(e1,e2)}")
print(f"  equalStates((3.0,j=2),(3.0,j=3)) = {hspace.equalStates(e1,e3)}")
check(hspace.equalStates(e1, e2) is True,  "identical → equal")
check(hspace.equalStates(e1, e3) is False, "diff jumps → not equal")
hspace.freeState(e1); hspace.freeState(e2); hspace.freeState(e3)

# ---------------------------------------------------------------
# copyState and cloneState
# ---------------------------------------------------------------
print("\n-- copyState and cloneState --")
s1.position, s1.jumps = 6.0, 4
hspace.copyState(s2, s1)
print(f"  After copyState: s2.position={s2.position}, s2.jumps={s2.jumps}")
check(s2.position == 6.0 and s2.jumps == 4, "copyState copies both fields")

clone = hspace.cloneState(s1)
check(clone.position == s1.position and clone.jumps == s1.jumps,
      "cloneState copies both fields")
hspace.freeState(clone)

# ---------------------------------------------------------------
# interpolate — linear for position, rounded for jumps
# C++: space->interpolate(from, to, t, result);
# ---------------------------------------------------------------
print("\n-- interpolate(from, to, t, result) --")
f = hspace.allocState(); f.position, f.jumps = 0.0, 0
t_s = hspace.allocState(); t_s.position, t_s.jumps = 10.0, 4
r = hspace.allocState()

for step_t in [0.0, 0.25, 0.5, 0.75, 1.0]:
    hspace.interpolate(f, t_s, step_t, r)
    exp_pos = step_t * 10.0
    exp_jumps = round(step_t * 4)
    print(f"  t={step_t}: position={r.position:.2f} jumps={r.jumps} "
          f"(expected pos={exp_pos:.2f} jumps={exp_jumps})")
    check(approx(r.position, exp_pos), f"position at t={step_t}")
    check(r.jumps == exp_jumps, f"jumps at t={step_t}")

hspace.freeState(f); hspace.freeState(t_s); hspace.freeState(r)

# ---------------------------------------------------------------
# getSerializationLength
# ---------------------------------------------------------------
slen = hspace.getSerializationLength()
print(f"\n-- getSerializationLength() = {slen} bytes --")
print(f"  = sizeof(double) + sizeof(unsigned int) = 8 + 4 = 12")
check(slen == 12, "serialization length == 12")

# ---------------------------------------------------------------
# Sampling in bounded mode
# ---------------------------------------------------------------
print("\n-- Sampling in bounded mode --")
sampler = hspace.allocDefaultStateSampler()
ss = hspace.allocState()
print("  sampleUniform() × 5:")
for i in range(5):
    hspace.sampleUniform(ss)
    valid = hspace.satisfiesBounds(ss)
    print(f"  sample {i}: position={ss.position:.3f} jumps={ss.jumps} valid={valid}")
    check(valid, f"sample {i} in bounds")
hspace.freeState(ss)

# ---------------------------------------------------------------
# printState / printSettings
# ---------------------------------------------------------------
print("\n-- printState / printSettings --")
ps = hspace.allocState(); ps.position = 3.7; ps.jumps = 2
print(f"  printState: '{hspace.printState(ps).strip()}'")
check(len(hspace.printState(ps)) > 0, "printState non-empty")
hspace.freeState(ps)

cfg = hspace.printSettings()
print(f"  printSettings:\n{cfg}")
check(len(cfg) > 0, "printSettings non-empty")

hspace.freeState(s1); hspace.freeState(s2)


# ---------------------------------------------------------------
# Real-world: bouncing ball hybrid system
# ---------------------------------------------------------------
print("\n-- Real-world: Bouncing ball hybrid system --")
print("""
A ball is dropped at t=0. It bounces off the ground multiple times.
  position = time elapsed
  jumps    = number of bounces so far
""")
ball = pyompl.HybridTimeStateSpace()
ball.setName("BouncingBall")
ball.setTimeBounds(0.0, 5.0)   # 5 seconds
ball.setJumpBounds(0, 10)      # up to 10 bounces
ball.setup()

start = ball.allocState(); start.position = 0.0; start.jumps = 0
goal  = ball.allocState(); goal.position  = 4.5; goal.jumps  = 7

print(f"  Start: t={start.position}s, {start.jumps} bounces")
print(f"  Goal:  t={goal.position}s, {goal.jumps} bounces")
print(f"  Distance: {ball.distance(start, goal):.4f}")
print(f"  (|4.5-0| + |7-0| = 4.5 + 7 = 11.5)")
check(approx(ball.distance(start, goal), 11.5), "ball distance correct")

wp = ball.allocState()
print(f"  Waypoints:")
for step_t in [0.0, 0.25, 0.5, 0.75, 1.0]:
    ball.interpolate(start, goal, step_t, wp)
    print(f"  t={step_t}: time={wp.position:.2f}s, bounces={wp.jumps}")
    check(ball.satisfiesBounds(wp), f"waypoint t={step_t} in bounds")

ball.freeState(start); ball.freeState(goal); ball.freeState(wp)


# ==============================================================
# PART 3: WrapperStateSpace
# ==============================================================
print("\n" + "=" * 60)
print("PART 3: WrapperStateSpace")
print("=" * 60)
print("""
WHAT IS WrapperStateSpace?
---------------------------
WrapperStateSpace is a DECORATOR pattern around any StateSpacePtr.
It transparently forwards ALL operations to the inner (wrapped) space.

C++ class:   ompl::base::WrapperStateSpace
C++ header:  ompl/base/spaces/WrapperStateSpace.h
Inherits:    StateSpace

HOW IT WORKS:
  Every method call is forwarded to the wrapped space:
    wrapper.distance(s1, s2) → innerSpace->distance(s1.inner, s2.inner)
    wrapper.interpolate(...)  → innerSpace->interpolate(...)
    wrapper.getDimension()    → innerSpace->getDimension()

WHY USE IT?
  1. Override specific methods while keeping all other behaviour
     (subclass WrapperStateSpace and override just what you need)
  2. Add extra state information without modifying the inner space
  3. Used in OMPL constraint planning (ProjectedStateSpace, etc.)

STATE STRUCTURE:
  WrapperState.getState() → pointer to inner space's state
  The inner state is managed by the wrapper.

CONSTRUCTION:
  wrapper = WrapperStateSpace(innerSpace)  ← wraps any StateSpacePtr

NOTE: WrapperStateSpace is rarely used directly by end users.
It is a base class for specialised wrappers.
""")

# ---------------------------------------------------------------
# Construction — wrap a RealVectorStateSpace
# C++: WrapperStateSpace wrapper(space);
# ---------------------------------------------------------------
print("-- Construction --")
inner = pyompl.RealVectorStateSpace(2)
inner_bounds = pyompl.RealVectorBounds(2)
inner_bounds.setLow(-5.0); inner_bounds.setHigh(5.0)
inner.setBounds(inner_bounds)

wrapper = pyompl.WrapperStateSpace(inner)
wrapper.setup()

print(f"  Wrapping: RealVectorStateSpace(2)")
print(f"  getName()        = {wrapper.getName()}")
print(f"  getDimension()   = {wrapper.getDimension()}")
print(f"  getMaximumExtent = {wrapper.getMaximumExtent():.4f}")
print(f"  isCompound()     = {wrapper.isCompound()}")
print(f"  isDiscrete()     = {wrapper.isDiscrete()}")
print(f"  isMetricSpace()  = {wrapper.isMetricSpace()}")
print(f"  repr: {repr(wrapper)}")

check(wrapper.getDimension() == 2,       "dim == 2 (from inner space)")
check(wrapper.isCompound() is False,     "not compound")
check(wrapper.isMetricSpace() is True,   "metric space")

# ---------------------------------------------------------------
# getSpace() — access the inner space
# C++: space->getSpace()
# ---------------------------------------------------------------
print("\n-- getSpace() — access the wrapped inner space --")
inner_ref = wrapper.getSpace()
print(f"  getSpace().getName()      = {inner_ref.getName()}")
print(f"  getSpace().getDimension() = {inner_ref.getDimension()}")
check(inner_ref.getDimension() == 2, "inner space dim == 2")

# ---------------------------------------------------------------
# allocState / freeState
# The wrapper allocates a WrapperState containing an inner state
# C++: space->allocState() → new WrapperStateSpace::StateType(innerSpace->allocState())
# ---------------------------------------------------------------
print("\n-- allocState() / freeState() --")
ws1 = wrapper.allocState()
ws2 = wrapper.allocState()
print(f"  Allocated ws1 and ws2 (WrapperState objects)")
check(ws1 is not None, "allocState returns non-None")

# ---------------------------------------------------------------
# getState() — access inner state from WrapperState
# C++: state->getState()  → inner State*
# ---------------------------------------------------------------
print("\n-- WrapperState.getState() — access inner state --")
inner_s1 = ws1.getState()
inner_s2 = ws2.getState()
print(f"  ws1.getState() = {inner_s1}")
print(f"  ws1 repr: {repr(ws1)}")
check(inner_s1 is not None, "getState() returns non-None")

# ---------------------------------------------------------------
# printSettings — delegates to inner space
# ---------------------------------------------------------------
print("\n-- printSettings() — delegates to inner space --")
cfg = wrapper.printSettings()
print(f"  printSettings():\n{cfg}")
check(len(cfg) > 0, "printSettings non-empty")

# ---------------------------------------------------------------
# Wrapper with SO2StateSpace
# ---------------------------------------------------------------
print("\n-- Wrapping SO2StateSpace --")
so2_inner = pyompl.SO2StateSpace()
so2_inner.setup()
so2_wrapper = pyompl.WrapperStateSpace(so2_inner)
so2_wrapper.setup()

print(f"  Wrapping SO2StateSpace:")
print(f"  getName()        = {so2_wrapper.getName()}")
print(f"  getDimension()   = {so2_wrapper.getDimension()}")
print(f"  getMaximumExtent = {so2_wrapper.getMaximumExtent():.6f}  (π)")
check(so2_wrapper.getDimension() == 1, "SO2 wrapper dim == 1")
check(approx(so2_wrapper.getMaximumExtent(), math.pi), "max extent == π")

# ---------------------------------------------------------------
# Wrapper with DiscreteStateSpace
# ---------------------------------------------------------------
print("\n-- Wrapping DiscreteStateSpace(0, 5) --")
disc_inner = pyompl.DiscreteStateSpace(0, 5)
disc_inner.setup()
disc_wrapper = pyompl.WrapperStateSpace(disc_inner)
disc_wrapper.setup()

print(f"  Wrapping DiscreteStateSpace(0,5):")
print(f"  isDiscrete()     = {disc_wrapper.isDiscrete()}")
print(f"  getDimension()   = {disc_wrapper.getDimension()}")
check(disc_wrapper.isDiscrete() is True, "discrete wrapper isDiscrete=True")

# ---------------------------------------------------------------
# freeState
# ---------------------------------------------------------------
print("\n-- freeState() --")
wrapper.freeState(ws1)
wrapper.freeState(ws2)
print("  freeState() called for ws1 and ws2")
check(True, "freeState completes without error")


print("\n" + "=" * 60)
print("ALL SECTIONS COMPLETE")
print("=" * 60)
print("""
What we covered:

  EmptyStateSpace:
    constructor (no args), getName (='EmptySpace')
    getDimension (=0), getMaximumExtent (=0), getMeasure (=0)
    setup (no-op)
    Use: placeholder in multi-level planners

  HybridTimeStateSpace:
    constructor (unbounded), setName/getName
    HybridTimeState: .position (time), .jumps (jump count)
    setTimeBounds, setJumpBounds (independent)
    getMinTimeBound, getMaxTimeBound, getMinJumpsBound, getMaxJumpBound
    isTimeBounded, areJumpsBounded
    getDimension (=1), getMaximumExtent, getMeasure
    satisfiesBounds, enforceBounds (both position and jumps)
    copyState, cloneState, equalStates
    getSerializationLength (=12 = 8+4)
    distance = |pos1-pos2| + |jumps1-jumps2|
    interpolate (linear pos, rounded jumps)
    allocState, freeState
    sampleUniform, sampleUniformNear, sampleGaussian
    printState, printSettings, setup
    Use: bouncing ball, legged robots, contact planning

  WrapperStateSpace:
    constructor (wraps any StateSpacePtr)
    getSpace() → inner StateSpacePtr
    allocState() → WrapperState (contains inner state)
    WrapperState.getState() → inner State pointer
    freeState() (frees wrapper + inner state)
    All other methods delegate to inner space
    Use: base class for constraint spaces, decorator pattern

Note on Dubins3DMotionValidator:
  Template class — will be bound after SpaceInformation is done.

Next: examples/10_space_information.py
""")