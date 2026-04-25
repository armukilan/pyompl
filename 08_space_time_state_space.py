"""
===============================================================
pyompl Example 08 — SpaceTimeStateSpace
===============================================================

WHAT IS SpaceTimeStateSpace?
-----------------------------
SpaceTimeStateSpace combines ANY position state space with TIME.
It is a CompoundStateSpace:
  Subspace 0: any StateSpacePtr  (position/configuration)
  Subspace 1: TimeStateSpace     (time)

WHY IS THIS DIFFERENT FROM JUST CompoundStateSpace?
----------------------------------------------------
A plain CompoundStateSpace has a standard weighted distance.
SpaceTimeStateSpace has a VELOCITY-AWARE distance:

  The robot has a maximum velocity vMax.
  If the robot CANNOT physically travel from position A to position B
  in the available time (given vMax), the distance returns INFINITY.

  Reachability check:
    space_dist = distance in position space
    time_diff  = |t2 - t1|
    if space_dist / time_diff > vMax  →  distance = INFINITY
    otherwise                         →  normal weighted distance

WHY DOES THIS MATTER?
  In kinodynamic planning, the robot cannot teleport.
  SpaceTimeStateSpace encodes the physics directly in the metric.
  Planners like SpaceTimeRRT use this to find feasible trajectories.

IMPORTANT NOTES:
  - isMetricSpace() = False  (triangle inequality not guaranteed)
  - getMaximumExtent() = infinity
  - States are CompoundState with (position_substate, time_substate)
  - Access state components through getSpaceComponent() and getTimeComponent()

CONSTRUCTOR:
  SpaceTimeStateSpace(spaceComponent, vMax=1.0, timeWeight=0.5)

C++ HEADER: ompl/base/spaces/SpaceTimeStateSpace.h
C++ CLASS:  ompl::base::SpaceTimeStateSpace

RUN THIS FILE:
    python examples/08_space_time_state_space.py
===============================================================
"""

import math
import pyompl

def check(condition, msg):
    print(f"  {'PASS' if condition else 'FAIL'} — {msg}")

def approx(a, b, tol=1e-6):
    return abs(a - b) < tol

INF = float('inf')

print("=" * 60)
print("SECTION 1: Construction and Basic Properties")
print("=" * 60)
print("""
SETUP SEQUENCE:
  1. Create the position (space) component
  2. Create SpaceTimeStateSpace(spaceComponent, vMax, timeWeight)
  3. Set position bounds on the space component
  4. Call setTimeBounds(minTime, maxTime)
  5. Call setup()

C++:
  auto spaceComp = std::make_shared<RealVectorStateSpace>(2);
  RealVectorBounds bounds(2);
  bounds.setLow(-10); bounds.setHigh(10);
  spaceComp->setBounds(bounds);
  auto space = std::make_shared<SpaceTimeStateSpace>(spaceComp, 1.0, 0.5);
  space->setTimeBounds(0.0, 20.0);
  space->setup();
""")

# Create position component: 2D plane
pos_space = pyompl.RealVectorStateSpace(2)
pos_bounds = pyompl.RealVectorBounds(2)
pos_bounds.setLow(-10.0)
pos_bounds.setHigh(10.0)
pos_space.setBounds(pos_bounds)

# Create SpaceTimeStateSpace
# vMax=2.0 means robot moves at most 2 units per second
# timeWeight=0.5 means time has half the weight of position in distance
space = pyompl.SpaceTimeStateSpace(pos_space, 2.0, 0.5)
space.setTimeBounds(0.0, 20.0)
space.setup()

print(f"  getName()           = {space.getName()}")
print(f"  getDimension()      = {space.getDimension()}  (2 position + 1 time)")
print(f"  getVMax()           = {space.getVMax()}")
print(f"  getMaximumExtent()  = {space.getMaximumExtent()}")
print(f"  isMetricSpace()     = {space.isMetricSpace()}")
print(f"  getSubspaceCount()  = {space.getSubspaceCount()}")
print(f"  repr: {repr(space)}")

check(space.getDimension() == 3, "getDimension() == 3 (2D + time)")
check(space.getVMax() == 2.0, "vMax == 2.0")
check(space.getMaximumExtent() == INF, "max extent == infinity")
check(space.isMetricSpace() is False, "NOT a metric space")
check(space.getSubspaceCount() == 2, "2 subspaces (position + time)")

# setName
space.setName("SpaceTimePlane")
check("SpaceTimePlane" in space.getName(), "setName/getName work")


print("\n" + "=" * 60)
print("SECTION 2: getVMax and setVMax")
print("=" * 60)
print("""
vMax = maximum velocity of the robot (position units / time unit).
It controls the distance function:
  - Higher vMax: robot is faster, more states are reachable
  - Lower vMax:  robot is slower, fewer states are reachable

getVMax()   — C++: space->getVMax()
setVMax(v)  — C++: space->setVMax(v);
""")

print(f"  Initial vMax: {space.getVMax()}")
space.setVMax(5.0)
print(f"  After setVMax(5.0): {space.getVMax()}")
check(space.getVMax() == 5.0, "setVMax works")
space.setVMax(2.0)  # reset for rest of tests
check(space.getVMax() == 2.0, "reset to 2.0")


print("\n" + "=" * 60)
print("SECTION 3: getSpaceComponent and getTimeComponent")
print("=" * 60)
print("""
getSpaceComponent()
    Returns the position subspace (subspace[0]) as StateSpacePtr.
    C++: space->getSpaceComponent()

getTimeComponent()
    Returns the TimeStateSpace (subspace[1]) as a raw pointer.
    C++: space->getTimeComponent()
    Use to check time bounds: getTimeComponent().getMinTimeBound()
""")

space_comp = space.getSpaceComponent()
time_comp  = space.getTimeComponent()

print(f"  getSpaceComponent() name: {space_comp.getName()}")
print(f"  getTimeComponent() type: {type(time_comp).__name__}")
print(f"  Time bounds: [{time_comp.getMinTimeBound()}, {time_comp.getMaxTimeBound()}]")
print(f"  Time isBounded: {time_comp.isBounded()}")

check(time_comp.isBounded() is True, "time component is bounded")
check(time_comp.getMinTimeBound() == 0.0, "min time == 0.0")
check(time_comp.getMaxTimeBound() == 20.0, "max time == 20.0")


print("\n" + "=" * 60)
print("SECTION 4: setTimeBounds and updateEpsilon")
print("=" * 60)
print("""
setTimeBounds(lb, ub)
    Sets time bounds and activates bounded time mode.
    C++: space->setTimeBounds(lb, ub);
    Equivalent to: getTimeComponent()->setBounds(lb, ub)

updateEpsilon()
    Recalculates internal epsilon after bounds/vMax changes.
    C++: space->updateEpsilon();
    Call after changing time bounds or vMax.
""")

space.setTimeBounds(0.0, 30.0)
space.updateEpsilon()
tc = space.getTimeComponent()
print(f"  After setTimeBounds(0, 30): [{tc.getMinTimeBound()}, {tc.getMaxTimeBound()}]")
check(tc.getMaxTimeBound() == 30.0, "time upper bound updated to 30")

space.setTimeBounds(0.0, 20.0)  # reset
space.updateEpsilon()
check(tc.getMaxTimeBound() == 20.0, "reset to 20")


print("\n" + "=" * 60)
print("SECTION 5: State Structure — How to Access Substates")
print("=" * 60)
print("""
SpaceTimeStateSpace states are CompoundStateSpace states.
The compound state has two substates:
  substate[0] = position state  (e.g. RealVectorState for x,y)
  substate[1] = TimeState       (time value)

In OMPL C++, you access them like this:
  const auto *compound = state->as<CompoundState>();
  const auto *posState = compound->as<RealVectorStateSpace::StateType>(0);
  const auto *timeState = compound->as<TimeStateSpace::StateType>(1);

In pyompl, the SpaceTimeStateSpace state is a CompoundState.
We use getStateTime(state) to get the time directly.
For position, allocate a separate position state and use the
space component's operations.

PRACTICAL APPROACH in pyompl:
  Because SpaceTimeStateSpace is a CompoundStateSpace, its states
  are managed internally. The most practical way to work with them
  is through the space's distance, interpolate, and sampler methods,
  which handle the compound structure automatically.

  getStateTime(state) → extract time from any SpaceTimeStateSpace state
""")

# Allocate states
# s1 = space.allocState()
# s2 = space.allocState()

# print("  Allocated s1 and s2 (CompoundState objects)")
# check(s1 is not None, "allocState returns non-None")

# # getStateTime — extract time from compound state
# print(f"\n-- SpaceTimeStateSpace.getStateTime(state) --")
# print("  This is a STATIC method — call as SpaceTimeStateSpace.getStateTime(s)")
# print("  C++: SpaceTimeStateSpace::getStateTime(state)  (static)")
# print("  Returns the time value (substate[1].position) of the compound state.")

# # We can check what getStateTime returns after allocation
# # (states are uninitialised but let's test the method exists)
# try:
#     t = pyompl.SpaceTimeStateSpace.getStateTime(s1)
#     print(f"  getStateTime(s1) = {t}")
#     check(True, "getStateTime() callable")
# except Exception as e:
#     print(f"  getStateTime raised: {e}")
# States are allocated via sampler — allocState returns raw State*
# which works only with space methods, not directly inspectable
sampler = space.allocDefaultStateSampler()
s1 = space.allocState()
s2 = space.allocState()

print("  Allocated s1 and s2 via allocState()")
check(s1 is not None, "allocState returns non-None")

# Fill with valid values via sampler
sampler.sampleUniform(s1)
sampler.sampleUniform(s2)

# getStateTime — extract time from compound state
print(f"\n-- SpaceTimeStateSpace.getStateTime(state) --")
t1 = pyompl.SpaceTimeStateSpace.getStateTime(s1)
t2 = pyompl.SpaceTimeStateSpace.getStateTime(s2)
print(f"  getStateTime(s1) = {t1:.4f}s")
print(f"  getStateTime(s2) = {t2:.4f}s")
check(0.0 <= t1 <= 20.0, "s1 time in [0,20]")
check(0.0 <= t2 <= 20.0, "s2 time in [0,20]")

# Test distance between them
d = space.distance(s1, s2)
ds = space.distanceSpace(s1, s2)
dt = space.distanceTime(s1, s2)
print(f"  distanceSpace = {ds:.4f}, distanceTime = {dt:.4f}, distance = {d}")

space.freeState(s1)
space.freeState(s2)


print("\n" + "=" * 60)
print("SECTION 6: distance — The Core of SpaceTimeStateSpace")
print("=" * 60)
print("""
distance(state1, state2)
    THE KEY FUNCTION of SpaceTimeStateSpace.
    C++: space->distance(state1, state2)

    Returns INFINITY if state2 is physically unreachable from state1
    given the maximum velocity vMax.

    Reachability condition:
      space_dist = distanceSpace(state1, state2)
      time_diff  = distanceTime(state1, state2) = |t1 - t2|
      REACHABLE if: space_dist / time_diff <= vMax
      (i.e., robot has enough time to cover the distance)

distanceSpace(state1, state2)
    Distance in position component only.
    C++: space->distanceSpace(state1, state2)

distanceTime(state1, state2)
    Distance in time component only = |t1 - t2|.
    C++: space->distanceTime(state1, state2)

timeToCoverDistance(state1, state2)
    Minimum time to travel between positions = space_dist / vMax
    C++: space->timeToCoverDistance(state1, state2)
""")

# To demonstrate distance, we use the sampler to get valid states
# and then check distances
sampler = space.allocDefaultStateSampler()

# Sample several states and check distance properties
states = []
for i in range(5):
    st = space.allocState()
    sampler.sampleUniform(st)
    states.append(st)
    t_val = pyompl.SpaceTimeStateSpace.getStateTime(st)
    print(f"  State {i}: time={t_val:.3f}")

print("\n-- distance() between sampled states --")
for i in range(min(3, len(states))):
    for j in range(i+1, min(4, len(states))):
        d = space.distance(states[i], states[j])
        d_space = space.distanceSpace(states[i], states[j])
        d_time  = space.distanceTime(states[i], states[j])
        t2c     = space.timeToCoverDistance(states[i], states[j])
        if d == INF:
            print(f"  d(state{i}, state{j}) = INFINITY  "
                  f"(space={d_space:.3f}, time={d_time:.3f}, need={t2c:.3f}s)")
        else:
            print(f"  d(state{i}, state{j}) = {d:.4f}  "
                  f"(space={d_space:.3f}, time={d_time:.3f}, need={t2c:.3f}s)")

for st in states:
    space.freeState(st)


print("\n" + "=" * 60)
print("SECTION 7: distanceSpace, distanceTime, timeToCoverDistance")
print("=" * 60)
print("""
These three methods let you inspect the COMPONENTS of the distance
independently:

distanceSpace(s1, s2)
    Only the position distance. Ignores time.
    C++: space->distanceSpace(s1, s2)

distanceTime(s1, s2)
    Only the time distance = |t1 - t2|. Ignores position.
    C++: space->distanceTime(s1, s2)

timeToCoverDistance(s1, s2)
    Minimum time needed = space_dist / vMax.
    If this > time_diff → motion is infeasible → distance = INFINITY.
    C++: space->timeToCoverDistance(s1, s2)
""")

# Sample two states to demonstrate
sa = space.allocState()
sb = space.allocState()
sampler.sampleUniform(sa)
sampler.sampleUniform(sb)

ta = pyompl.SpaceTimeStateSpace.getStateTime(sa)
tb = pyompl.SpaceTimeStateSpace.getStateTime(sb)

d_full  = space.distance(sa, sb)
d_sp    = space.distanceSpace(sa, sb)
d_t     = space.distanceTime(sa, sb)
t2c     = space.timeToCoverDistance(sa, sb)

print(f"  State A time: {ta:.3f}s")
print(f"  State B time: {tb:.3f}s")
print(f"  distanceSpace(A,B)         = {d_sp:.4f}  (position only)")
print(f"  distanceTime(A,B)          = {d_t:.4f}  (|{ta:.2f} - {tb:.2f}|)")
print(f"  timeToCoverDistance(A,B)   = {t2c:.4f}  (= {d_sp:.4f} / {space.getVMax()})")
print(f"  distance(A,B)              = {d_full}")
if d_full == INF:
    print(f"  → INFINITY because {t2c:.4f}s needed > {d_t:.4f}s available")
else:
    print(f"  → Finite because {t2c:.4f}s needed <= {d_t:.4f}s available")

check(approx(d_t, abs(ta - tb)), "distanceTime == |ta - tb|")
check(approx(t2c, d_sp / space.getVMax(), tol=1e-4),
      "timeToCoverDistance == space_dist / vMax")

space.freeState(sa)
space.freeState(sb)


print("\n" + "=" * 60)
print("SECTION 8: isMetricSpace and getMaximumExtent")
print("=" * 60)
print("""
isMetricSpace()
    Always returns False for SpaceTimeStateSpace.
    C++: space->isMetricSpace()
    
    Reason: The distance function can return INFINITY, which means
    the triangle inequality d(A,C) <= d(A,B) + d(B,C) is NOT
    guaranteed. Some planners require metric spaces — check docs
    before using SpaceTimeStateSpace with a specific planner.

getMaximumExtent()
    Always returns infinity for SpaceTimeStateSpace.
    C++: space->getMaximumExtent()
    
    Reason: Even with bounded time, two states can have infinite
    distance (if the velocity constraint is violated). So no finite
    upper bound on distance exists.
""")

print(f"  isMetricSpace()    = {space.isMetricSpace()}")
print(f"  getMaximumExtent() = {space.getMaximumExtent()}")
check(space.isMetricSpace() is False, "NOT a metric space")
check(space.getMaximumExtent() == INF, "max extent is infinity")

print("\n  CONSEQUENCE: SpaceTimeRRT and similar planners are designed")
print("  specifically for non-metric spaces with infinite distances.")
print("  Standard RRT/PRM may not work correctly with this space.")


print("\n" + "=" * 60)
print("SECTION 9: Sampling")
print("=" * 60)
print("""
The default sampler samples:
  - Position: uniformly from position bounds
  - Time: uniformly from [minTime, maxTime]

States are valid SpaceTimeStateSpace states ready for planning.
""")

ss = space.allocState()
print("-- sampleUniform() × 5 --")
for i in range(5):
    sampler.sampleUniform(ss)
    t = pyompl.SpaceTimeStateSpace.getStateTime(ss)
    print(f"  sample {i}: time={t:.3f}s")
    check(0.0 <= t <= 20.0, f"time in [0, 20]")

space.freeState(ss)


print("\n" + "=" * 60)
print("SECTION 10: Real-world — Drone with velocity constraint")
print("=" * 60)
print("""
Use case: a drone in 2D that moves at most 3 m/s.
We check which positions are reachable within a time window.

Position space: 2D plane [-20, 20] metres
Time range:     [0, 30] seconds
Max velocity:   3.0 m/s
""")

drone_pos = pyompl.RealVectorStateSpace(2)
dp_bounds = pyompl.RealVectorBounds(2)
dp_bounds.setLow(-20.0)
dp_bounds.setHigh(20.0)
drone_pos.setBounds(dp_bounds)

drone_space = pyompl.SpaceTimeStateSpace(drone_pos, 3.0, 0.5)
drone_space.setName("DroneSpaceTime")
drone_space.setTimeBounds(0.0, 30.0)
drone_space.setup()

print(f"  Drone space: {drone_space.getName()}")
print(f"  vMax = {drone_space.getVMax()} m/s")
print(f"  Time: [{drone_space.getTimeComponent().getMinTimeBound()}, "
      f"{drone_space.getTimeComponent().getMaxTimeBound()}]s")

drone_sampler = drone_space.allocDefaultStateSampler()

# Sample states and analyse reachability
print(f"\n  Sampling states and checking distances:")
drone_states = []
for i in range(6):
    st = drone_space.allocState()
    drone_sampler.sampleUniform(st)
    t = pyompl.SpaceTimeStateSpace.getStateTime(st)
    drone_states.append((st, t))
    print(f"  State {i}: t={t:.2f}s")

print(f"\n  Pairwise distance check (vMax={drone_space.getVMax()}m/s):")
reachable_count = 0
infinite_count  = 0
for i in range(len(drone_states)):
    for j in range(i+1, len(drone_states)):
        si, ti = drone_states[i]
        sj, tj = drone_states[j]
        d = drone_space.distance(si, sj)
        ds = drone_space.distanceSpace(si, sj)
        dt = drone_space.distanceTime(si, sj)
        t2c = drone_space.timeToCoverDistance(si, sj)
        feasible = (d != INF)
        if feasible:
            reachable_count += 1
        else:
            infinite_count += 1
        status = "REACHABLE" if feasible else "UNREACHABLE"
        print(f"  ({i}→{j}): pos_dist={ds:.1f}m, Δt={dt:.1f}s, "
              f"need={t2c:.1f}s → {status}")

print(f"\n  Reachable pairs: {reachable_count}")
print(f"  Unreachable pairs: {infinite_count}")

for st, _ in drone_states:
    drone_space.freeState(st)


print("\n" + "=" * 60)
print("ALL SECTIONS COMPLETE")
print("=" * 60)
print("""
What we covered:

  SpaceTimeStateSpace:
    constructor (spaceComponent, vMax, timeWeight)
    setName/getName
    getDimension (position_dim + 1)
    getVMax, setVMax
    getSpaceComponent (returns position StateSpacePtr)
    getTimeComponent  (returns TimeStateSpace pointer)
    setTimeBounds     (sets time range)
    updateEpsilon     (call after bounds/vMax change)
    isMetricSpace     (always False)
    getMaximumExtent  (always infinity)
    distance          (INFINITY if unreachable given vMax)
    distanceSpace     (position distance only)
    distanceTime      (time distance only = |t1-t2|)
    timeToCoverDistance (= space_dist / vMax)
    getStateTime      (static — extracts time from compound state)
    allocState, freeState
    allocDefaultStateSampler
    getSubspaceCount, getSubspaceWeight
    setup

Key concepts:
  - Velocity-aware distance: returns INFINITY if unreachable
  - NOT a metric space (triangle inequality not guaranteed)
  - States are CompoundState (position + time)
  - Use getStateTime() to extract time value
  - Must call setTimeBounds() before use
  - Designed for kinodynamic/spatio-temporal planners (SpaceTimeRRT)

Note on Dubins3DMotionValidator:
  This is a C++ template class — it cannot be bound directly in pybind11
  without knowing the concrete template parameter. It will be bound
  after SpaceInformation is wrapped (it depends on SI).
""")