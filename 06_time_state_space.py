"""
===============================================================
pyompl Example 06 — TimeStateSpace
===============================================================

WHAT IS TimeStateSpace?
-----------------------
TimeStateSpace represents TIME as a planning dimension.
A state is just ONE double: a moment in time (state.position).

WHY PLAN OVER TIME?
-------------------
In standard motion planning, you plan WHERE the robot goes.
In spatio-temporal or kinodynamic planning, you also plan WHEN.

Examples where time matters:
  - Robot must reach a door exactly when it opens
  - Drone must avoid moving obstacles at specific times
  - Vehicle must cross an intersection during a safe window
  - Robot arm must synchronise with a moving conveyor belt

COMBINING WITH OTHER SPACES:
  TimeStateSpace is usually combined with a position space via
  CompoundStateSpace:
    combined = RealVectorStateSpace(2) + TimeStateSpace()
  This gives (x, y, t) — a spatio-temporal state.

TWO MODES — THIS IS THE KEY CONCEPT:
--------------------------------------
After construction, the space is UNBOUNDED:
  - satisfiesBounds() → always True
  - enforceBounds()   → no-op
  - sampleUniform()   → always position=0
  - getMaximumExtent()→ 1

After setBounds(minTime, maxTime), space is BOUNDED:
  - All operations behave normally
  - sampleUniform() → uniform in [minTime, maxTime]

DISTANCE:   |t1 - t2|  (absolute difference)
INTERPOLATION: linear  (from + t*(to - from))

C++ HEADER: ompl/base/spaces/TimeStateSpace.h
C++ CLASS:  ompl::base::TimeStateSpace
INHERITS:   StateSpace

RUN THIS FILE:
    python examples/06_time_state_space.py
===============================================================
"""

import math
import pyompl

def check(condition, msg):
    print(f"  {'PASS' if condition else 'FAIL'} — {msg}")

def approx(a, b, tol=1e-9):
    return abs(a - b) < tol


print("=" * 60)
print("SECTION 1: Construction — Unbounded Mode")
print("=" * 60)
print("""
After construction, TimeStateSpace is UNBOUNDED.
This is the default mode. No bounds are set.

C++: auto space = std::make_shared<TimeStateSpace>();

In unbounded mode:
  isBounded()         → False
  getMinTimeBound()   → 0
  getMaxTimeBound()   → 0
  getMaximumExtent()  → 1  (convention for unbounded)
  getMeasure()        → 1
  satisfiesBounds()   → always True
  enforceBounds()     → no-op
  sampleUniform()     → always produces position=0
""")

space = pyompl.TimeStateSpace()
space.setup()

print(f"  getName()            = {space.getName()}")
print(f"  getDimension()       = {space.getDimension()}")
print(f"  isBounded()          = {space.isBounded()}")
print(f"  getMinTimeBound()    = {space.getMinTimeBound()}")
print(f"  getMaxTimeBound()    = {space.getMaxTimeBound()}")
print(f"  getMaximumExtent()   = {space.getMaximumExtent()}")
print(f"  getMeasure()         = {space.getMeasure()}")

check(space.getDimension() == 1, "getDimension() == 1")
check(space.isBounded() is False, "isBounded() == False after construction")
check(space.getMinTimeBound() == 0.0, "min bound == 0 when unbounded")
check(space.getMaxTimeBound() == 0.0, "max bound == 0 when unbounded")
check(space.getMaximumExtent() == 1.0, "max extent == 1 when unbounded")

# setName
space.setName("MissionTime")
print(f"\n  After setName: {space.getName()}")
check("MissionTime" in space.getName(), "setName/getName work")


print("\n" + "=" * 60)
print("SECTION 2: TimeState — The State Object")
print("=" * 60)
print("""
A TimeState has exactly ONE field: .position (the time value).

C++ field: state->position
Python:    state.position

Do NOT construct directly — use space.allocState().
""")

s1 = space.allocState()
s2 = space.allocState()

print("-- allocState() --")
check(s1 is not None, "allocState returns non-None")

print("\n-- state.position — read and write --")
s1.position = 3.5
print(f"  s1.position = 3.5")
print(f"  Read back: {s1.position}")
check(approx(s1.position, 3.5), ".position read back correctly")

s1.position = 0.0
print(f"  s1.position = 0.0: {s1.position}")
check(approx(s1.position, 0.0), ".position = 0.0 correct")

s2.position = 10.0
print(f"  s2.position = 10.0: {s2.position}")

print(f"\n  repr(s1) = {repr(s1)}")
print(f"  repr(s2) = {repr(s2)}")
print(f"  repr(space) = {repr(space)}")


print("\n" + "=" * 60)
print("SECTION 3: Unbounded Mode — Special Behaviour")
print("=" * 60)
print("""
When UNBOUNDED:
  satisfiesBounds(state) → ALWAYS True, regardless of position value
  enforceBounds(state)   → no-op, position unchanged
  sampleUniform(state)   → ALWAYS sets position = 0
""")

# satisfiesBounds in unbounded mode
s = space.allocState()

for t_val in [0.0, 100.0, -999.0, 1e10, float('inf')]:
    s.position = t_val
    result = space.satisfiesBounds(s)
    print(f"  satisfiesBounds(position={t_val}) = {result}")

check(space.satisfiesBounds(s) is True,
      "unbounded satisfiesBounds always True")

# enforceBounds in unbounded mode — no-op
s.position = 12345.0
space.enforceBounds(s)
print(f"\n  After enforceBounds on position=12345.0: {s.position}")
check(approx(s.position, 12345.0),
      "unbounded enforceBounds is no-op (position unchanged)")

# sampleUniform in unbounded mode — always 0
sampler = space.allocDefaultStateSampler()
print(f"\n  allocDefaultStateSampler() → {type(sampler).__name__}")
for i in range(3):
    sampler.sampleUniform(s)
    print(f"  sampleUniform() sample {i}: position = {s.position}")
    check(approx(s.position, 0.0), f"unbounded sampleUniform always gives 0")

space.freeState(s)


print("\n" + "=" * 60)
print("SECTION 4: setBounds — Switching to Bounded Mode")
print("=" * 60)
print("""
setBounds(minTime, maxTime)
    Switches the space to BOUNDED mode.
    C++: space->setBounds(minTime, maxTime);

After calling setBounds:
  isBounded()           → True
  getMinTimeBound()     → minTime
  getMaxTimeBound()     → maxTime
  getMaximumExtent()    → maxTime - minTime
  getMeasure()          → maxTime - minTime
  satisfiesBounds()     → True iff position in [minTime, maxTime]
  enforceBounds()       → clamps to [minTime, maxTime]
  sampleUniform()       → uniform in [minTime, maxTime]
""")

# Create a bounded space
bspace = pyompl.TimeStateSpace()
bspace.setName("BoundedMissionTime")
bspace.setBounds(0.0, 10.0)   # time runs from 0 to 10 seconds
bspace.setup()

print(f"  After setBounds(0.0, 10.0):")
print(f"  isBounded()          = {bspace.isBounded()}")
print(f"  getMinTimeBound()    = {bspace.getMinTimeBound()}")
print(f"  getMaxTimeBound()    = {bspace.getMaxTimeBound()}")
print(f"  getMaximumExtent()   = {bspace.getMaximumExtent()}")
print(f"  getMeasure()         = {bspace.getMeasure()}")

check(bspace.isBounded() is True,    "isBounded() == True after setBounds")
check(bspace.getMinTimeBound() == 0.0, "minTime == 0.0")
check(bspace.getMaxTimeBound() == 10.0, "maxTime == 10.0")
check(approx(bspace.getMaximumExtent(), 10.0), "max extent == 10.0")
check(approx(bspace.getMeasure(), 10.0), "measure == 10.0")

print(f"\n  repr(bspace) = {repr(bspace)}")


print("\n" + "=" * 60)
print("SECTION 5: Bounded Mode — satisfiesBounds and enforceBounds")
print("=" * 60)

bs = bspace.allocState()

print("-- satisfiesBounds() in bounded mode [0, 10] --")
for t_val, expected in [(0.0, True), (5.0, True), (10.0, True),
                         (-1.0, False), (10.1, False), (100.0, False)]:
    bs.position = t_val
    result = bspace.satisfiesBounds(bs)
    print(f"  position={t_val:6.1f}: satisfiesBounds = {result}")
    check(result == expected, f"position={t_val} → {expected}")

print("\n-- enforceBounds() in bounded mode [0, 10] --")
test_cases = [(-5.0, 0.0), (15.0, 10.0), (5.0, 5.0), (0.0, 0.0)]
for before, after in test_cases:
    bs.position = before
    bspace.enforceBounds(bs)
    print(f"  {before:6.1f} → {bs.position}")
    check(approx(bs.position, after), f"enforceBounds({before}) == {after}")

bspace.freeState(bs)


print("\n" + "=" * 60)
print("SECTION 6: copyState, cloneState, equalStates")
print("=" * 60)

b1 = bspace.allocState(); b1.position = 3.0
b2 = bspace.allocState(); b2.position = 7.0

print("-- copyState(dst, src) --")
bspace.copyState(b2, b1)
print(f"  src.position=3.0, after copyState: dst.position={b2.position}")
check(approx(b2.position, 3.0), "copyState copies position correctly")

print("\n-- cloneState(src) --")
b1.position = 4.5
clone = bspace.cloneState(b1)
print(f"  src.position=4.5, clone.position={clone.position}")
check(approx(clone.position, 4.5), "cloneState correct")
bspace.freeState(clone)

print("\n-- equalStates() --")
e1 = bspace.allocState(); e1.position = 5.0
e2 = bspace.allocState(); e2.position = 5.0
e3 = bspace.allocState(); e3.position = 6.0
print(f"  equalStates(5.0, 5.0) = {bspace.equalStates(e1, e2)}")
print(f"  equalStates(5.0, 6.0) = {bspace.equalStates(e1, e3)}")
check(bspace.equalStates(e1, e2) is True,  "identical times are equal")
check(bspace.equalStates(e1, e3) is False, "different times not equal")

bspace.freeState(b1); bspace.freeState(b2)
bspace.freeState(e1); bspace.freeState(e2); bspace.freeState(e3)


print("\n" + "=" * 60)
print("SECTION 7: copyToReals, copyFromReals, getSerializationLength")
print("=" * 60)

bs = bspace.allocState()
bs.position = 7.5

reals = bspace.copyToReals(bs)
print(f"  copyToReals(7.5) = {reals}")
check(len(reals) == 1, "copyToReals returns 1-element list")
check(approx(reals[0], 7.5), "value is correct")

bs2 = bspace.allocState()
bspace.copyFromReals(bs2, [3.14])
print(f"  copyFromReals([3.14]): position = {bs2.position}")
check(approx(bs2.position, 3.14), "copyFromReals correct")

slen = bspace.getSerializationLength()
print(f"\n  getSerializationLength() = {slen} bytes  (1 double × 8 bytes)")
check(slen == 8, "serialization length == 8")

bspace.freeState(bs); bspace.freeState(bs2)


print("\n" + "=" * 60)
print("SECTION 8: distance — Absolute Time Difference")
print("=" * 60)
print("""
distance(state1, state2)
    Returns |t1 - t2| — absolute difference in time.
    C++: space->distance(state1, state2)

    Simple 1D Euclidean distance. No wrapping, no weighting.
""")

t1 = bspace.allocState()
t2 = bspace.allocState()

# Same time
t1.position, t2.position = 5.0, 5.0
d = bspace.distance(t1, t2)
print(f"-- d(5.0, 5.0) = {d:.6f}  (should be 0)")
check(approx(d, 0.0), "distance 0 for same time")

# Simple cases
for a_val, b_val, expected in [(0.0, 10.0, 10.0), (3.0, 7.0, 4.0),
                                 (8.0, 2.0, 6.0), (0.0, 0.5, 0.5)]:
    t1.position, t2.position = a_val, b_val
    d = bspace.distance(t1, t2)
    print(f"-- d({a_val}, {b_val}) = {d:.4f}  (expected {expected})")
    check(approx(d, expected), f"|{a_val}-{b_val}| == {expected}")

# Symmetry
t1.position, t2.position = 3.0, 8.0
d_ab = bspace.distance(t1, t2)
d_ba = bspace.distance(t2, t1)
print(f"-- Symmetry: d(3,8)={d_ab}, d(8,3)={d_ba}")
check(approx(d_ab, d_ba), "distance is symmetric")

bspace.freeState(t1); bspace.freeState(t2)


print("\n" + "=" * 60)
print("SECTION 9: interpolate — Linear Time Interpolation")
print("=" * 60)
print("""
interpolate(from, to, t, result)
    result.position = from.position + t * (to.position - from.position)
    C++: space->interpolate(from, to, t, result);

    Simple linear interpolation. t in [0, 1].
""")

f = bspace.allocState()
t_state = bspace.allocState()
r = bspace.allocState()

f.position = 0.0
t_state.position = 10.0

space_for_interp = bspace

for step_t, expected in [(0.0, 0.0), (0.25, 2.5), (0.5, 5.0), (0.75, 7.5), (1.0, 10.0)]:
    space_for_interp.interpolate(f, t_state, step_t, r)
    print(f"  t={step_t}: position={r.position:.4f}  (expected {expected})")
    check(approx(r.position, expected), f"t={step_t} gives {expected}")

# Non-trivial case
f.position = 3.0
t_state.position = 9.0
space_for_interp.interpolate(f, t_state, 0.5, r)
print(f"\n  from=3.0, to=9.0, t=0.5 → {r.position}  (expected 6.0)")
check(approx(r.position, 6.0), "midpoint of [3,9] is 6.0")

bspace.freeState(f); bspace.freeState(t_state); bspace.freeState(r)


print("\n" + "=" * 60)
print("SECTION 10: Bounded Sampler")
print("=" * 60)
print("""
In bounded mode, sampleUniform() samples uniformly from [minTime, maxTime].
sampleUniformNear() and sampleGaussian() also respect bounds.
""")

bs_sampler = bspace.allocDefaultStateSampler()
ss = bspace.allocState()
near_s = bspace.allocState()
near_s.position = 5.0

print("-- sampleUniform() × 5 (bounded [0, 10]) --")
for i in range(5):
    bs_sampler.sampleUniform(ss)
    valid = bspace.satisfiesBounds(ss)
    print(f"  sample {i}: position={ss.position:.4f}  in_bounds={valid}")
    check(valid, f"bounded sample {i} in [0,10]")

print("\n-- sampleUniformNear(near=5.0, distance=2.0) × 3 --")
for i in range(3):
    bs_sampler.sampleUniformNear(ss, near_s, 2.0)
    d = bspace.distance(ss, near_s)
    print(f"  sample {i}: position={ss.position:.4f}  dist from 5.0={d:.4f}")
    check(bspace.satisfiesBounds(ss), f"near sample {i} in bounds")

print("\n-- sampleGaussian(mean=5.0, stdDev=1.0) × 3 --")
mean_s = bspace.allocState(); mean_s.position = 5.0
for i in range(3):
    bs_sampler.sampleGaussian(ss, mean_s, 1.0)
    print(f"  sample {i}: position={ss.position:.4f}  in_bounds={bspace.satisfiesBounds(ss)}")
    check(bspace.satisfiesBounds(ss), f"Gaussian sample {i} in bounds")

bspace.freeState(ss); bspace.freeState(near_s); bspace.freeState(mean_s)


print("\n" + "=" * 60)
print("SECTION 11: printState, printSettings")
print("=" * 60)

ps = bspace.allocState()
ps.position = 4.2
print(f"  printState(4.2): '{bspace.printState(ps).strip()}'")
check(len(bspace.printState(ps)) > 0, "printState non-empty")
bspace.freeState(ps)

cfg = bspace.printSettings()
print(f"\n  printSettings():\n{cfg}")
check(len(cfg) > 0, "printSettings non-empty")


print("\n" + "=" * 60)
print("SECTION 12: Real-world — Spatio-Temporal Planning")
print("=" * 60)
print("""
Use case: a robot must reach a checkpoint within a time window.
We use TimeStateSpace standalone to reason about timing constraints.
In a real planner, this would be combined with a position space.
""")

timing = pyompl.TimeStateSpace()
timing.setName("CheckpointTiming")
timing.setBounds(0.0, 30.0)   # mission runs 0 to 30 seconds
timing.setup()

earliest = timing.allocState(); earliest.position = 8.0    # door opens at t=8
latest   = timing.allocState(); latest.position   = 12.0   # door closes at t=12

print(f"  Door open window: [{earliest.position}, {latest.position}] seconds")

# Check if various arrival times are valid
for t_val in [5.0, 8.0, 10.0, 12.0, 15.0]:
    arrival = timing.allocState()
    arrival.position = t_val
    in_window = (t_val >= earliest.position and t_val <= latest.position)
    in_mission = timing.satisfiesBounds(arrival)
    print(f"  Arrive at t={t_val:4.1f}s: in_mission={in_mission}, in_window={in_window}")
    timing.freeState(arrival)

# Plan from "start time" to "target arrival time"
start_t = timing.allocState(); start_t.position = 0.0
target_t = timing.allocState(); target_t.position = 10.0

print(f"\n  Time from mission start to checkpoint: {timing.distance(start_t, target_t):.1f}s")

wp = timing.allocState()
print(f"  Time waypoints (start=0s → arrive at 10s):")
for step_t in [0.0, 0.25, 0.5, 0.75, 1.0]:
    timing.interpolate(start_t, target_t, step_t, wp)
    print(f"    fraction={step_t}: t={wp.position:.1f}s")
    check(timing.satisfiesBounds(wp), f"time waypoint at fraction={step_t} in mission bounds")

timing.freeState(earliest); timing.freeState(latest)
timing.freeState(start_t); timing.freeState(target_t); timing.freeState(wp)


print("\n" + "=" * 60)
print("ALL SECTIONS COMPLETE")
print("=" * 60)
print("""
What we covered:

  TimeState:
    .position (read/write), repr

  TimeStateSampler:
    sampleUniform, sampleUniformNear, sampleGaussian

  TimeStateSpace (UNBOUNDED mode):
    isBounded=False, satisfiesBounds always True,
    enforceBounds is no-op, sampleUniform always gives 0

  TimeStateSpace (BOUNDED mode after setBounds):
    setBounds(minTime, maxTime), isBounded=True
    getMinTimeBound, getMaxTimeBound
    satisfiesBounds checks [min,max]
    enforceBounds clamps to [min,max]
    sampleUniform samples uniformly from [min,max]

  Both modes:
    getDimension (=1), getMaximumExtent, getMeasure
    copyState, cloneState, equalStates
    copyToReals ([position]), copyFromReals
    getSerializationLength (=8)
    distance = |t1 - t2|
    interpolate = linear
    allocState, freeState
    printState, printSettings, setup

Key concepts:
  - Two modes: UNBOUNDED (default) and BOUNDED
  - Unbounded has special no-op/always-true/zero-sample behaviour
  - Distance is simply |t1 - t2|
  - Use with CompoundStateSpace for spatio-temporal planning

All state spaces done! Next: SpaceInformation (examples/07_space_information.py)
""")