"""
===============================================================
pyompl Example 02 — SO2StateSpace
===============================================================

WHAT IS SO(2)?
--------------
SO(2) = Special Orthogonal Group in 2 dimensions.
In plain English: the set of all possible 2D rotations.

Geometrically, SO(2) is a CIRCLE. A state is just ONE angle
in radians. The valid range is (-π, π].

KEY INSIGHT — SO(2) vs RealVectorStateSpace(1):
------------------------------------------------
You might think: "an angle is just a number, why not use R^1?"

The difference is TOPOLOGY:
  In R^1:   angle 3.0 and angle -3.0 are far apart (dist = 6.0)
  In SO(2): angle 3.0 and angle -3.0 are CLOSE    (dist ≈ 0.28)
            because going the short way around the circle is shorter.

SO(2) handles angle wrapping automatically. Use it for:
  - Robot heading / yaw
  - A joint that can rotate freely (no mechanical stops)
  - Any periodic angle

Use RealVectorStateSpace(1) for joints WITH hard stops (e.g. [-π, π]).

C++ HEADER: ompl/base/spaces/SO2StateSpace.h
C++ CLASS:  ompl::base::SO2StateSpace

RUN THIS FILE:
    python examples/02_so2_state_space.py
===============================================================
"""

import math
import pyompl

PI = math.pi

def check(condition, msg):
    print(f"  {'PASS' if condition else 'FAIL'} — {msg}")

def approx_equal(a, b, tol=1e-9):
    return abs(a - b) < tol

print("=" * 60)
print("SECTION 1: SO2StateSpace — Construction")
print("=" * 60)
print("""
WHAT HAPPENS AT CONSTRUCTION?
------------------------------
SO2StateSpace sets its name to "SO2" + auto suffix, and sets
its type_ to STATE_SPACE_SO2. No bounds are needed because
the valid range (-π, π] is fixed by the circular topology.

C++: auto space = std::make_shared<SO2StateSpace>();
""")

# ---------------------------------------------------------------
# Constructor — no arguments needed
# C++: auto space = std::make_shared<SO2StateSpace>();
# ---------------------------------------------------------------
space = pyompl.SO2StateSpace()
space.setup()   # must call before copyToReals/copyFromReals work
print(f"  SO2StateSpace() created")
print(f"  getName()        = {space.getName()}")
print(f"  getDimension()   = {space.getDimension()}")
print(f"  getMaximumExtent = {space.getMaximumExtent():.6f}  (should be π ≈ {PI:.6f})")
print(f"  getMeasure()     = {space.getMeasure():.6f}  (should be 2π ≈ {2*PI:.6f})")

check(space.getDimension() == 1, "getDimension() == 1")
check(approx_equal(space.getMaximumExtent(), PI), "getMaximumExtent() == π")
check(approx_equal(space.getMeasure(), 2 * PI),   "getMeasure() == 2π")

# ---------------------------------------------------------------
# setName / getName
# C++: space->setName("MyRotation");  space->getName();
# ---------------------------------------------------------------
print("\n-- setName / getName --")
space.setName("RobotHeading")
print(f"  After setName('RobotHeading'): getName() = {space.getName()}")
check("RobotHeading" in space.getName(), "getName() contains 'RobotHeading'")

# ---------------------------------------------------------------
# getDimension()
# C++: space->getDimension()
# SO(2) is always 1-dimensional (it is a 1D manifold = a circle).
# Even though a rotation in 2D can be represented as a 2×2 matrix,
# it has only ONE degree of freedom.
# ---------------------------------------------------------------
print("\n-- getDimension() --")
print(f"  getDimension() = {space.getDimension()}")
print("  SO(2) is a 1D manifold — one angle describes any 2D rotation.")
check(space.getDimension() == 1, "SO(2) dimension is 1")

# ---------------------------------------------------------------
# getMaximumExtent()
# C++: space->getMaximumExtent()
# Maximum arc distance between any two SO(2) states = π.
# Two angles at exactly opposite sides of the circle are π apart.
# ---------------------------------------------------------------
print("\n-- getMaximumExtent() --")
ext = space.getMaximumExtent()
print(f"  getMaximumExtent() = {ext:.6f}  (π = {PI:.6f})")
print("  The max distance between any two SO(2) states is π.")
print("  This occurs when the two angles are exactly opposite (e.g. 0 and π).")
check(approx_equal(ext, PI), "max extent == π")

# ---------------------------------------------------------------
# getMeasure()
# C++: space->getMeasure()
# The 'volume' (length) of the SO(2) manifold = full circle = 2π.
# ---------------------------------------------------------------
print("\n-- getMeasure() --")
meas = space.getMeasure()
print(f"  getMeasure() = {meas:.6f}  (2π = {2*PI:.6f})")
print("  The measure of SO(2) is 2π — the circumference of the unit circle.")
check(approx_equal(meas, 2 * PI), "measure == 2π")

# ---------------------------------------------------------------
# printSettings()
# C++: space->printSettings(std::cout)
# ---------------------------------------------------------------
print("\n-- printSettings() --")
settings = space.printSettings()
print(f"  printSettings():\n{settings}")
check(len(settings) > 0, "printSettings() returns non-empty string")


print("\n" + "=" * 60)
print("SECTION 2: SO2State — The State Object")
print("=" * 60)
print("""
WHAT IS SO2State?
-----------------
The state for SO(2) is the simplest possible: one angle in radians.

C++ type: ompl::base::SO2StateSpace::StateType
C++ field: double value;   ← the angle in (-π, π]

In Python:
  s = space.allocState()     # allocate
  s.value = 1.57             # set (90°)
  angle = s.value            # read
  space.freeState(s)         # free when done
""")

# ---------------------------------------------------------------
# allocState()
# C++: space->allocState()->as<SO2StateSpace::StateType>()
# Allocates a new state on the heap. Memory managed by the space.
# The state is uninitialised — always set .value before using it.
# ---------------------------------------------------------------
print("-- allocState() --")
s1 = space.allocState()
s2 = space.allocState()
print(f"  Allocated s1 and s2 (uninitialised)")
check(s1 is not None, "allocState() returns non-None")

# ---------------------------------------------------------------
# .value property — read and write the angle
# C++: state->value = 1.57;   double a = state->value;
# ---------------------------------------------------------------
print("\n-- state.value — read and write the angle --")
s1.value = 1.57    # ≈ 90 degrees
print(f"  s1.value = 1.57")
print(f"  s1.value read back: {s1.value}")
check(approx_equal(s1.value, 1.57), ".value read back correctly")

s1.value = -PI / 2  # -90 degrees
print(f"  s1.value = -π/2 = {s1.value:.6f}")
check(approx_equal(s1.value, -PI / 2), ".value = -π/2 set correctly")

# ---------------------------------------------------------------
# setIdentity()
# C++: state->setIdentity();
# Sets state.value = 0.0 — the "no rotation" identity.
# ---------------------------------------------------------------
print("\n-- setIdentity() — set angle to zero (no rotation) --")
s1.value = 2.5
print(f"  Before setIdentity(): s1.value = {s1.value}")
s1.setIdentity()
print(f"  After  setIdentity(): s1.value = {s1.value}")
check(s1.value == 0.0, "setIdentity() sets value to 0.0")

# ---------------------------------------------------------------
# __repr__
# ---------------------------------------------------------------
print("\n-- repr --")
s1.value = 1.0
print(f"  repr(s1) = {repr(s1)}")
check("1.0" in repr(s1) or "1." in repr(s1), "repr contains the value")

space.freeState(s1)
space.freeState(s2)


print("\n" + "=" * 60)
print("SECTION 3: enforceBounds and satisfiesBounds")
print("=" * 60)
print("""
WHAT IS ANGLE WRAPPING?
------------------------
SO(2) is periodic. Any angle outside (-π, π] has an equivalent
angle inside that range (just add or subtract multiples of 2π).

enforceBounds(state)
    Wraps state.value into (-π, π]. Modifies in place.
    C++: space->enforceBounds(state);

satisfiesBounds(state)
    Returns True if state.value is in (-π, π].
    C++: space->satisfiesBounds(state)

Note: the interval is OPEN at -π and CLOSED at π.
  -π is NOT valid. π IS valid. This is the convention (-π, π].
""")

# ---------------------------------------------------------------
# satisfiesBounds — check if angle is in valid range
# ---------------------------------------------------------------
print("-- satisfiesBounds() --")

s = space.allocState()

# Valid: angle inside (-π, π]
s.value = 1.0
print(f"  value= 1.0:   satisfiesBounds = {space.satisfiesBounds(s)}")
check(space.satisfiesBounds(s) is True, "1.0 is in (-π,π]")

s.value = PI    # π is valid (closed on right)
print(f"  value= π:     satisfiesBounds = {space.satisfiesBounds(s)}")
check(space.satisfiesBounds(s) is True, "π is valid")

s.value = 0.0
print(f"  value= 0.0:   satisfiesBounds = {space.satisfiesBounds(s)}")
check(space.satisfiesBounds(s) is True, "0.0 is valid")

# Invalid: angle outside (-π, π]
s.value = 4.0   # > π
print(f"  value= 4.0:   satisfiesBounds = {space.satisfiesBounds(s)}")
check(space.satisfiesBounds(s) is False, "4.0 > π, not valid")

s.value = -4.0  # < -π
print(f"  value=-4.0:   satisfiesBounds = {space.satisfiesBounds(s)}")
check(space.satisfiesBounds(s) is False, "-4.0 < -π, not valid")

s.value = -PI   # -π is NOT valid (open on left)
print(f"  value=-π:     satisfiesBounds = {space.satisfiesBounds(s)}")
check(space.satisfiesBounds(s) is False, "-π is NOT valid (open interval)")

# ---------------------------------------------------------------
# enforceBounds — wrap angle into valid range
# ---------------------------------------------------------------
print("\n-- enforceBounds() — wrap angle into (-π, π] --")

# 4.0 → 4.0 - 2π ≈ -2.283
s.value = 4.0
space.enforceBounds(s)
print(f"  4.0   → {s.value:.6f}  (expected {4.0 - 2*PI:.6f})")
check(space.satisfiesBounds(s), "4.0 wrapped to valid range")
check(approx_equal(s.value, 4.0 - 2*PI), "4.0 wrapped correctly")

# -4.0 → -4.0 + 2π ≈ 2.283
s.value = -4.0
space.enforceBounds(s)
print(f"  -4.0  → {s.value:.6f}  (expected {-4.0 + 2*PI:.6f})")
check(space.satisfiesBounds(s), "-4.0 wrapped to valid range")
check(approx_equal(s.value, -4.0 + 2*PI), "-4.0 wrapped correctly")

# 7.0 → 7.0 - 2π ≈ 0.717
s.value = 7.0
space.enforceBounds(s)
print(f"  7.0   → {s.value:.6f}  (expected {7.0 - 2*PI:.6f})")
check(space.satisfiesBounds(s), "7.0 wrapped to valid range")

# Already valid — unchanged
s.value = 1.5
space.enforceBounds(s)
print(f"  1.5   → {s.value:.6f}  (unchanged)")
check(approx_equal(s.value, 1.5), "1.5 unchanged by enforceBounds")

space.freeState(s)


print("\n" + "=" * 60)
print("SECTION 4: copyState, cloneState, copyToReals, copyFromReals")
print("=" * 60)
print("""
copyState(destination, source)
    Copies the angle from source to destination.
    C++: space->copyState(destination, source);

cloneState(source)
    Allocates a new state and copies source into it.
    C++: space->cloneState(src)->as<SO2StateSpace::StateType>()

copyToReals(source)
    Returns the angle as a 1-element Python list.
    C++: space->copyToReals(reals, source);

copyFromReals(destination, reals)
    Sets the angle from a 1-element Python list.
    C++: space->copyFromReals(destination, reals);
""")

src = space.allocState()
dst = space.allocState()
src.value = 1.23

# copyState
print("-- copyState(destination, source) --")
space.copyState(dst, src)
print(f"  src.value={src.value}, after copyState: dst.value={dst.value}")
check(approx_equal(dst.value, src.value), "copyState copies angle correctly")

# cloneState
print("\n-- cloneState(source) --")
clone = space.cloneState(src)
print(f"  src.value={src.value}, clone.value={clone.value}")
check(approx_equal(clone.value, src.value), "cloneState copies angle correctly")
space.freeState(clone)

# copyToReals
print("\n-- copyToReals(source) — state → list --")
reals = space.copyToReals(src)
print(f"  copyToReals(src) = {reals}")
check(len(reals) == 1, "copyToReals returns 1-element list")
check(approx_equal(reals[0], src.value), "copyToReals value is correct")

# copyFromReals
print("\n-- copyFromReals(destination, reals) — list → state --")
dst2 = space.allocState()
space.copyFromReals(dst2, [2.5])
print(f"  copyFromReals([2.5]): dst2.value = {dst2.value}")
check(approx_equal(dst2.value, 2.5), "copyFromReals sets angle correctly")
space.freeState(dst2)

space.freeState(src)
space.freeState(dst)

# getSerializationLength
print("\n-- getSerializationLength() --")
slen = space.getSerializationLength()
print(f"  getSerializationLength() = {slen} bytes  (sizeof(double) = 8)")
check(slen == 8, "serialization length == 8 bytes")


print("\n" + "=" * 60)
print("SECTION 5: distance — The KEY difference from R^1")
print("=" * 60)
print("""
distance(state1, state2)
    Returns the SHORTEST arc distance between two angles.
    C++: space->distance(state1, state2)

    Formula: min(|a - b|, 2π - |a - b|)
    Always in [0, π].

This is what makes SO(2) different from a line segment.
On a circle, there are TWO ways to get from A to B:
  clockwise and counter-clockwise.
distance() always returns the SHORTER one.
""")

a = space.allocState()
b = space.allocState()

print("-- distance() — shortest arc on the circle --")

# Case 1: same angle → distance = 0
a.value, b.value = 1.0, 1.0
d = space.distance(a, b)
print(f"\n  d(1.0, 1.0)   = {d:.6f}   (same angle → 0)")
check(approx_equal(d, 0.0), "same angle has distance 0")

# Case 2: simple case, no wrapping needed
a.value, b.value = 0.0, 1.0
d = space.distance(a, b)
print(f"  d(0.0, 1.0)   = {d:.6f}   (direct arc = 1.0)")
check(approx_equal(d, 1.0), "simple distance = 1.0")

# Case 3: THE KEY CASE — angles near ±π are actually close
# In R^1: |3.0 - (-3.0)| = 6.0 (far apart)
# In SO(2): going through π is only ≈ 0.28 radians
a.value, b.value = 3.0, -3.0
d = space.distance(a, b)
expected = min(abs(3.0 - (-3.0)), 2*PI - abs(3.0 - (-3.0)))
print(f"\n  d(3.0, -3.0)  = {d:.6f}   (short arc ≈ {expected:.6f}, NOT 6.0)")
print(f"  In R^1 this would be 6.0. In SO(2) it is {d:.4f}.")
print(f"  The short path goes through ±π (the 'back' of the circle).")
check(approx_equal(d, expected), "wrapping distance correct")
check(d < 1.0, "wrapped distance < 1.0 (much less than 6.0)")

# Case 4: exactly opposite → distance = π (maximum)
a.value, b.value = 0.0, PI
d = space.distance(a, b)
print(f"\n  d(0.0, π)     = {d:.6f}   (opposite sides → π)")
check(approx_equal(d, PI), "opposite angles have distance π")

# Case 5: π and -π are the SAME point (both mean 180°)
a.value, b.value = PI, -PI + 1e-15  # as close to -π as possible
space.enforceBounds(a)
space.enforceBounds(b)
d = space.distance(a, b)
print(f"\n  d(π, ≈-π)     = {d:.10f}  (same point, distance ≈ 0)")
check(d < 1e-6, "π and -π are the same point")

# Case 6: symmetry
a.value, b.value = 1.0, 2.5
d_ab = space.distance(a, b)
d_ba = space.distance(b, a)
print(f"\n  d(1.0, 2.5) = {d_ab:.6f},  d(2.5, 1.0) = {d_ba:.6f}")
check(approx_equal(d_ab, d_ba), "distance is symmetric")

space.freeState(a)
space.freeState(b)


print("\n" + "=" * 60)
print("SECTION 6: equalStates")
print("=" * 60)
print("""
equalStates(state1, state2)
    Returns True if both states have EXACTLY the same .value.
    C++: space->equalStates(state1, state2)

    Exact floating-point comparison.
    Note: π and -π have distance 0 but are NOT equalStates.
    Use enforceBounds() first if needed.
""")

e1 = space.allocState()
e2 = space.allocState()
e3 = space.allocState()

e1.value = 1.5
e2.value = 1.5
e3.value = 2.0

print(f"  equalStates(1.5, 1.5) = {space.equalStates(e1, e2)}")
print(f"  equalStates(1.5, 2.0) = {space.equalStates(e1, e3)}")
check(space.equalStates(e1, e2) is True,  "identical states are equal")
check(space.equalStates(e1, e3) is False, "different states are not equal")

# Demonstrate the π/-π edge case
e1.value =  PI
e2.value = -PI
print(f"\n  equalStates(π, -π) = {space.equalStates(e1, e2)}")
print(f"  distance(π, -π)    = {space.distance(e1, e2):.10f}")
print("  They represent the same rotation but have different .value !")
print("  Always enforceBounds() both states first for robust comparison.")
check(space.equalStates(e1, e2) is False, "π and -π are not equalStates")

space.freeState(e1)
space.freeState(e2)
space.freeState(e3)


print("\n" + "=" * 60)
print("SECTION 7: interpolate — shortest arc path")
print("=" * 60)
print("""
interpolate(from, to, t, result)
    Computes the state at fraction t along the SHORTEST arc
    from 'from' to 'to'. t in [0, 1].

    C++: space->interpolate(from, to, t, result);

    t=0.0 → result == from
    t=1.0 → result == to
    t=0.5 → halfway along shortest arc

    This is NOT simple (from.value + t*(to.value - from.value)).
    It always goes the SHORT WAY around the circle.
    'result' must be pre-allocated.
""")

f = space.allocState()
t = space.allocState()
r = space.allocState()

print("-- Simple case: no wrapping needed --")
f.value, t.value = 0.0, 2.0
space.interpolate(f, t, 0.0, r)
print(f"  from=0.0, to=2.0, t=0.0 → result={r.value:.6f}  (should be 0.0)")
check(approx_equal(r.value, 0.0), "t=0 gives from")

space.interpolate(f, t, 1.0, r)
print(f"  from=0.0, to=2.0, t=1.0 → result={r.value:.6f}  (should be 2.0)")
check(approx_equal(r.value, 2.0), "t=1 gives to")

space.interpolate(f, t, 0.5, r)
print(f"  from=0.0, to=2.0, t=0.5 → result={r.value:.6f}  (should be 1.0)")
check(approx_equal(r.value, 1.0), "t=0.5 gives midpoint")

print("\n-- Wrapping case: short arc goes through ±π --")
# from=3.0, to=-3.0: the short arc goes backwards through π
# Linear interpolation of raw values would give (3+(-3))/2 = 0.0 (WRONG)
# SO(2) interpolation gives the midpoint of the short arc (near ±π)
f.value, t.value = 3.0, -3.0
space.interpolate(f, t, 0.5, r)
print(f"  from=3.0, to=-3.0, t=0.5 → result={r.value:.6f}")
print(f"  (The short arc goes through ±π, so midpoint is near ±{PI:.4f})")
# The midpoint should be near π or -π
check(abs(abs(r.value) - PI) < 0.5, "midpoint is near ±π for wrapping case")

print("\n-- Quarter steps --")
f.value, t.value = 0.0, PI
for step_t in [0.0, 0.25, 0.5, 0.75, 1.0]:
    space.interpolate(f, t, step_t, r)
    print(f"  t={step_t}: result={r.value:.6f}  "
          f"(expected {step_t * PI:.6f})")
    check(approx_equal(r.value, step_t * PI, tol=1e-6),
          f"t={step_t} interpolation correct")

space.freeState(f)
space.freeState(t)
space.freeState(r)


print("\n" + "=" * 60)
print("SECTION 8: Sampler — generating random SO(2) states")
print("=" * 60)
print("""
allocDefaultStateSampler() / allocStateSampler()
    Returns a SO2StateSampler for this space.
    C++: space->allocDefaultStateSampler()

sampleUniform(state)
    Samples a uniformly random angle from (-π, π].
    C++: sampler->sampleUniform(state);

sampleUniformNear(state, near, distance)
    Samples uniformly from an arc of radius 'distance' around 'near'.
    Result is wrapped to (-π, π].
    C++: sampler->sampleUniformNear(state, near, distance);

sampleGaussian(state, mean, stdDev)
    Samples from a wrapped Gaussian centred at 'mean'.
    Result is wrapped to (-π, π].
    C++: sampler->sampleGaussian(state, mean, stdDev);
""")

sampler = space.allocDefaultStateSampler()
print(f"  allocDefaultStateSampler() → {type(sampler).__name__}")
check(sampler is not None, "sampler is not None")

ss = space.allocState()

# sampleUniform — run multiple times, all should be in valid range
print("\n-- sampleUniform() — uniform random angle from (-π, π] --")
all_valid = True
for i in range(10):
    sampler.sampleUniform(ss)
    if not space.satisfiesBounds(ss):
        all_valid = False
        print(f"  FAIL: sample {i} = {ss.value:.4f} out of bounds!")
    if i < 3:
        print(f"  sample {i}: {ss.value:.6f} rad = {math.degrees(ss.value):.2f}°")
check(all_valid, "all 10 uniform samples are within bounds")

# sampleUniformNear
print("\n-- sampleUniformNear(near=0.0, distance=0.5) --")
near = space.allocState()
near.value = 0.0
all_valid = True
for i in range(10):
    sampler.sampleUniformNear(ss, near, 0.5)
    if not space.satisfiesBounds(ss):
        all_valid = False
    if i < 3:
        print(f"  sample {i}: {ss.value:.6f} rad  "
              f"(distance from 0.0: {space.distance(ss, near):.4f})")
check(all_valid, "all near-uniform samples are within bounds")

# Test wrapping: near=π, distance=0.5 — samples may wrap around
near.value = PI
print(f"\n-- sampleUniformNear(near=π={PI:.4f}, distance=0.5) — wraps around --")
for i in range(3):
    sampler.sampleUniformNear(ss, near, 0.5)
    print(f"  sample {i}: {ss.value:.6f} rad  "
          f"(dist from π: {space.distance(ss, near):.4f})")
    check(space.satisfiesBounds(ss), f"near-π sample {i} in bounds")

# sampleGaussian
print("\n-- sampleGaussian(mean=1.0, stdDev=0.3) --")
mean_s = space.allocState()
mean_s.value = 1.0
for i in range(3):
    sampler.sampleGaussian(ss, mean_s, 0.3)
    print(f"  sample {i}: {ss.value:.6f} rad  "
          f"(dist from mean: {space.distance(ss, mean_s):.4f})")
    check(space.satisfiesBounds(ss), f"Gaussian sample {i} in bounds")

space.freeState(ss)
space.freeState(near)
space.freeState(mean_s)


print("\n" + "=" * 60)
print("SECTION 9: printState, printSettings, setup, registerProjections")
print("=" * 60)

# printState
print("-- printState(state) --")
ps = space.allocState()
ps.value = 1.2345
state_str = space.printState(ps)
print(f"  printState(1.2345) = '{state_str.strip()}'")
check(len(state_str) > 0, "printState returns non-empty string")
space.freeState(ps)

# printSettings
print("\n-- printSettings() --")
cfg = space.printSettings()
print(f"  printSettings():\n{cfg}")
check(len(cfg) > 0, "printSettings returns non-empty string")

# setup
print("\n-- setup() --")
fresh = pyompl.SO2StateSpace()
fresh.setup()
print("  setup() completed without error")
check(True, "setup() runs cleanly")

# registerProjections
print("\n-- registerProjections() --")
fresh.registerProjections()
print("  registerProjections() completed without error")
check(True, "registerProjections() runs cleanly")


print("\n" + "=" * 60)
print("SECTION 10: Real-world usage — robot heading")
print("=" * 60)
print("""
Use case: a ground robot that can rotate freely (no heading limits).
The heading is an SO(2) state. We plan paths that respect angle wrapping.
""")

heading_space = pyompl.SO2StateSpace()
heading_space.setName("RobotHeading")
heading_space.setup()

# Current heading: almost due East (0 rad), facing slightly North
current = heading_space.allocState()
current.value = 0.1   # slightly above 0 (≈ 6°)

# Target heading: almost due West (π rad), slightly below
target = heading_space.allocState()
target.value = PI - 0.1  # ≈ 3.04 rad (≈ 174°)

# The short arc from 0.1 to 2.94 goes directly clockwise
dist_heading = heading_space.distance(current, target)
print(f"  Current heading: {math.degrees(current.value):.1f}°")
print(f"  Target  heading: {math.degrees(target.value):.1f}°")
print(f"  Distance (shortest arc): {dist_heading:.4f} rad = {math.degrees(dist_heading):.1f}°")

# Plan waypoints at t=0.25, 0.5, 0.75
waypoint = heading_space.allocState()
print(f"\n  Waypoints along shortest arc:")
for step_t in [0.0, 0.25, 0.5, 0.75, 1.0]:
    heading_space.interpolate(current, target, step_t, waypoint)
    print(f"    t={step_t}: {math.degrees(waypoint.value):.1f}°")
    check(heading_space.satisfiesBounds(waypoint),
          f"waypoint at t={step_t} is valid SO(2) state")

# Now try a case where wrapping matters:
# Robot at heading ≈ 175° needs to reach heading ≈ -175°
# Short path goes THROUGH 180° (±π), NOT all the way around 350°
current.value = PI - 0.1     # ≈ +175°
target.value  = -(PI - 0.1)  # ≈ -175°
# Note: -175° wraps to ≈ -3.04 which is valid
heading_space.enforceBounds(target)

dist2 = heading_space.distance(current, target)
print(f"\n  Wrapping case:")
print(f"  From {math.degrees(current.value):.1f}° to {math.degrees(target.value):.1f}°")
print(f"  Distance = {dist2:.4f} rad = {math.degrees(dist2):.1f}°")
print(f"  (Short path goes through ±180°, NOT the 350° long way around)")
check(dist2 < 1.0, "short arc through ±π is less than 1 radian")

heading_space.freeState(current)
heading_space.freeState(target)
heading_space.freeState(waypoint)


print("\n" + "=" * 60)
print("ALL SECTIONS COMPLETE")
print("=" * 60)
print("""
What we covered:

  SO2State:
    .value (get/set), setIdentity, repr

  SO2StateSpace:
    constructor, setName, getName
    getDimension, getMaximumExtent, getMeasure
    satisfiesBounds, enforceBounds
    copyState, cloneState, copyToReals, copyFromReals
    getSerializationLength
    distance, equalStates, interpolate
    allocState, freeState
    allocDefaultStateSampler, allocStateSampler
    sampleUniform, sampleUniformNear, sampleGaussian
    printState, printSettings
    setup, registerProjections

Key concepts learned:
  - SO(2) is a CIRCLE, not a line
  - distance() uses the SHORTEST arc, not |a - b|
  - enforceBounds() wraps angles — essential for planning
  - interpolate() goes the short way around, not linearly
  - π and -π represent the same rotation

Next: examples/03_se2_state_space.py (position + heading = R^2 × SO(2))
""")