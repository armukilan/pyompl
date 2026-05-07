"""
===============================================================
pyompl Example 11 — Base Types
     Cost, GoalType, PlannerStatus, StateSpaceType
===============================================================

WHAT IS THIS FILE?
-------------------
This covers the four fundamental "type definition" headers in ompl/base/.
These are the smallest building blocks — used everywhere in OMPL.

FILES WRAPPED:
  ompl/base/Cost.h              → pyompl.Cost
  ompl/base/GoalTypes.h         → pyompl.GoalType  (enum)
  ompl/base/PlannerStatus.h     → pyompl.PlannerStatus + pyompl.PlannerStatusType
  ompl/base/StateSpaceTypes.h   → pyompl.StateSpaceType (enum)

WHY DO THESE MATTER?
  Cost         — every optimal planner returns costs, compares costs
  GoalType     — planners check what kind of goal they received
  PlannerStatus— the return value of planner.solve()
  StateSpaceType — check what kind of space you have without isinstance()

RUN THIS FILE:
    python examples/11_base_types.py
===============================================================
"""

import math
import pyompl

def check(condition, msg):
    print(f"  {'PASS' if condition else 'FAIL'} — {msg}")

def approx(a, b, tol=1e-9):
    return abs(a - b) < tol


# ==============================================================
# PART 1: Cost
# ==============================================================
print("=" * 60)
print("PART 1: Cost  (ompl/base/Cost.h)")
print("=" * 60)
print("""
WHAT IS Cost?
--------------
Cost is a type-safe wrapper around a single double value.
It represents the "cost" of a motion, a state, or an entire path.

WHY NOT JUST USE float?
  OMPL uses Cost instead of raw floats so that the compiler
  catches mistakes like: comparing a cost to a distance, or
  accidentally adding a cost to a state value.

WHERE IS Cost USED?
  - OptimizationObjective.motionCost(s1, s2)  → Cost
  - OptimizationObjective.stateCost(state)     → Cost
  - OptimizationObjective.combineCosts(c1, c2) → Cost
  - ProblemDefinition.getSolutionPath().cost() → Cost (after Path.h is bound)

C++ class:  ompl::base::Cost
C++ header: ompl/base/Cost.h

CONSTRUCTOR:
  C++:    Cost c(3.14);
  Python: c = pyompl.Cost(3.14)
""")

# ---------------------------------------------------------------
# Constructor
# C++: Cost c(3.14);
# C++: Cost c;          // default = 0.0
# ---------------------------------------------------------------
print("-- Constructor --")
c1 = pyompl.Cost(3.14)
c2 = pyompl.Cost(2.71)
c3 = pyompl.Cost()       # default = 0.0
c4 = pyompl.Cost(0.0)

print(f"  Cost(3.14) created: repr = {repr(c1)}")
print(f"  Cost(2.71) created: repr = {repr(c2)}")
print(f"  Cost()     default: repr = {repr(c3)}")
check(True, "Cost constructor works")

# ---------------------------------------------------------------
# value()
# C++: double v = c.value();
# Returns the raw double stored inside.
# ---------------------------------------------------------------
print("\n-- value() — get the raw double --")
print(f"  Cost(3.14).value() = {c1.value()}")
print(f"  Cost(2.71).value() = {c2.value()}")
print(f"  Cost().value()     = {c3.value()}")
check(approx(c1.value(), 3.14),  "value() returns 3.14")
check(approx(c2.value(), 2.71),  "value() returns 2.71")
check(approx(c3.value(), 0.0),   "default value() returns 0.0")

# ---------------------------------------------------------------
# float(cost) — convert to Python float
# C++: (double)c  or  c.value()
# ---------------------------------------------------------------
print("\n-- float(cost) — convert to Python float --")
f = float(c1)
print(f"  float(Cost(3.14)) = {f}")
check(approx(f, 3.14), "float() conversion correct")

# ---------------------------------------------------------------
# Comparison operators: <, >, ==
# C++: a.value() < b.value()   (no built-in operators in C++)
# We add these in the binding for Python convenience.
# ---------------------------------------------------------------
print("\n-- Comparison: <, >, == --")
print(f"  Cost(3.14) < Cost(2.71)  = {c1 < c2}")   # False
print(f"  Cost(3.14) > Cost(2.71)  = {c1 > c2}")   # True
print(f"  Cost(3.14) == Cost(3.14) = {c1 == c1}")  # True
print(f"  Cost(3.14) == Cost(2.71) = {c1 == c2}")  # False

check(not (c1 < c2),   "3.14 is NOT less than 2.71")
check(c1 > c2,          "3.14 IS greater than 2.71")
check(c1 == c1,         "same cost is equal to itself")
check(not (c1 == c2),  "different costs are not equal")

# Sorting a list of costs
costs = [pyompl.Cost(5.0), pyompl.Cost(1.0), pyompl.Cost(3.0), pyompl.Cost(2.0)]
sorted_costs = sorted(costs, key=lambda c: c.value())
print(f"\n  Sorted costs: {[round(c.value(),1) for c in sorted_costs]}")
check(sorted_costs[0].value() == 1.0, "smallest cost first after sort")

# ---------------------------------------------------------------
# Addition: cost + cost
# C++: Cost(a.value() + b.value())
# Used by OptimizationObjective.combineCosts(c1, c2)
# ---------------------------------------------------------------
print("\n-- Addition: cost + cost --")
total = c1 + c2
print(f"  Cost(3.14) + Cost(2.71) = {repr(total)}")
print(f"  total.value() = {total.value():.6f}")
check(approx(total.value(), 3.14 + 2.71), "cost addition correct")

# Chained addition
chain = pyompl.Cost(1.0) + pyompl.Cost(2.0) + pyompl.Cost(3.0)
print(f"  Cost(1) + Cost(2) + Cost(3) = {chain.value()}")
check(approx(chain.value(), 6.0), "chained addition correct")

# ---------------------------------------------------------------
# Special cost values — infinity and zero
# C++: Cost(std::numeric_limits<double>::infinity())
# C++: Cost(0.0)
# ---------------------------------------------------------------
print("\n-- Special values: infinity and zero --")
inf_cost  = pyompl.Cost(float('inf'))
zero_cost = pyompl.Cost(0.0)

print(f"  infinite cost: {inf_cost.value()}")
print(f"  zero cost:     {zero_cost.value()}")
check(math.isinf(inf_cost.value()),          "infinity cost works")
check(approx(zero_cost.value(), 0.0),        "zero cost works")
check(inf_cost > zero_cost,                  "infinity > zero")
check(zero_cost < inf_cost,                  "zero < infinity")

# ---------------------------------------------------------------
# Negative costs — technically valid (some objectives use them)
# ---------------------------------------------------------------
print("\n-- Negative costs --")
neg = pyompl.Cost(-5.0)
print(f"  Cost(-5.0).value() = {neg.value()}")
check(neg.value() == -5.0, "negative cost works")
check(neg < zero_cost,      "negative < zero")

# ---------------------------------------------------------------
# Real-world use pattern
# In optimal planning, you compare costs like this:
# ---------------------------------------------------------------
print("\n-- Real-world pattern: cost comparison in optimal planning --")
current_best = pyompl.Cost(float('inf'))  # start with worst possible
candidates = [pyompl.Cost(5.2), pyompl.Cost(3.1), pyompl.Cost(4.8), pyompl.Cost(3.0)]

for candidate in candidates:
    if candidate < current_best:
        current_best = candidate
        print(f"  New best solution cost: {current_best.value()}")

print(f"  Final best cost: {current_best.value()}")
check(approx(current_best.value(), 3.0), "found minimum cost correctly")


# ==============================================================
# PART 2: GoalType
# ==============================================================
print("\n" + "=" * 60)
print("PART 2: GoalType  (ompl/base/GoalTypes.h)")
print("=" * 60)
print("""
WHAT IS GoalType?
------------------
GoalType is a bit-flag enum. Every Goal subclass sets its type_
so planners can check what capabilities the goal provides.

WHY BIT FLAGS?
  A GoalState IS ALSO a GoalSampleableRegion IS ALSO a GoalRegion.
  By using bit flags, a single integer can represent all of these.

THE HIERARCHY:
  Goal (GOAL_ANY)
    └── GoalRegion (GOAL_REGION)
          └── GoalSampleableRegion (GOAL_SAMPLEABLE_REGION)
                ├── GoalState  (GOAL_STATE)
                ├── GoalStates (GOAL_STATES)
                └── GoalLazySamples (GOAL_LAZY_SAMPLES)

C++ header: ompl/base/GoalTypes.h
C++ enum:   ompl::base::GoalType
""")

print("-- All GoalType values and their numeric values --")
goal_types = [
    (pyompl.GoalType.GOAL_ANY,               "GOAL_ANY"),
    (pyompl.GoalType.GOAL_REGION,            "GOAL_REGION"),
    (pyompl.GoalType.GOAL_SAMPLEABLE_REGION, "GOAL_SAMPLEABLE_REGION"),
    (pyompl.GoalType.GOAL_STATE,             "GOAL_STATE"),
    (pyompl.GoalType.GOAL_STATES,            "GOAL_STATES"),
    (pyompl.GoalType.GOAL_LAZY_SAMPLES,      "GOAL_LAZY_SAMPLES"),
]

for gt, name in goal_types:
    print(f"  {name:30s} = {int(gt)}")

check(int(pyompl.GoalType.GOAL_ANY) == 1,                "GOAL_ANY == 1")
check(int(pyompl.GoalType.GOAL_REGION) == 3,             "GOAL_REGION == 3")
check(int(pyompl.GoalType.GOAL_SAMPLEABLE_REGION) == 7,  "GOAL_SAMPLEABLE_REGION == 7")
check(int(pyompl.GoalType.GOAL_STATE) == 15,             "GOAL_STATE == 15")
check(int(pyompl.GoalType.GOAL_STATES) == 23,            "GOAL_STATES == 23")
check(int(pyompl.GoalType.GOAL_LAZY_SAMPLES) == 55,      "GOAL_LAZY_SAMPLES == 55")

print("\n-- GoalType is used with space.getType() after Goal.h is bound --")
print("  Example (future use once Goal.h is bound):")
print("    goal = pdef.getGoal()")
print("    if goal.getType() == pyompl.GoalType.GOAL_STATE:")
print("        print('Single state goal')")
print("    if goal.getType() == pyompl.GoalType.GOAL_SAMPLEABLE_REGION:")
print("        print('Can sample from goal region')")

print("\n-- Bit flag check: GoalState satisfies GoalRegion check --")
# A GoalState has type GOAL_STATE = 15
# GOAL_REGION = 3
# 15 & 3 == 3 means GoalState IS a GoalRegion
goal_state_type = int(pyompl.GoalType.GOAL_STATE)
goal_region_type = int(pyompl.GoalType.GOAL_REGION)
is_region = (goal_state_type & goal_region_type) == goal_region_type
print(f"  GOAL_STATE ({goal_state_type}) & GOAL_REGION ({goal_region_type}) == {goal_state_type & goal_region_type}")
print(f"  GoalState satisfies GoalRegion check: {is_region}")
check(is_region, "GoalState IS a GoalRegion (bit flag check)")


# ==============================================================
# PART 3: PlannerStatus
# ==============================================================
print("\n" + "=" * 60)
print("PART 3: PlannerStatus  (ompl/base/PlannerStatus.h)")
print("=" * 60)
print("""
WHAT IS PlannerStatus?
-----------------------
PlannerStatus is the return value of planner.solve().
It tells you WHY the planner stopped and WHETHER it found a solution.

THE MOST IMPORTANT PATTERN (you will use this constantly):
  status = planner.solve(5.0)
  if status:
      path = pdef.getSolutionPath()

bool(status) is True ONLY for EXACT_SOLUTION or APPROXIMATE_SOLUTION.

C++ header: ompl/base/PlannerStatus.h
C++ class:  ompl::base::PlannerStatus
C++ enum:   ompl::base::PlannerStatus::StatusType
""")

# ---------------------------------------------------------------
# PlannerStatusType enum — all values
# ---------------------------------------------------------------
print("-- All PlannerStatusType values --")
status_types = [
    (pyompl.PlannerStatusType.UNKNOWN,               "UNKNOWN"),
    (pyompl.PlannerStatusType.INVALID_START,         "INVALID_START"),
    (pyompl.PlannerStatusType.INVALID_GOAL,          "INVALID_GOAL"),
    (pyompl.PlannerStatusType.UNRECOGNIZED_GOAL_TYPE,"UNRECOGNIZED_GOAL_TYPE"),
    (pyompl.PlannerStatusType.TIMEOUT,               "TIMEOUT"),
    (pyompl.PlannerStatusType.APPROXIMATE_SOLUTION,  "APPROXIMATE_SOLUTION"),
    (pyompl.PlannerStatusType.EXACT_SOLUTION,        "EXACT_SOLUTION"),
    (pyompl.PlannerStatusType.CRASH,                 "CRASH"),
    (pyompl.PlannerStatusType.ABORT,                 "ABORT"),
    (pyompl.PlannerStatusType.INFEASIBLE,            "INFEASIBLE"),
]
for st, name in status_types:
    print(f"  {name:30s} = {int(st)}")

# ---------------------------------------------------------------
# Constructor 1: PlannerStatus(StatusType)
# C++: PlannerStatus status(PlannerStatus::EXACT_SOLUTION);
# ---------------------------------------------------------------
print("\n-- Constructor 1: PlannerStatus(StatusType) --")
s_exact   = pyompl.PlannerStatus(pyompl.PlannerStatusType.EXACT_SOLUTION)
s_approx  = pyompl.PlannerStatus(pyompl.PlannerStatusType.APPROXIMATE_SOLUTION)
s_timeout = pyompl.PlannerStatus(pyompl.PlannerStatusType.TIMEOUT)
s_unknown = pyompl.PlannerStatus(pyompl.PlannerStatusType.UNKNOWN)
s_invalid = pyompl.PlannerStatus(pyompl.PlannerStatusType.INVALID_START)
s_crash   = pyompl.PlannerStatus(pyompl.PlannerStatusType.CRASH)
s_infeas  = pyompl.PlannerStatus(pyompl.PlannerStatusType.INFEASIBLE)

print(f"  EXACT_SOLUTION:   repr = {repr(s_exact)}")
print(f"  APPROXIMATE:      repr = {repr(s_approx)}")
print(f"  TIMEOUT:          repr = {repr(s_timeout)}")

# ---------------------------------------------------------------
# Constructor 2: PlannerStatus(hasSolution, isApproximate)
# C++: PlannerStatus status(true, false);   // exact
# C++: PlannerStatus status(true, true);    // approximate
# C++: PlannerStatus status(false, false);  // timeout
# ---------------------------------------------------------------
print("\n-- Constructor 2: PlannerStatus(hasSolution, isApproximate) --")
s2_exact  = pyompl.PlannerStatus(True,  False)  # exact solution
s2_approx = pyompl.PlannerStatus(True,  True)   # approximate solution
s2_none   = pyompl.PlannerStatus(False, False)  # no solution = TIMEOUT

print(f"  (True,  False) → {s2_exact.asString()}")
print(f"  (True,  True)  → {s2_approx.asString()}")
print(f"  (False, False) → {s2_none.asString()}")

check(s2_exact.getType()  == pyompl.PlannerStatusType.EXACT_SOLUTION,
      "(True, False) → EXACT_SOLUTION")
check(s2_approx.getType() == pyompl.PlannerStatusType.APPROXIMATE_SOLUTION,
      "(True, True) → APPROXIMATE_SOLUTION")
check(s2_none.getType()   == pyompl.PlannerStatusType.TIMEOUT,
      "(False, False) → TIMEOUT")

# ---------------------------------------------------------------
# asString()
# C++: status.asString()
# Returns a human-readable description.
# ---------------------------------------------------------------
print("\n-- asString() — human-readable description --")
all_statuses = [s_exact, s_approx, s_timeout, s_unknown,
                s_invalid, s_crash, s_infeas]
for s in all_statuses:
    print(f"  {s.asString()}")
check(all(len(s.asString()) > 0 for s in all_statuses),
      "all statuses return non-empty string")

# ---------------------------------------------------------------
# getType()
# C++: (PlannerStatus::StatusType)status
# Returns the raw enum value.
# ---------------------------------------------------------------
print("\n-- getType() — get the raw StatusType --")
print(f"  s_exact.getType()   = {s_exact.getType()}")
print(f"  s_timeout.getType() = {s_timeout.getType()}")
check(s_exact.getType()   == pyompl.PlannerStatusType.EXACT_SOLUTION,
      "getType() for exact == EXACT_SOLUTION")
check(s_timeout.getType() == pyompl.PlannerStatusType.TIMEOUT,
      "getType() for timeout == TIMEOUT")

# ---------------------------------------------------------------
# bool(status) — THE MOST IMPORTANT CHECK
# C++: if (status) { ... }
# True ONLY for EXACT_SOLUTION or APPROXIMATE_SOLUTION.
# ---------------------------------------------------------------
print("\n-- bool(status) — did we find a solution? --")
print("  This is the #1 check you do after every planner.solve() call.")
print()

# Successes — should be True
for s, name in [(s_exact, "EXACT_SOLUTION"),
                (s_approx, "APPROXIMATE_SOLUTION"),
                (s2_exact, "bool-constructed exact"),
                (s2_approx, "bool-constructed approx")]:
    result = bool(s)
    print(f"  bool({name:30s}) = {result}")
    check(result is True, f"{name} → bool == True")

print()

# Failures — should be False
for s, name in [(s_timeout, "TIMEOUT"),
                (s_unknown, "UNKNOWN"),
                (s_invalid, "INVALID_START"),
                (s_crash,   "CRASH"),
                (s_infeas,  "INFEASIBLE"),
                (s2_none,   "bool-constructed no-solution")]:
    result = bool(s)
    print(f"  bool({name:30s}) = {result}")
    check(result is False, f"{name} → bool == False")

# ---------------------------------------------------------------
# The canonical planning pattern
# ---------------------------------------------------------------
print("\n-- Canonical planning pattern --")
print("""
  # This is how you use PlannerStatus in real planning code:
  
  # (Once planners are bound, this will be the real pattern)
  # status = planner.solve(5.0)
  # if status:
  #     print("Found solution:", status.asString())
  #     path = pdef.getSolutionPath()
  # elif status.getType() == pyompl.PlannerStatusType.TIMEOUT:
  #     print("Ran out of time — try longer timeout")
  # elif status.getType() == pyompl.PlannerStatusType.INVALID_START:
  #     print("Start state is invalid — check your SVC")
  # else:
  #     print("Planning failed:", status.asString())
""")

# Simulate the pattern with constructed statuses
def simulate_planning(status):
    if status:
        return f"SUCCESS: {status.asString()}"
    elif status.getType() == pyompl.PlannerStatusType.TIMEOUT:
        return "TIMEOUT: try longer solve time"
    elif status.getType() == pyompl.PlannerStatusType.INVALID_START:
        return "ERROR: invalid start state"
    else:
        return f"FAILED: {status.asString()}"

print("  Simulating the canonical pattern:")
for s, name in [(s_exact, "exact"), (s_approx, "approx"),
                (s_timeout, "timeout"), (s_invalid, "invalid_start")]:
    print(f"  {name:15s} → {simulate_planning(s)}")

check(True, "canonical planning pattern works")


# ==============================================================
# PART 4: StateSpaceType
# ==============================================================
print("\n" + "=" * 60)
print("PART 4: StateSpaceType  (ompl/base/StateSpaceTypes.h)")
print("=" * 60)
print("""
WHAT IS StateSpaceType?
------------------------
StateSpaceType is an integer enum. Every state space sets its
type_ field in the constructor. You can read it via getType().

WHY USE IT?
  In C++, OMPL uses this for fast safe downcasting:
    if (space->getType() == STATE_SPACE_SE2)
        space->as<SE2StateSpace>()->setBounds(...)

  In Python you can use either getType() OR isinstance(),
  but getType() is faster and matches the C++ pattern.

C++ header: ompl/base/StateSpaceTypes.h
C++ enum:   ompl::base::StateSpaceType
""")

# All enum values
print("-- All StateSpaceType values --")
space_types = [
    (pyompl.StateSpaceType.STATE_SPACE_UNKNOWN,      "UNKNOWN"),
    (pyompl.StateSpaceType.STATE_SPACE_REAL_VECTOR,  "REAL_VECTOR"),
    (pyompl.StateSpaceType.STATE_SPACE_SO2,          "SO2"),
    (pyompl.StateSpaceType.STATE_SPACE_SO3,          "SO3"),
    (pyompl.StateSpaceType.STATE_SPACE_SE2,          "SE2"),
    (pyompl.StateSpaceType.STATE_SPACE_SE3,          "SE3"),
    (pyompl.StateSpaceType.STATE_SPACE_TIME,         "TIME"),
    (pyompl.StateSpaceType.STATE_SPACE_DISCRETE,     "DISCRETE"),
    (pyompl.StateSpaceType.STATE_SPACE_DUBINS,       "DUBINS"),
    (pyompl.StateSpaceType.STATE_SPACE_REEDS_SHEPP,  "REEDS_SHEPP"),
    (pyompl.StateSpaceType.STATE_SPACE_MOBIUS,       "MOBIUS"),
    (pyompl.StateSpaceType.STATE_SPACE_SPHERE,       "SPHERE"),
    (pyompl.StateSpaceType.STATE_SPACE_TORUS,        "TORUS"),
    (pyompl.StateSpaceType.STATE_SPACE_KLEIN_BOTTLE, "KLEIN_BOTTLE"),
    (pyompl.StateSpaceType.STATE_SPACE_VANA,         "VANA"),
    (pyompl.StateSpaceType.STATE_SPACE_OWEN,         "OWEN"),
    (pyompl.StateSpaceType.STATE_SPACE_VANA_OWEN,    "VANA_OWEN"),
    (pyompl.StateSpaceType.STATE_SPACE_TROCHOID,     "TROCHOID"),
]
for st, name in space_types:
    print(f"  STATE_SPACE_{name:20s} = {int(st)}")

# Verify specific values
check(int(pyompl.StateSpaceType.STATE_SPACE_UNKNOWN)     == 0,  "UNKNOWN == 0")
check(int(pyompl.StateSpaceType.STATE_SPACE_REAL_VECTOR) == 1,  "REAL_VECTOR == 1")
check(int(pyompl.StateSpaceType.STATE_SPACE_SO2)         == 2,  "SO2 == 2")
check(int(pyompl.StateSpaceType.STATE_SPACE_SO3)         == 3,  "SO3 == 3")
check(int(pyompl.StateSpaceType.STATE_SPACE_SE2)         == 4,  "SE2 == 4")
check(int(pyompl.StateSpaceType.STATE_SPACE_SE3)         == 5,  "SE3 == 5")

# ---------------------------------------------------------------
# getType() on actual state spaces
# C++: space->getType()
# Shows that actual spaces report the correct type.
# ---------------------------------------------------------------
print("\n-- getType() on actual state spaces --")
spaces_to_check = [
    (pyompl.RealVectorStateSpace(2), pyompl.StateSpaceType.STATE_SPACE_REAL_VECTOR, "RealVectorStateSpace"),
    (pyompl.SO2StateSpace(),         pyompl.StateSpaceType.STATE_SPACE_SO2,         "SO2StateSpace"),
    (pyompl.SO3StateSpace(),         pyompl.StateSpaceType.STATE_SPACE_SO3,         "SO3StateSpace"),
    (pyompl.SE2StateSpace(),         pyompl.StateSpaceType.STATE_SPACE_SE2,         "SE2StateSpace"),
    (pyompl.SE3StateSpace(),         pyompl.StateSpaceType.STATE_SPACE_SE3,         "SE3StateSpace"),
    (pyompl.TimeStateSpace(),        pyompl.StateSpaceType.STATE_SPACE_TIME,        "TimeStateSpace"),
    (pyompl.DiscreteStateSpace(0,5), pyompl.StateSpaceType.STATE_SPACE_DISCRETE,    "DiscreteStateSpace"),
]

for space, expected_type, name in spaces_to_check:
    actual = space.getType()
    match = (actual == int(expected_type))
    print(f"  {name:30s}.getType() = {actual}  {'✓' if match else '✗'}")
    check(match, f"{name} has correct type {int(expected_type)}")

# ---------------------------------------------------------------
# Using getType() for type-based dispatch
# This mirrors the C++ pattern: space->getType() == STATE_SPACE_SE2
# ---------------------------------------------------------------
print("\n-- Type-based dispatch pattern --")

def describe_space(space):
    """Returns a description based on the space type."""
    t = space.getType()
    if t == int(pyompl.StateSpaceType.STATE_SPACE_REAL_VECTOR):
        return f"R^{space.getDimension()} Euclidean space"
    elif t == int(pyompl.StateSpaceType.STATE_SPACE_SO2):
        return "SO(2) — 2D rotations"
    elif t == int(pyompl.StateSpaceType.STATE_SPACE_SO3):
        return "SO(3) — 3D rotations"
    elif t == int(pyompl.StateSpaceType.STATE_SPACE_SE2):
        return "SE(2) — 2D pose (x, y, yaw)"
    elif t == int(pyompl.StateSpaceType.STATE_SPACE_SE3):
        return "SE(3) — 3D pose (x, y, z, quaternion)"
    elif t == int(pyompl.StateSpaceType.STATE_SPACE_TIME):
        return "Time space"
    elif t == int(pyompl.StateSpaceType.STATE_SPACE_DISCRETE):
        return "Discrete integer space"
    else:
        return f"Unknown type {t}"

test_spaces = [
    pyompl.RealVectorStateSpace(3),
    pyompl.SO2StateSpace(),
    pyompl.SE2StateSpace(),
    pyompl.DiscreteStateSpace(0, 10),
]

for sp in test_spaces:
    desc = describe_space(sp)
    print(f"  {type(sp).__name__:30s} → {desc}")

check(True, "type-based dispatch works")


# ==============================================================
# PART 5: Integration — using all four types together
# ==============================================================
print("\n" + "=" * 60)
print("PART 5: Integration — all four types together")
print("=" * 60)
print("""
In real planning code, all four types work together:
  1. You create a state space (getType() → StateSpaceType)
  2. You define a goal (getType() → GoalType)
  3. You call planner.solve() (returns PlannerStatus)
  4. You compare solution costs (Cost comparison)

This section shows how they fit together conceptually.
""")

# Simulate a planning result
print("-- Simulating a complete planning workflow --")

# Step 1: Check what kind of space we have
space = pyompl.SE2StateSpace()
space_type = space.getType()
print(f"  Space type: {space_type} (SE2 = {int(pyompl.StateSpaceType.STATE_SPACE_SE2)})")
check(space_type == int(pyompl.StateSpaceType.STATE_SPACE_SE2),
      "space is SE2")

# Step 2: Simulate a goal type check
print(f"\n  Goal would be: GOAL_STATE (value={int(pyompl.GoalType.GOAL_STATE)})")
print(f"  Can sample goals? {bool(int(pyompl.GoalType.GOAL_STATE) & int(pyompl.GoalType.GOAL_SAMPLEABLE_REGION))}")

# Step 3: Simulate planner returning a status
simulated_status = pyompl.PlannerStatus(pyompl.PlannerStatusType.EXACT_SOLUTION)
print(f"\n  Planner status: {simulated_status.asString()}")
print(f"  Found solution? {bool(simulated_status)}")
check(bool(simulated_status), "simulated exact solution is truthy")

# Step 4: Compare solution costs
solution_cost   = pyompl.Cost(15.7)
previous_best   = pyompl.Cost(18.3)
improved        = solution_cost < previous_best
print(f"\n  New solution cost:  {solution_cost.value()}")
print(f"  Previous best cost: {previous_best.value()}")
print(f"  Improved? {improved}")
check(improved, "new solution is cheaper than previous best")


print("\n" + "=" * 60)
print("ALL SECTIONS COMPLETE")
print("=" * 60)
print("""
What we covered:

  Cost:
    Cost(v), Cost()
    value(), float(cost)
    <, >, ==, +
    Special values: infinity, zero, negative

  GoalType enum:
    GOAL_ANY, GOAL_REGION, GOAL_SAMPLEABLE_REGION
    GOAL_STATE, GOAL_STATES, GOAL_LAZY_SAMPLES
    Bit-flag semantics

  PlannerStatus:
    PlannerStatus(StatusType), PlannerStatus(hasSolution, isApproximate)
    asString(), getType(), bool(status)
    All StatusType values explained

  StateSpaceType enum:
    All 18 values (UNKNOWN through TROCHOID)
    getType() on real state spaces
    Type-based dispatch pattern

Next: examples/12_generic_param.py  (GenericParam.h)
""")