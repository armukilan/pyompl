import pyompl
# from pyompl import base as ob   # if you namespace it this way instead

def check(condition, msg):
    print(f"  {'PASS' if condition else 'FAIL'} — {msg}")


# ==============================================================
# PART 1: State — abstract base class (ompl/base/State.h)
# ==============================================================
print("=" * 60)
print("PART 1: State (ompl/base/State.h)")
print("=" * 60)
print("""
WHAT IS ompl::base::State?
----------------------------
The abstract base class for every state type in OMPL
(RealVectorState, SO2State, SE3State, etc).

In C++, State has a PROTECTED constructor/destructor — it is
never created directly. States are only ever allocated by a
StateSpace via StateSpace::allocState() and freed via
StateSpace::freeState().

WHY CAN'T WE MAKE ONE FROM PYTHON?
  Since the binding disables py::init() for State, trying to
  call pyompl.State() should raise a TypeError, matching the
  C++ restriction.

C++ class: ompl::base::State
""")

# State should exist as a type, but not be constructible
print("-- State type exists --")
check(hasattr(pyompl, "State"), "pyompl.State is exposed")
check(isinstance(pyompl.State, type), "pyompl.State is a class/type object")

print("\n-- State() should fail (protected ctor in C++) --")
try:
    s = pyompl.State()
    check(False, "State() raised TypeError (it did NOT — this is a FAIL)")
except TypeError:
    check(True, "State() raised TypeError as expected")


# ==============================================================
# PART 2: CompoundState — concrete state made of components
# ==============================================================
print("\n" + "=" * 60)
print("PART 2: CompoundState (ompl/base/State.h)")
print("=" * 60)
print("""
WHAT IS ompl::base::CompoundState?
-------------------------------------
A State made up of multiple component states (used by
CompoundStateSpace, e.g. combining SE2 + velocity, etc).

Unlike State, CompoundState has a PUBLIC default constructor
in C++, so it should be constructible from Python.

Its `components` array is a raw State** and starts as nullptr
until a StateSpace allocates into it — so indexing an unallocated
CompoundState is undefined behavior and is NOT tested here.
That will be covered once StateSpace.h bindings (allocState) exist.

C++ class: ompl::base::CompoundState : public State
""")

# Construction
print("-- Constructor --")
cs = pyompl.CompoundState()
print(f"  Created: {cs}")
check(cs is not None, "CompoundState() constructs successfully")

# Inheritance check
print("\n-- Inheritance: CompoundState IS-A State --")
check(isinstance(cs, pyompl.State), "CompoundState instance is also a State")
check(issubclass(pyompl.CompoundState, pyompl.State), "CompoundState is a subclass of State")

# repr sanity check
print("\n-- __repr__ --")
r = repr(cs)
print(f"  repr(cs) = {r}")
check("CompoundState" in r, "repr mentions CompoundState")

# __getitem__ exists but is not called yet (components unallocated)
print("\n-- __getitem__ method exists (not called — components is unallocated) --")
check(hasattr(cs, "__getitem__"), "CompoundState has __getitem__ defined")
print("  Skipping actual indexing — will be exercised once StateSpace.h")
print("  bindings can call allocState() to populate `components`.")


# ==============================================================
# SUMMARY
# ==============================================================
print("\n" + "=" * 60)
print("state.h binding tests complete")
print("=" * 60)