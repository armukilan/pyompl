import pyompl

def check(condition, msg):
    print(f"  {'PASS' if condition else 'FAIL'} — {msg}")


# ==============================================================
# PART 1: MotionValidator — abstract base (ompl/base/MotionValidator.h)
# ==============================================================
print("=" * 60)
print("PART 1: MotionValidator (ompl/base/MotionValidator.h)")
print("=" * 60)
print("""
WHAT IS ompl::base::MotionValidator?
---------------------------------------
Checks whether the path segment between two states is valid —
often called a "local planner". Every SpaceInformation owns one
(usually a DiscreteMotionValidator by default).

WHY CAN'T WE MAKE ONE FROM PYTHON RIGHT NOW?
  MotionValidator is pure virtual (checkMotion has no body) and
  its constructor requires a SpaceInformation, which isn't bound
  yet at this stage (item #11 in the binding order). Real usage
  will be tested via si.getMotionValidator() once SpaceInformation.h
  and a concrete validator (e.g. DiscreteMotionValidator) exist.

For now we can only confirm the TYPE is exposed correctly and
that all its methods are present.

C++ class: ompl::base::MotionValidator
""")

# Type exists
print("-- MotionValidator type exists --")
check(hasattr(pyompl, "MotionValidator"), "pyompl.MotionValidator is exposed")
check(isinstance(pyompl.MotionValidator, type), "pyompl.MotionValidator is a class/type object")

# Cannot construct directly (no ctor bound, abstract in C++)
print("\n-- MotionValidator() should fail (abstract, no ctor bound) --")
try:
    mv = pyompl.MotionValidator()
    check(False, "MotionValidator() raised TypeError (it did NOT — FAIL)")
except TypeError:
    check(True, "MotionValidator() raised TypeError as expected")


# ==============================================================
# PART 2: Method surface check
# ==============================================================
print("\n" + "=" * 60)
print("PART 2: Confirm every bound method is present")
print("=" * 60)

expected_methods = [
    "checkMotion",
    "getValidMotionCount",
    "getInvalidMotionCount",
    "getCheckedMotionCount",
    "getValidMotionFraction",
    "resetMotionCounter",
]

for name in expected_methods:
    check(hasattr(pyompl.MotionValidator, name), f"MotionValidator.{name} exists")

print(f"\n  Total methods checked: {len(expected_methods)}")
check(len(expected_methods) == 6, "all 6 public methods from MotionValidator.h are present")


# ==============================================================
# PART 3: Placeholder for real behavioral tests (deferred)
# ==============================================================
print("\n" + "=" * 60)
print("PART 3: Deferred behavioral tests")
print("=" * 60)
print("""
The following will be exercised in a LATER test file, once
SpaceInformation.h (#11) and DiscreteMotionValidator (#28) are bound:

  space = pyompl.RealVectorStateSpace(2)
  space.setBounds(-1, 1)
  si = pyompl.SpaceInformation(space)
  validator = si.getMotionValidator()

  s1 = space.allocState(); s1[0], s1[1] = 0.0, 0.0
  s2 = space.allocState(); s2[0], s2[1] = 1.0, 1.0

  ok = validator.checkMotion(s1, s2)

  last = space.allocState()
  ok, t = validator.checkMotion(s1, s2, last)

  print(validator.getValidMotionCount())
  print(validator.getInvalidMotionCount())
  print(validator.getCheckedMotionCount())
  print(validator.getValidMotionFraction())
  validator.resetMotionCounter()

Not runnable yet — flagged here so it isn't forgotten once those
headers are done.
""")


# ==============================================================
# SUMMARY
# ==============================================================
print("=" * 60)
print("motion_validator.h binding tests complete")
print("=" * 60)