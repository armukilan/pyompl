#pragma once
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <ompl/base/MotionValidator.h>
#include <ompl/base/State.h>

namespace py = pybind11;
using namespace ompl::base;

// ---------------------------------------------------------------------------
// Helper free functions for MotionValidator
// ---------------------------------------------------------------------------

// checkMotion(s1, s2) -> bool
static bool mv_check_motion_simple(const MotionValidator &mv, const State *s1, const State *s2)
{
    return mv.checkMotion(s1, s2);
}

// checkMotion(s1, s2, lastValid) with an in/out std::pair<State*,double>&.
// Python has no equivalent of an in/out struct-by-reference parameter, so this
// wraps it: caller passes a pre-allocated `lastValidState` (e.g. space.allocState())
// which gets overwritten with the last valid state IF the motion is invalid.
// Returns (isValid: bool, lastValidTime: float).
static py::tuple mv_check_motion_with_lastvalid(const MotionValidator &mv,
                                                 const State *s1, const State *s2,
                                                 State *lastValidState)
{
    std::pair<State *, double> lastValid(lastValidState, 0.0);
    bool ok = mv.checkMotion(s1, s2, lastValid);
    return py::make_tuple(ok, lastValid.second);
}

static unsigned int mv_get_valid_count(const MotionValidator &mv)    { return mv.getValidMotionCount(); }
static unsigned int mv_get_invalid_count(const MotionValidator &mv)  { return mv.getInvalidMotionCount(); }
static unsigned int mv_get_checked_count(const MotionValidator &mv)  { return mv.getCheckedMotionCount(); }
static double       mv_get_valid_fraction(const MotionValidator &mv) { return mv.getValidMotionFraction(); }
static void         mv_reset_motion_counter(MotionValidator &mv)     { mv.resetMotionCounter(); }

// ---------------------------------------------------------------------------
// Binding function
// ---------------------------------------------------------------------------

inline void bind_motion_validator(py::module_ &m)
{
    py::class_<MotionValidator, std::shared_ptr<MotionValidator>>(m, "MotionValidator",
        "Abstract base class for checking the validity of motions\n"
        "(path segments between two states) — often called a 'local planner'.\n\n"
        "C++ header: ompl/base/MotionValidator.h\n"
        "C++ class:  ompl::base::MotionValidator\n\n"
        "NOT constructible from Python directly — the C++ class is pure\n"
        "virtual (checkMotion has no default implementation) and its\n"
        "constructor requires a SpaceInformation, which owns and creates\n"
        "concrete validators (e.g. DiscreteMotionValidator) internally.\n"
        "Retrieve an instance via si.getMotionValidator() once\n"
        "SpaceInformation.h is bound.\n\n"
        "Implementations must be thread-safe.")

        .def("checkMotion", &mv_check_motion_simple,
             py::arg("s1"), py::arg("s2"),
             "Check if the motion from s1 to s2 is valid. Assumes s1 is valid.\n\n"
             "C++: bool checkMotion(const State *s1, const State *s2) const\n\n"
             "Updates the internal valid/invalid segment counters.\n\n"
             "  ok = validator.checkMotion(s1, s2)")

        .def("checkMotion", &mv_check_motion_with_lastvalid,
             py::arg("s1"), py::arg("s2"), py::arg("lastValidState"),
             "Check if the motion from s1 to s2 is valid, and if not, record\n"
             "the last valid state along the way.\n\n"
             "C++: bool checkMotion(const State *s1, const State *s2,\n"
             "                      std::pair<State*, double> &lastValid) const\n\n"
             "'lastValidState' must be a pre-allocated State (e.g. from\n"
             "space.allocState()) — this call overwrites it with the last\n"
             "valid state found, IF the motion turns out to be invalid.\n\n"
             "Returns a tuple: (isValid: bool, lastValidTime: float)\n"
             "  lastValidTime in [0, 1], parametrizing s1 (t=0) to s2 (t=1).\n"
             "  If isValid is True, lastValidState/lastValidTime are unchanged.\n\n"
             "Updates the internal valid/invalid segment counters.\n\n"
             "  last = space.allocState()\n"
             "  ok, t = validator.checkMotion(s1, s2, last)\n"
             "  if not ok:\n"
             "      print('motion becomes invalid at t =', t)\n"
             "  space.freeState(last)")

        .def("getValidMotionCount",    &mv_get_valid_count,
             "Return the number of segments that tested as valid so far.\n"
             "C++: validator.getValidMotionCount()")

        .def("getInvalidMotionCount",  &mv_get_invalid_count,
             "Return the number of segments that tested as invalid so far.\n"
             "C++: validator.getInvalidMotionCount()")

        .def("getCheckedMotionCount",  &mv_get_checked_count,
             "Return the total number of segments tested (valid + invalid).\n"
             "C++: validator.getCheckedMotionCount()")

        .def("getValidMotionFraction", &mv_get_valid_fraction,
             "Return the fraction of tested segments that were valid, in [0, 1].\n"
             "Returns 0.0 if no valid segments have been tested yet.\n"
             "C++: validator.getValidMotionFraction()")

        .def("resetMotionCounter",     &mv_reset_motion_counter,
             "Reset the valid/invalid segment counters back to zero.\n"
             "C++: validator.resetMotionCounter();")

        .def("__repr__", [](const MotionValidator &mv) {
            return "<MotionValidator valid=" + std::to_string(mv.getValidMotionCount()) +
                   " invalid=" + std::to_string(mv.getInvalidMotionCount()) + ">";
        });
}