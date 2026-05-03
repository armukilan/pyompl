#pragma once
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

// Console / logging
#include <ompl/util/Console.h>

// Random numbers
#include <ompl/util/RandomNumbers.h>

// Time utilities
#include <ompl/util/Time.h>

// Geometric equations
#include <ompl/util/GeometricEquations.h>

// PPM image
#include <ompl/util/PPM.h>

// Prolate Hyperspheroid
#include <ompl/util/ProlateHyperspheroid.h>

#include <sstream>
#include <vector>
#include <cstdint>

namespace py = pybind11;

// =============================================================================
// WHAT IS OMPL UTIL?
// ------------------
// The util/ folder contains utility classes used throughout OMPL:
//   Console       — logging system (control OMPL's output verbosity)
//   RNG           — random number generator (seed for reproducibility)
//   Time          — timing utilities (measure planning time)
//   GeometricEquations — n-ball volume calculations
//   PPM           — PPM image loader (2D obstacle maps)
//   ProlateHyperspheroid — ellipse used by Informed RRT*, BIT*
// =============================================================================

// ===========================================================================
// CONSOLE helpers
// ===========================================================================

static ompl::msg::LogLevel console_get_log_level()
{
    // C++: ompl::msg::getLogLevel()
    // Returns the current minimum log level.
    // Messages below this level are suppressed.
    return ompl::msg::getLogLevel();
}

static void console_set_log_level(ompl::msg::LogLevel level)
{
    // C++: ompl::msg::setLogLevel(level)
    // Set the minimum log level. Messages below this are not shown.
    // Use LOG_NONE to silence all OMPL output.
    // Use LOG_DEBUG to see everything.
    ompl::msg::setLogLevel(level);
}

static void console_no_output()
{
    // C++: ompl::msg::noOutputHandler()
    // Disables ALL OMPL console output.
    // Useful when running benchmarks or tests silently.
    ompl::msg::noOutputHandler();
}

static void console_restore_output()
{
    // C++: ompl::msg::restorePreviousOutputHandler()
    // Restores the previous output handler (undoes noOutputHandler).
    ompl::msg::restorePreviousOutputHandler();
}

static void console_use_std_output()
{
    // C++: ompl::msg::useOutputHandler(new OutputHandlerSTD())
    // Switches to the default stdout output handler.
    static ompl::msg::OutputHandlerSTD stdHandler;
    ompl::msg::useOutputHandler(&stdHandler);
}

// ===========================================================================
// RNG helpers
// ===========================================================================

static std::vector<double> rng_quaternion(ompl::RNG &r)
{
    // C++: rng.quaternion(value)
    // Generates a uniformly random unit quaternion.
    // Returns [x, y, z, w] as a 4-element list.
    // Used internally by SO3StateSampler.
    double q[4];
    r.quaternion(q);
    return {q[0], q[1], q[2], q[3]};
}

static std::vector<double> rng_euler_rpy(ompl::RNG &r)
{
    // C++: rng.eulerRPY(value)
    // Generates uniform random Euler angles (roll, pitch, yaw).
    // Each in (-pi, pi]. Returns [roll, pitch, yaw].
    double rpy[3];
    r.eulerRPY(rpy);
    return {rpy[0], rpy[1], rpy[2]};
}

static std::vector<double> rng_uniform_normal_vector(ompl::RNG &r, int dim)
{
    // C++: rng.uniformNormalVector(v)
    // Samples a random unit-length vector (point on unit n-sphere surface).
    // Returns a list of 'dim' doubles.
    std::vector<double> v(dim);
    r.uniformNormalVector(v);
    return v;
}

static std::vector<double> rng_uniform_in_ball(ompl::RNG &r, double radius, int dim)
{
    // C++: rng.uniformInBall(r, v)
    // Samples uniformly inside an n-ball of given radius.
    // Returns a list of 'dim' doubles.
    std::vector<double> v(dim);
    r.uniformInBall(radius, v);
    return v;
}

// ===========================================================================
// Time helpers
// ===========================================================================

static ompl::time::point time_now()
{
    // C++: ompl::time::now()
    // Returns the current time point.
    return ompl::time::now();
}

static double time_seconds_since(const ompl::time::point &start)
{
    // Convenience: seconds elapsed since 'start'.
    // C++: ompl::time::seconds(ompl::time::now() - start)
    return ompl::time::seconds(ompl::time::now() - start);
}

static double time_duration_to_seconds(const ompl::time::duration &d)
{
    // C++: ompl::time::seconds(d)
    // Convert a duration to a double in seconds.
    return ompl::time::seconds(d);
}

static ompl::time::duration time_seconds_to_duration(double sec)
{
    // C++: ompl::time::seconds(sec)
    // Convert a double (seconds) to a duration object.
    return ompl::time::seconds(sec);
}

static std::string time_point_as_string(const ompl::time::point &p)
{
    // C++: ompl::time::as_string(p)
    // Returns a human-readable string like "2025-01-15 14:30:22".
    return ompl::time::as_string(p);
}

// ===========================================================================
// ProlateHyperspheroid helpers
// ===========================================================================

static std::shared_ptr<ompl::ProlateHyperspheroid>
phs_create(unsigned int n,
           const std::vector<double> &focus1,
           const std::vector<double> &focus2)
{
    // C++: ProlateHyperspheroid phs(n, focus1, focus2)
    // Creates an n-dimensional prolate hyperspheroid with two foci.
    // Used by Informed RRT* and BIT* to sample only in the promising region.
    return std::make_shared<ompl::ProlateHyperspheroid>(
        n, focus1.data(), focus2.data());
}

static std::vector<double> phs_transform(
    const ompl::ProlateHyperspheroid &phs,
    const std::vector<double> &sphere)
{
    // C++: phs.transform(sphere, phs_point)
    // Transforms a point on a unit sphere to a point in the PHS.
    // Used to map uniform sphere samples to uniform PHS samples.
    std::vector<double> result(phs.getDimension());
    phs.transform(sphere.data(), result.data());
    return result;
}

static bool phs_is_in(const ompl::ProlateHyperspheroid &phs,
                       const std::vector<double> &point)
{
    // C++: phs.isInPhs(point)
    // Returns True if 'point' lies inside the prolate hyperspheroid.
    return phs.isInPhs(point.data());
}

static bool phs_is_on(const ompl::ProlateHyperspheroid &phs,
                       const std::vector<double> &point)
{
    // C++: phs.isOnPhs(point)
    // Returns True if 'point' lies on the surface of the PHS.
    return phs.isOnPhs(point.data());
}

static double phs_get_path_length(const ompl::ProlateHyperspheroid &phs,
                                   const std::vector<double> &point)
{
    // C++: phs.getPathLength(point)
    // Returns the sum of distances from 'point' to each focus.
    // = the transverse diameter of the ellipse on which the point lies.
    return phs.getPathLength(point.data());
}

// ===========================================================================
// Binding function
// ===========================================================================

inline void bind_util(py::module_ &m)
{
    // -----------------------------------------------------------------------
    // Create a submodule for util
    // -----------------------------------------------------------------------
    auto util = m.def_submodule("util",
        "OMPL utility classes: logging, random numbers, timing, geometry.\n\n"
        "Usage:\n"
        "  import pyompl\n"
        "  pyompl.util.setLogLevel(pyompl.util.LogLevel.LOG_NONE)\n"
        "  rng = pyompl.util.RNG()\n"
        "  t = pyompl.util.now()");

    // =======================================================================
    // LogLevel enum
    // C++: ompl::msg::LogLevel
    // =======================================================================
    py::enum_<ompl::msg::LogLevel>(util, "LogLevel",
        "Log level for OMPL console output.\n\n"
        "C++: ompl::msg::LogLevel\n"
        "C++ header: ompl/util/Console.h\n\n"
        "Controls which messages OMPL prints to the console.\n"
        "Only messages at or above the set level are shown.")
        .value("LOG_DEV2",  ompl::msg::LOG_DEV2,  "Developer-only messages (most verbose)")
        .value("LOG_DEV1",  ompl::msg::LOG_DEV1,  "Developer-only messages")
        .value("LOG_DEBUG", ompl::msg::LOG_DEBUG,  "Debug messages")
        .value("LOG_INFO",  ompl::msg::LOG_INFO,   "Informational messages (default)")
        .value("LOG_WARN",  ompl::msg::LOG_WARN,   "Warnings only")
        .value("LOG_ERROR", ompl::msg::LOG_ERROR,  "Errors only")
        .value("LOG_NONE",  ompl::msg::LOG_NONE,   "Suppress all output")
        .export_values();

    // =======================================================================
    // Console / Logging functions
    // C++ header: ompl/util/Console.h
    // =======================================================================
    util.def("getLogLevel", &console_get_log_level,
        "Return the current minimum log level.\n\n"
        "C++: ompl::msg::getLogLevel()\n\n"
        "Messages below this level are suppressed.");

    util.def("setLogLevel", &console_set_log_level, py::arg("level"),
        "Set the minimum log level.\n\n"
        "C++: ompl::msg::setLogLevel(level)\n\n"
        "Messages below this level will not be shown.\n\n"
        "Common usage:\n"
        "  pyompl.util.setLogLevel(pyompl.util.LogLevel.LOG_NONE)   # silence all\n"
        "  pyompl.util.setLogLevel(pyompl.util.LogLevel.LOG_WARN)   # warnings only\n"
        "  pyompl.util.setLogLevel(pyompl.util.LogLevel.LOG_DEBUG)  # see everything");

    util.def("noOutputHandler", &console_no_output,
        "Disable ALL OMPL console output.\n\n"
        "C++: ompl::msg::noOutputHandler()\n\n"
        "Use for silent benchmarking or testing.\n"
        "Call restorePreviousOutputHandler() to re-enable.");

    util.def("restorePreviousOutputHandler", &console_restore_output,
        "Restore the output handler that was active before noOutputHandler().\n\n"
        "C++: ompl::msg::restorePreviousOutputHandler()");

    util.def("useStdOutputHandler", &console_use_std_output,
        "Switch to the default stdout output handler.\n\n"
        "C++: ompl::msg::useOutputHandler(new OutputHandlerSTD())\n\n"
        "Useful if you previously redirected output and want to restore it.");

    // =======================================================================
    // RNG — Random Number Generator
    // C++ class: ompl::RNG
    // C++ header: ompl/util/RandomNumbers.h
    // =======================================================================
    py::class_<ompl::RNG>(util, "RNG",
        "OMPL random number generator.\n\n"
        "C++ class:  ompl::RNG\n"
        "C++ header: ompl/util/RandomNumbers.h\n\n"
        "WHAT IS RNG?\n"
        "The central random number generator used by all OMPL samplers.\n"
        "Each instance gets a unique seed automatically.\n\n"
        "WHY USE IT FROM PYTHON?\n"
        "  - Generate random numbers with the same distribution as OMPL\n"
        "  - Set a global seed for REPRODUCIBLE planning results\n"
        "  - Sample quaternions, unit vectors, ball interiors\n\n"
        "REPRODUCIBILITY:\n"
        "  RNG.setSeed(42)  → all subsequent RNG instances use seed 42\n"
        "  This makes planning deterministic across runs.")

        .def(py::init<>(),
            "Construct with a unique auto-generated seed.\n\n"
            "C++: ompl::RNG rng;")

        .def(py::init<std::uint_fast32_t>(), py::arg("seed"),
            "Construct with a specific seed for reproducibility.\n\n"
            "C++: ompl::RNG rng(seed);")

        // Basic distributions
        .def("uniform01", &ompl::RNG::uniform01,
            "Generate a random double in [0, 1).\n\n"
            "C++: rng.uniform01()")

        .def("uniformReal", &ompl::RNG::uniformReal,
            py::arg("lower_bound"), py::arg("upper_bound"),
            "Generate a random double in [lower_bound, upper_bound).\n\n"
            "C++: rng.uniformReal(lower_bound, upper_bound)")

        .def("uniformInt", &ompl::RNG::uniformInt,
            py::arg("lower_bound"), py::arg("upper_bound"),
            "Generate a random integer in [lower_bound, upper_bound] (inclusive).\n\n"
            "C++: rng.uniformInt(lower_bound, upper_bound)")

        .def("uniformBool", &ompl::RNG::uniformBool,
            "Generate a random boolean (50/50).\n\n"
            "C++: rng.uniformBool()")

        .def("gaussian01", &ompl::RNG::gaussian01,
            "Generate a random double from N(0, 1).\n\n"
            "C++: rng.gaussian01()")

        .def("gaussian", &ompl::RNG::gaussian,
            py::arg("mean"), py::arg("stddev"),
            "Generate a random double from N(mean, stddev^2).\n\n"
            "C++: rng.gaussian(mean, stddev)")

        .def("halfNormalReal", &ompl::RNG::halfNormalReal,
            py::arg("r_min"), py::arg("r_max"), py::arg("focus") = 3.0,
            "Generate a double in [r_min, r_max] biased toward r_max.\n\n"
            "C++: rng.halfNormalReal(r_min, r_max, focus)\n\n"
            "Higher focus → more probability near r_max.\n"
            "Used by planners to bias sampling toward promising regions.")

        .def("halfNormalInt", &ompl::RNG::halfNormalInt,
            py::arg("r_min"), py::arg("r_max"), py::arg("focus") = 3.0,
            "Generate an integer in [r_min, r_max] biased toward r_max.\n\n"
            "C++: rng.halfNormalInt(r_min, r_max, focus)")

        // Geometric sampling
        .def("quaternion", &rng_quaternion,
            "Generate a uniformly random unit quaternion [x, y, z, w].\n\n"
            "C++: rng.quaternion(value)  (writes to double[4])\n\n"
            "Used internally by SO3StateSampler.\n"
            "Returns [x, y, z, w] as a Python list.")

        .def("eulerRPY", &rng_euler_rpy,
            "Generate uniform random Euler angles [roll, pitch, yaw].\n\n"
            "C++: rng.eulerRPY(value)  (writes to double[3])\n\n"
            "Each angle is in (-pi, pi].\n"
            "Returns [roll, pitch, yaw] as a Python list.")

        .def("uniformNormalVector", &rng_uniform_normal_vector,
            py::arg("dim"),
            "Sample a random unit-length vector in R^dim.\n\n"
            "C++: rng.uniformNormalVector(v)  (v is std::vector<double>)\n\n"
            "Returns a point on the surface of the unit n-sphere.\n"
            "Returns a Python list of 'dim' doubles.")

        .def("uniformInBall", &rng_uniform_in_ball,
            py::arg("radius"), py::arg("dim"),
            "Sample uniformly inside an n-ball of given radius.\n\n"
            "C++: rng.uniformInBall(r, v)  (v is std::vector<double>)\n\n"
            "The distribution is uniform in Cartesian coordinates.\n"
            "Returns a Python list of 'dim' doubles.")

        // Seed control
        .def("getLocalSeed", &ompl::RNG::getLocalSeed,
            "Return the seed used by THIS instance.\n\n"
            "C++: rng.getLocalSeed()\n\n"
            "Save this to reproduce the same sequence later.")

        .def("setLocalSeed", &ompl::RNG::setLocalSeed, py::arg("seed"),
            "Set the seed for THIS instance.\n\n"
            "C++: rng.setLocalSeed(seed)\n\n"
            "Resets the generator to produce the same sequence.")

        .def_static("getSeed", &ompl::RNG::getSeed,
            "Return the global seed used to generate all instance seeds.\n\n"
            "C++: ompl::RNG::getSeed()  (static)\n\n"
            "Save this value and pass to setSeed() for full reproducibility.")

        .def_static("setSeed", &ompl::RNG::setSeed, py::arg("seed"),
            "Set the GLOBAL seed for ALL subsequent RNG instances.\n\n"
            "C++: ompl::RNG::setSeed(seed)  (static)\n\n"
            "CRITICAL for reproducibility: call this BEFORE creating\n"
            "any SpaceInformation, planners, or samplers.\n\n"
            "Example:\n"
            "  pyompl.util.RNG.setSeed(42)  # deterministic planning");

    // =======================================================================
    // Time utilities
    // C++ namespace: ompl::time
    // C++ header: ompl/util/Time.h
    // =======================================================================

    // time::point — opaque time point type
    py::class_<ompl::time::point>(util, "TimePoint",
        "A point in time.\n\n"
        "C++: ompl::time::point (= std::chrono::system_clock::time_point)\n\n"
        "Obtain via pyompl.util.now().\n"
        "Use with elapsed() to measure time intervals.")
        .def("__repr__", [](const ompl::time::point &p) {
            return "<TimePoint " + ompl::time::as_string(p) + ">";
        });

    // time::duration — opaque duration type
    py::class_<ompl::time::duration>(util, "TimeDuration",
        "A time duration.\n\n"
        "C++: ompl::time::duration (= std::chrono::system_clock::duration)\n\n"
        "Obtain via pyompl.util.seconds(float) or by subtracting two TimePoints.")
        .def("__repr__", [](const ompl::time::duration &d) {
            return "<TimeDuration " + std::to_string(ompl::time::seconds(d)) + "s>";
        });

    util.def("now", &time_now,
        "Return the current time point.\n\n"
        "C++: ompl::time::now()\n\n"
        "Use to start a timer:\n"
        "  start = pyompl.util.now()\n"
        "  ... do work ...\n"
        "  elapsed = pyompl.util.elapsed(start)");

    util.def("elapsed", &time_seconds_since, py::arg("start"),
        "Return seconds elapsed since 'start'.\n\n"
        "C++: ompl::time::seconds(ompl::time::now() - start)\n\n"
        "Example:\n"
        "  start = pyompl.util.now()\n"
        "  elapsed = pyompl.util.elapsed(start)  # float in seconds");

    util.def("secondsToDuration", &time_seconds_to_duration, py::arg("seconds"),
        "Convert a float (seconds) to a TimeDuration object.\n\n"
        "C++: ompl::time::seconds(sec)\n\n"
        "Used to set planning time limits:\n"
        "  planner.solve(pyompl.util.secondsToDuration(5.0))");

    util.def("durationToSeconds", &time_duration_to_seconds, py::arg("duration"),
        "Convert a TimeDuration to float seconds.\n\n"
        "C++: ompl::time::seconds(d)");

    util.def("asString", &time_point_as_string, py::arg("point"),
        "Return a human-readable string for a time point.\n\n"
        "C++: ompl::time::as_string(p)\n\n"
        "Returns something like '2025-01-15 14:30:22'.");

    // =======================================================================
    // Geometric Equations
    // C++ header: ompl/util/GeometricEquations.h
    // =======================================================================
    util.def("nBallMeasure", &ompl::nBallMeasure,
        py::arg("N"), py::arg("r"),
        "Return the volume of an N-dimensional ball of radius r.\n\n"
        "C++: ompl::nBallMeasure(N, r)\n\n"
        "Formula: pi^(N/2) * r^N / Gamma(N/2 + 1)\n\n"
        "Examples:\n"
        "  nBallMeasure(2, 1.0) = pi        (area of unit circle)\n"
        "  nBallMeasure(3, 1.0) = 4pi/3     (volume of unit sphere)\n\n"
        "Used by planners to compute sampling probabilities.");

    util.def("unitNBallMeasure", &ompl::unitNBallMeasure,
        py::arg("N"),
        "Return the volume of an N-dimensional unit ball (radius=1).\n\n"
        "C++: ompl::unitNBallMeasure(N)\n\n"
        "Equivalent to nBallMeasure(N, 1.0).");

    util.def("prolateHyperspheroidMeasure", &ompl::prolateHyperspheroidMeasure,
        py::arg("N"), py::arg("dFoci"), py::arg("dTransverse"),
        "Return the volume of an N-dimensional prolate hyperspheroid.\n\n"
        "C++: ompl::prolateHyperspheroidMeasure(N, dFoci, dTransverse)\n\n"
        "Parameters:\n"
        "  N           — dimension\n"
        "  dFoci       — distance between the two foci\n"
        "  dTransverse — transverse diameter (major axis length)\n\n"
        "Used by Informed RRT* to compute the fraction of space being sampled.");

    // =======================================================================
    // PPM image loader
    // C++ class: ompl::PPM
    // C++ header: ompl/util/PPM.h
    // =======================================================================
    py::class_<ompl::PPM::Color>(util, "PPMColor",
        "An RGB pixel color.\n\n"
        "C++ type: ompl::PPM::Color\n"
        "Fields: red, green, blue (each unsigned char, 0-255)")
        .def(py::init<>())
        .def_readwrite("red",   &ompl::PPM::Color::red,   "Red channel (0-255)")
        .def_readwrite("green", &ompl::PPM::Color::green, "Green channel (0-255)")
        .def_readwrite("blue",  &ompl::PPM::Color::blue,  "Blue channel (0-255)")
        .def("__eq__", &ompl::PPM::Color::operator==)
        .def("__repr__", [](const ompl::PPM::Color &c) {
            return "<PPMColor r=" + std::to_string(c.red) +
                   " g=" + std::to_string(c.green) +
                   " b=" + std::to_string(c.blue) + ">";
        });

    py::class_<ompl::PPM>(util, "PPM",
        "Load and save PPM (Portable Pixmap) image files.\n\n"
        "C++ class:  ompl::PPM\n"
        "C++ header: ompl/util/PPM.h\n\n"
        "WHAT IS PPM?\n"
        "A simple uncompressed image format. OMPL uses PPM images\n"
        "as 2D obstacle maps for demo problems:\n"
        "  - White pixels = free space\n"
        "  - Dark pixels  = obstacles\n\n"
        "USAGE:\n"
        "  img = pyompl.util.PPM('map.ppm')\n"
        "  color = img.getPixel(row, col)\n"
        "  if color.red == 255: # free space\n"
        "      ...")

        .def(py::init<>(),
            "Construct an empty PPM image.\n"
            "C++: ompl::PPM ppm;")

        .def(py::init<const char *>(), py::arg("filename"),
            "Load a PPM file immediately.\n\n"
            "C++: ompl::PPM ppm(filename);\n\n"
            "Throws RuntimeError if file cannot be read.")

        .def("loadFile", &ompl::PPM::loadFile, py::arg("filename"),
            "Load a PPM file.\n\n"
            "C++: ppm.loadFile(filename);\n\n"
            "Throws RuntimeError if file cannot be read.")

        .def("saveFile", &ompl::PPM::saveFile, py::arg("filename"),
            "Save current image data to a PPM file.\n\n"
            "C++: ppm.saveFile(filename);\n\n"
            "Throws RuntimeError if file cannot be written.")

        .def("getWidth",  &ompl::PPM::getWidth,
            "Return the image width in pixels.\n"
            "C++: ppm.getWidth()")

        .def("getHeight", &ompl::PPM::getHeight,
            "Return the image height in pixels.\n"
            "C++: ppm.getHeight()")

        .def("setWidth",  &ompl::PPM::setWidth,  py::arg("width"),
            "Set the image width. Must match pixel count for saveFile().\n"
            "C++: ppm.setWidth(width);")

        .def("setHeight", &ompl::PPM::setHeight, py::arg("height"),
            "Set the image height. Must match pixel count for saveFile().\n"
            "C++: ppm.setHeight(height);")

        .def("getPixel",
            py::overload_cast<int, int>(&ompl::PPM::getPixel),
            py::arg("row"), py::arg("col"),
            py::return_value_policy::reference_internal,
            "Return the pixel at (row, col).\n\n"
            "C++: ppm.getPixel(row, col)\n\n"
            "Access pattern: pixel at row r, column c:\n"
            "  color = img.getPixel(r, c)\n"
            "  if color.red > 127: # light = free space")

        .def("getPixels",
            py::overload_cast<>(&ompl::PPM::getPixels),
            py::return_value_policy::reference_internal,
            "Return the flat pixel array (row-major order).\n\n"
            "C++: ppm.getPixels()\n\n"
            "Access pixel at (row, col) via:\n"
            "  pixels[row * width + col]");

    // =======================================================================
    // ProlateHyperspheroid
    // C++ class: ompl::ProlateHyperspheroid
    // C++ header: ompl/util/ProlateHyperspheroid.h
    // =======================================================================
    py::class_<ompl::ProlateHyperspheroid,
               std::shared_ptr<ompl::ProlateHyperspheroid>>(util, "ProlateHyperspheroid",
        "An n-dimensional prolate hyperspheroid (symmetric hyperellipse).\n\n"
        "C++ class:  ompl::ProlateHyperspheroid\n"
        "C++ header: ompl/util/ProlateHyperspheroid.h\n\n"
        "WHAT IS A PROLATE HYPERSPHEROID?\n"
        "A symmetric n-dimensional ellipse with two foci.\n"
        "All points where (dist_to_focus1 + dist_to_focus2) < c lie inside.\n\n"
        "WHY IS THIS USEFUL?\n"
        "Informed RRT* and BIT* use this to focus sampling:\n"
        "  - focus1 = start state, focus2 = goal state\n"
        "  - transverseDiameter = current best path length\n"
        "  - Only states INSIDE the PHS can improve the solution\n"
        "  - Sampling only inside the PHS is more efficient!\n\n"
        "REFERENCE:\n"
        "  Gammell et al., 'Informed sampling for asymptotically optimal\n"
        "  path planning.' IEEE T-RO 2018.")

        .def(py::init(&phs_create),
            py::arg("n"), py::arg("focus1"), py::arg("focus2"),
            "Create an n-dimensional PHS with two foci.\n\n"
            "C++: ProlateHyperspheroid phs(n, focus1, focus2);\n\n"
            "Parameters:\n"
            "  n      — dimension\n"
            "  focus1 — list of n doubles (start point)\n"
            "  focus2 — list of n doubles (goal point)")

        .def("setTransverseDiameter", &ompl::ProlateHyperspheroid::setTransverseDiameter,
            py::arg("transverseDiameter"),
            "Set the transverse diameter (major axis length).\n\n"
            "C++: phs.setTransverseDiameter(d)\n\n"
            "In informed planning: set to the current best path cost.\n"
            "Only states where path_length <= transverseDiameter are inside.")

        .def("transform", &phs_transform, py::arg("sphere"),
            "Transform a unit-sphere point to a PHS point.\n\n"
            "C++: phs.transform(sphere, phs_point)\n\n"
            "Maps a point from the unit sphere to the PHS.\n"
            "Used for sampling: sample sphere uniformly, then transform.")

        .def("isInPhs", &phs_is_in, py::arg("point"),
            "Return True if point lies inside the PHS.\n\n"
            "C++: phs.isInPhs(point)")

        .def("isOnPhs", &phs_is_on, py::arg("point"),
            "Return True if point lies on the PHS surface.\n\n"
            "C++: phs.isOnPhs(point)")

        .def("getPhsDimension", &ompl::ProlateHyperspheroid::getPhsDimension,
            "Return the dimension of the PHS.\n\n"
            "C++: phs.getPhsDimension()")

        .def("getDimension", &ompl::ProlateHyperspheroid::getDimension,
            "Return the state dimension.\n\n"
            "C++: phs.getDimension()")

        .def("getPhsMeasure",
            py::overload_cast<>(&ompl::ProlateHyperspheroid::getPhsMeasure, py::const_),
            "Return the volume of the PHS (current transverse diameter).\n\n"
            "C++: phs.getPhsMeasure()")

        .def("getPhsMeasure",
            py::overload_cast<double>(&ompl::ProlateHyperspheroid::getPhsMeasure, py::const_),
            py::arg("transverseDiameter"),
            "Return the volume of the PHS for a given transverse diameter.\n\n"
            "C++: phs.getPhsMeasure(tranDiam)")

        .def("getMinTransverseDiameter",
            &ompl::ProlateHyperspheroid::getMinTransverseDiameter,
            "Return the distance between the two foci.\n\n"
            "C++: phs.getMinTransverseDiameter()\n\n"
            "This is the minimum possible transverse diameter\n"
            "(a straight line from focus1 to focus2).")

        .def("getPathLength", &phs_get_path_length, py::arg("point"),
            "Return sum of distances from point to each focus.\n\n"
            "C++: phs.getPathLength(point)\n\n"
            "= dist(point, focus1) + dist(point, focus2)\n"
            "If this <= transverseDiameter → point is inside PHS.");
}