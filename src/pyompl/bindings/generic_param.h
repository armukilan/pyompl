#pragma once
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include <ompl/base/GenericParam.h>

#include <sstream>
#include <string>
#include <map>
#include <vector>

namespace py = pybind11;
using namespace ompl::base;

// =============================================================================
// WHAT IS GenericParam.h?
// -----------------------
// C++ header: ompl/base/GenericParam.h
// C++ classes: ompl::base::GenericParam
//              ompl::base::SpecificParam<T>   (template — not directly bound)
//              ompl::base::ParamSet
//
// WHAT PROBLEM DOES THIS SOLVE?
// Every planner and state space in OMPL has tunable parameters:
//   - RRT has goal_bias (how often to sample the goal)
//   - PRM has max_nearest_neighbors
//   - RealVectorStateSpace has longestValidSegmentFraction
//
// GenericParam provides a UNIFORM INTERFACE to all these parameters
// so they can be set/get by name using strings, without knowing
// the exact type at compile time. This enables:
//   - Configuration files: read param names + values from JSON/YAML
//   - GUIs: display and edit all planner parameters automatically
//   - Benchmarking: sweep parameter values programmatically
//
// HOW IT WORKS (C++ side):
//   SpecificParam<double> wraps a getter+setter function pair.
//   The getter returns a double; the setter takes a double.
//   All values are serialised to/from string for the generic interface.
//
//   ParamSet holds a map<string, GenericParamPtr>.
//   Every StateSpace exposes: space->params()  → ParamSet&
//   Every Planner exposes:   planner->params() → ParamSet&
//
// PYTHON BINDING DECISIONS:
//   GenericParam       — bound as abstract base (no constructor)
//   SpecificParam<T>   — NOT bound (template class)
//                        Instead, ParamSet.declareParamDouble/Int/Bool/String
//                        are provided for Python users to declare params.
//   ParamSet           — fully bound
// =============================================================================


// =============================================================================
// GenericParam free helpers
// =============================================================================

static const std::string &gp_get_name(const GenericParam &p)
{
    // C++: param.getName()
    // Returns the name of this parameter (e.g., "goal_bias", "range").
    // The name is how you look up the param in a ParamSet.
    return p.getName();
}

static void gp_set_name(GenericParam &p, const std::string &name)
{
    // C++: param.setName(name)
    // Renames this parameter. Rarely needed — names are set at creation.
    p.setName(name);
}

static std::string gp_get_value(const GenericParam &p)
{
    // C++: param.getValue()
    // Returns the current value as a STRING regardless of the actual type.
    // A double parameter with value 0.05 returns "0.050000".
    // A bool parameter with value true returns "1".
    // A string parameter returns the string directly.
    return p.getValue();
}

static bool gp_set_value(GenericParam &p, const std::string &value)
{
    // C++: param.setValue(value)
    // Sets the value from a STRING. The string is converted to the
    // actual type (double, int, bool, etc.) internally.
    // Returns True if the conversion and setter succeeded.
    // Returns False if the string could not be parsed as the correct type.
    //
    // Examples:
    //   param.setValue("0.1")    → sets double param to 0.1
    //   param.setValue("true")   → sets bool param to true
    //   param.setValue("abc")    → returns False (invalid for double)
    return p.setValue(value);
}

static const std::string &gp_get_range_suggestion(const GenericParam &p)
{
    // C++: param.getRangeSuggestion()
    // Returns a hint about valid values, used by GUIs and benchmarks.
    //
    // OMPL conventions for range strings:
    //   bool:   "0,1"
    //   enum:   "val0,val1,val2"
    //   int:    "0:100"        (0 to 100, step 1)
    //           "0:5:100"      (0 to 100, step 5)
    //   double: "0.:1."        (0.0 to 1.0, step 1)
    //           "0.:0.05:1."   (0.0 to 1.0, step 0.05)
    //
    // Note: floating-point ranges use "1." not "1" to distinguish from int.
    return p.getRangeSuggestion();
}

static void gp_set_range_suggestion(GenericParam &p, const std::string &r)
{
    // C++: param.setRangeSuggestion(r)
    // Sets a hint about valid values for this parameter.
    // Used by GUIs, benchmarks, and automated parameter sweeps.
    p.setRangeSuggestion(r);
}

static std::string gp_repr(const GenericParam &p)
{
    return "<GenericParam name='" + p.getName() +
           "' value='" + p.getValue() + "'>";
}


// =============================================================================
// ParamSet free helpers
// =============================================================================

// --- declare helpers for common types ---
// These wrap SpecificParam<T> instantiation so Python can declare params
// without dealing with C++ templates.

static void ps_declare_double(ParamSet &ps, const std::string &name,
                               std::function<void(double)> setter,
                               std::function<double()> getter)
{
    // C++: params.declareParam<double>(name, setter, getter)
    // Declares a double parameter with the given name.
    // setter — called when setValue() is used to change the value
    // getter — called when getValue() is used to read the value
    //
    // Example (C++ equivalent):
    //   double myValue_ = 0.5;
    //   params.declareParam<double>("my_param",
    //       [this](double v) { myValue_ = v; },
    //       [this]()         { return myValue_; });
    ps.declareParam<double>(name, setter, getter);
}

static void ps_declare_int(ParamSet &ps, const std::string &name,
                            std::function<void(int)> setter,
                            std::function<int()> getter)
{
    // C++: params.declareParam<int>(name, setter, getter)
    // Declares an integer parameter.
    ps.declareParam<int>(name, setter, getter);
}

static void ps_declare_bool(ParamSet &ps, const std::string &name,
                             std::function<void(bool)> setter,
                             std::function<bool()> getter)
{
    // C++: params.declareParam<bool>(name, setter, getter)
    // Declares a boolean parameter.
    // Values "0"/"false"/"False" → false
    // Values "1"/"true"/"True"   → true
    ps.declareParam<bool>(name, setter, getter);
}

static void ps_declare_string(ParamSet &ps, const std::string &name,
                               std::function<void(std::string)> setter,
                               std::function<std::string()> getter)
{
    // C++: params.declareParam<std::string>(name, setter, getter)
    // Declares a string parameter. No type conversion needed.
    ps.declareParam<std::string>(name, setter, getter);
}

// --- setParam / getParam ---

static bool ps_set_param(ParamSet &ps, const std::string &key,
                          const std::string &value)
{
    // C++: params.setParam(key, value)
    // Sets the parameter named 'key' to 'value' (as a string).
    // The string is converted to the correct type internally.
    // Returns True if the parameter exists and the value was set.
    // Returns False if:
    //   - the parameter name is not found
    //   - the value string cannot be converted to the correct type
    //
    // This is the MAIN WAY to set parameters from external sources
    // (config files, command line arguments, benchmarking scripts).
    return ps.setParam(key, value);
}

static std::string ps_get_param_str(const ParamSet &ps, const std::string &key)
{
    // C++: std::string val; params.getParam(key, val);
    // Gets the value of parameter 'key' as a string.
    // Returns empty string if the parameter is not found.
    std::string val;
    ps.getParam(key, val);
    return val;
}

static bool ps_set_params(ParamSet &ps,
                           const std::map<std::string, std::string> &kv,
                           bool ignoreUnknown)
{
    // C++: params.setParams(kv, ignoreUnknown)
    // Sets multiple parameters at once from a dict of name→value strings.
    // Returns True only if ALL parameters were set successfully.
    // If ignoreUnknown=True, unknown parameter names are silently skipped
    //   (no error, just ignored).
    // If ignoreUnknown=False, unknown names cause a return value of False.
    return ps.setParams(kv, ignoreUnknown);
}

static std::map<std::string, std::string> ps_get_params_map(const ParamSet &ps)
{
    // C++: std::map<string,string> m; params.getParams(m);
    // Returns ALL parameter name→value pairs as a Python dict.
    // All values are strings regardless of their actual type.
    // Useful for: saving state, displaying in GUI, logging.
    std::map<std::string, std::string> m;
    ps.getParams(m);
    return m;
}

static std::vector<std::string> ps_get_param_names(const ParamSet &ps)
{
    // C++: std::vector<string> names; params.getParamNames(names);
    // Returns a list of all declared parameter names.
    // Order is alphabetical (std::map ordering).
    std::vector<std::string> names;
    ps.getParamNames(names);
    return names;
}

static std::vector<std::string> ps_get_param_values(const ParamSet &ps)
{
    // C++: std::vector<string> vals; params.getParamValues(vals);
    // Returns a list of all parameter values as strings.
    // Order matches getParamNames() — same alphabetical order.
    // Zip with getParamNames() to get name-value pairs.
    std::vector<std::string> vals;
    ps.getParamValues(vals);
    return vals;
}

static bool ps_has_param(const ParamSet &ps, const std::string &key)
{
    // C++: params.hasParam(key)
    // Returns True if a parameter with the given name exists.
    // Use before setParam/getParam to avoid errors on unknown names.
    return ps.hasParam(key);
}

static GenericParam &ps_getitem(ParamSet &ps, const std::string &key)
{
    // C++: params[key]
    // Returns the GenericParam object for the given name.
    // Throws an exception if the name is not found.
    // Use hasParam() first to check if the parameter exists.
    return ps[key];
}

static std::size_t ps_size(const ParamSet &ps)
{
    // C++: params.size()
    // Returns the number of declared parameters.
    return ps.size();
}

static void ps_clear(ParamSet &ps)
{
    // C++: params.clear()
    // Removes ALL declared parameters from the set.
    // After this call, size() == 0.
    ps.clear();
}

static void ps_remove(ParamSet &ps, const std::string &name)
{
    // C++: params.remove(name)
    // Removes the parameter with the given name.
    // If the name is not found, this is a no-op.
    ps.remove(name);
}

static void ps_include(ParamSet &ps, const ParamSet &other,
                        const std::string &prefix)
{
    // C++: params.include(other, prefix)
    // Merges all parameters from 'other' into this ParamSet.
    // If prefix is non-empty, each parameter name gets prefixed.
    //
    // Example:
    //   space.params().include(planner.params(), "planner.")
    //   → adds "planner.goal_bias", "planner.range", etc.
    //
    // Used by SimpleSetup to expose planner+space params together.
    ps.include(other, prefix);
}

static std::string ps_print_str(const ParamSet &ps)
{
    // C++: params.print(std::cout)
    // Returns a formatted string of all parameters and their values.
    // Useful for debugging — shows all name=value pairs.
    std::ostringstream oss;
    ps.print(oss);
    return oss.str();
}

static std::string ps_repr(const ParamSet &ps)
{
    return "<ParamSet size=" + std::to_string(ps.size()) + ">";
}


// =============================================================================
// Binding function
// =============================================================================

inline void bind_generic_param(py::module_ &m)
{
    // =========================================================================
    // GenericParam — abstract parameter base class
    // C++ header: ompl/base/GenericParam.h
    // C++ class:  ompl::base::GenericParam
    // =========================================================================
    py::class_<GenericParam, std::shared_ptr<GenericParam>>(m, "GenericParam",
        "Abstract base class for a named, string-serialisable parameter.\n\n"
        "C++ header: ompl/base/GenericParam.h\n"
        "C++ class:  ompl::base::GenericParam\n\n"
        "WHAT IS GenericParam?\n"
        "A named parameter with get/set via strings. Every planner and\n"
        "state space declares its tunable parameters as GenericParam.\n\n"
        "You access parameters through ParamSet (space.params() or\n"
        "planner.params()). GenericParam is the object you get back\n"
        "when you look up a parameter by name.\n\n"
        "CONCRETE TYPES (C++ templates, not directly bound in Python):\n"
        "  SpecificParam<double>  → float parameters\n"
        "  SpecificParam<int>     → integer parameters\n"
        "  SpecificParam<bool>    → boolean parameters\n"
        "  SpecificParam<string>  → string parameters\n\n"
        "Do NOT construct directly. Get from ParamSet[name].")

        .def("getName",  &gp_get_name,
            "Return the name of this parameter.\n\n"
            "C++: param.getName()\n\n"
            "  p = space.params()['goal_bias']\n"
            "  print(p.getName())   # 'goal_bias'")

        .def("setName",  &gp_set_name, py::arg("name"),
            "Rename this parameter.\n\n"
            "C++: param.setName(name)\n\n"
            "Rarely used — names are set at declaration time.")

        .def("getValue", &gp_get_value,
            "Return the current value as a string.\n\n"
            "C++: param.getValue()\n\n"
            "All parameter types (double, int, bool) are serialised\n"
            "to string. Convert back yourself:\n"
            "  v = float(p.getValue())   # for double params\n"
            "  v = int(p.getValue())     # for int params\n"
            "  v = bool(int(p.getValue()))# for bool params")

        .def("setValue", &gp_set_value, py::arg("value"),
            "Set the value from a string. Returns True on success.\n\n"
            "C++: param.setValue(value)\n\n"
            "The string is converted to the actual parameter type.\n"
            "Returns False if conversion fails (e.g. 'abc' for double).\n\n"
            "  p.setValue('0.1')    # set double to 0.1\n"
            "  p.setValue('5')      # set int to 5\n"
            "  p.setValue('1')      # set bool to True\n"
            "  p.setValue('0')      # set bool to False")

        .def("getRangeSuggestion", &gp_get_range_suggestion,
            "Return the suggested value range as a string.\n\n"
            "C++: param.getRangeSuggestion()\n\n"
            "OMPL range string conventions:\n"
            "  bool:   '0,1'\n"
            "  enum:   'val0,val1,val2'\n"
            "  int:    '0:100'         (0 to 100, step 1)\n"
            "          '0:5:100'       (0 to 100, step 5)\n"
            "  double: '0.:1.'         (0.0 to 1.0)\n"
            "          '0.:0.05:1.'    (0.0 to 1.0, step 0.05)\n\n"
            "Note: use '1.' not '1' for doubles to distinguish from int.")

        .def("setRangeSuggestion", &gp_set_range_suggestion, py::arg("range"),
            "Set the suggested value range as a string.\n\n"
            "C++: param.setRangeSuggestion(range)\n\n"
            "Used by GUIs and benchmarking tools to know valid values.\n"
            "  p.setRangeSuggestion('0.:0.05:1.')  # doubles 0 to 1")

        .def("__repr__", &gp_repr);


    // =========================================================================
    // ParamSet — a named collection of GenericParam
    // C++ header: ompl/base/GenericParam.h
    // C++ class:  ompl::base::ParamSet
    // =========================================================================
    py::class_<ParamSet>(m, "ParamSet",
        "A named collection of GenericParam objects.\n\n"
        "C++ header: ompl/base/GenericParam.h\n"
        "C++ class:  ompl::base::ParamSet\n\n"
        "WHAT IS ParamSet?\n"
        "Every StateSpace and Planner in OMPL has a ParamSet accessible\n"
        "via .params(). It holds all tunable parameters for that object.\n\n"
        "HOW TO USE:\n"
        "  # Get the parameter set from a space or planner\n"
        "  ps = space.params()          # once spaces expose params()\n"
        "  ps = planner.params()        # once planners are bound\n\n"
        "  # Set a parameter by name\n"
        "  ps.setParam('longest_valid_segment_fraction', '0.01')\n\n"
        "  # Get a parameter value\n"
        "  val = ps.getParam('longest_valid_segment_fraction')\n\n"
        "  # Check if parameter exists\n"
        "  if ps.hasParam('goal_bias'):\n"
        "      ps.setParam('goal_bias', '0.1')\n\n"
        "  # List all parameters\n"
        "  for name in ps.getParamNames():\n"
        "      print(name, '=', ps.getParam(name))\n\n"
        "DECLARING PARAMETERS (for custom planners/spaces):\n"
        "  value = [0.5]   # use list for mutable closure\n"
        "  ps.declareParamDouble('my_param',\n"
        "      lambda v: value.__setitem__(0, v),\n"
        "      lambda: value[0])\n\n"
        "  # Equivalent C++:\n"
        "  # params_.declareParam<double>('my_param',\n"
        "  #     [this](double v){ myValue_ = v; },\n"
        "  #     [this](){ return myValue_; });")

        .def(py::init<>(),
            "Construct an empty ParamSet.\n\n"
            "C++: ParamSet ps;")

        // --- declare helpers ---
        .def("declareParamDouble", &ps_declare_double,
            py::arg("name"), py::arg("setter"), py::arg("getter"),
            "Declare a double parameter with the given name.\n\n"
            "C++: params.declareParam<double>(name, setter, getter)\n\n"
            "Parameters:\n"
            "  name   — parameter name (e.g. 'goal_bias')\n"
            "  setter — callable(float) → None  called when value changes\n"
            "  getter — callable() → float      called when value is read\n\n"
            "Example:\n"
            "  value = [0.05]   # mutable closure\n"
            "  ps.declareParamDouble('range',\n"
            "      lambda v: value.__setitem__(0, v),\n"
            "      lambda: value[0])\n"
            "  ps.setParam('range', '0.1')\n"
            "  print(value[0])  # 0.1")

        .def("declareParamInt", &ps_declare_int,
            py::arg("name"), py::arg("setter"), py::arg("getter"),
            "Declare an integer parameter with the given name.\n\n"
            "C++: params.declareParam<int>(name, setter, getter)\n\n"
            "  value = [10]\n"
            "  ps.declareParamInt('max_neighbors',\n"
            "      lambda v: value.__setitem__(0, v),\n"
            "      lambda: value[0])")

        .def("declareParamBool", &ps_declare_bool,
            py::arg("name"), py::arg("setter"), py::arg("getter"),
            "Declare a boolean parameter with the given name.\n\n"
            "C++: params.declareParam<bool>(name, setter, getter)\n\n"
            "Values '0'/'false'/'False' → False\n"
            "Values '1'/'true'/'True'   → True\n\n"
            "  flag = [True]\n"
            "  ps.declareParamBool('use_k_nearest',\n"
            "      lambda v: flag.__setitem__(0, v),\n"
            "      lambda: flag[0])")

        .def("declareParamString", &ps_declare_string,
            py::arg("name"), py::arg("setter"), py::arg("getter"),
            "Declare a string parameter with the given name.\n\n"
            "C++: params.declareParam<std::string>(name, setter, getter)\n\n"
            "  s = ['']\n"
            "  ps.declareParamString('description',\n"
            "      lambda v: s.__setitem__(0, v),\n"
            "      lambda: s[0])")

        // --- set/get by name ---
        .def("setParam", &ps_set_param,
            py::arg("key"), py::arg("value"),
            "Set parameter 'key' to 'value' (both strings). Returns True on success.\n\n"
            "C++: params.setParam(key, value)\n\n"
            "This is the PRIMARY way to configure planners and spaces:\n"
            "  ps.setParam('goal_bias', '0.1')\n"
            "  ps.setParam('range', '0.5')\n"
            "  ps.setParam('max_nearest_neighbors', '10')\n\n"
            "Returns False if:\n"
            "  - Parameter name not found\n"
            "  - Value string cannot be parsed as the correct type")

        .def("getParam", &ps_get_param_str,
            py::arg("key"),
            "Get the value of parameter 'key' as a string.\n\n"
            "C++: std::string val; params.getParam(key, val);\n\n"
            "Returns empty string if the parameter does not exist.\n"
            "Check hasParam() first if needed.\n\n"
            "  val = ps.getParam('goal_bias')   # e.g. '0.050000'\n"
            "  f = float(val)                    # convert to float")

        .def("setParams", &ps_set_params,
            py::arg("params"), py::arg("ignoreUnknown") = false,
            "Set multiple parameters from a dict. Returns True if all succeeded.\n\n"
            "C++: params.setParams(kv, ignoreUnknown)\n\n"
            "Parameters:\n"
            "  params       — dict of {name: value_string}\n"
            "  ignoreUnknown— if True, skip unknown names silently\n\n"
            "  ps.setParams({\n"
            "      'goal_bias': '0.1',\n"
            "      'range':     '0.5',\n"
            "  })\n\n"
            "Useful for loading configuration from files.")

        .def("getParams", &ps_get_params_map,
            "Return all parameters as a dict of {name: value_string}.\n\n"
            "C++: std::map<string,string> m; params.getParams(m);\n\n"
            "  d = ps.getParams()\n"
            "  for name, val in d.items():\n"
            "      print(f'{name} = {val}')")

        .def("getParamNames", &ps_get_param_names,
            "Return a list of all parameter names (alphabetical order).\n\n"
            "C++: std::vector<string> names; params.getParamNames(names);\n\n"
            "  names = ps.getParamNames()\n"
            "  for name in names:\n"
            "      print(name)")

        .def("getParamValues", &ps_get_param_values,
            "Return all parameter values as strings (same order as getParamNames).\n\n"
            "C++: std::vector<string> vals; params.getParamValues(vals);\n\n"
            "  names  = ps.getParamNames()\n"
            "  values = ps.getParamValues()\n"
            "  for n, v in zip(names, values):\n"
            "      print(f'{n} = {v}')")

        .def("hasParam", &ps_has_param, py::arg("key"),
            "Return True if a parameter named 'key' exists.\n\n"
            "C++: params.hasParam(key)\n\n"
            "Use before getParam/setParam to avoid errors:\n"
            "  if ps.hasParam('goal_bias'):\n"
            "      ps.setParam('goal_bias', '0.2')")

        .def("__getitem__", &ps_getitem,
            py::arg("key"),
            py::return_value_policy::reference_internal,
            "Return the GenericParam object for the given name.\n\n"
            "C++: params[key]  (throws if not found)\n\n"
            "  p = ps['goal_bias']\n"
            "  print(p.getValue())      # '0.050000'\n"
            "  p.setValue('0.1')        # set to 0.1\n"
            "  print(p.getRangeSuggestion())")

        .def("size", &ps_size,
            "Return the number of declared parameters.\n\n"
            "C++: params.size()\n\n"
            "  print(ps.size())   # e.g. 3")

        .def("__len__", &ps_size,
            "Return the number of declared parameters.\n\n"
            "  len(ps)   # same as ps.size()")

        .def("clear", &ps_clear,
            "Remove all declared parameters.\n\n"
            "C++: params.clear()")

        .def("remove", &ps_remove, py::arg("name"),
            "Remove the parameter with the given name.\n\n"
            "C++: params.remove(name)\n\n"
            "No-op if the name is not found.")

        .def("include", &ps_include,
            py::arg("other"), py::arg("prefix") = "",
            "Merge all parameters from 'other' into this ParamSet.\n\n"
            "C++: params.include(other, prefix)\n\n"
            "If prefix is non-empty, each param name is prefixed:\n"
            "  ps1.include(ps2, 'planner.')\n"
            "  # adds 'planner.goal_bias', 'planner.range', etc.\n\n"
            "Used by SimpleSetup to expose all params together.")

        .def("print", &ps_print_str,
            "Return a formatted string of all parameters.\n\n"
            "C++: params.print(std::cout)\n\n"
            "  print(ps.print())\n"
            "  # goal_bias = 0.050000\n"
            "  # range = 0.000000")

        .def("__repr__", &ps_repr);
}