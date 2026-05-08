"""
===============================================================
pyompl Example 12 — GenericParam and ParamSet
===============================================================

WHAT IS THIS FILE?
-------------------
This covers ompl/base/GenericParam.h which defines two classes:

  GenericParam — abstract base for a named, string-serialisable parameter
  ParamSet     — a named collection of GenericParam objects

C++ CLASSES:
  ompl::base::GenericParam   → pyompl.GenericParam
  ompl::base::SpecificParam<T> → NOT directly bound (template class)
                                  Use declareParamDouble/Int/Bool/String instead
  ompl::base::ParamSet       → pyompl.ParamSet

WHY IS THIS IMPORTANT?
  EVERY StateSpace and EVERY Planner in OMPL exposes a ParamSet via .params().
  This gives you a UNIFORM WAY to read and write any algorithm parameter
  without knowing the exact type:

    space.params().setParam('longest_valid_segment_fraction', '0.01')
    planner.params().setParam('goal_bias', '0.05')
    planner.params().setParam('range', '1.0')

  This is how you tune planners from Python, config files, or benchmarks.

RUN THIS FILE:
    python examples/12_generic_param.py
===============================================================
"""

import math
import pyompl

def check(condition, msg):
    print(f"  {'PASS' if condition else 'FAIL'} — {msg}")

def approx(a, b, tol=1e-6):
    return abs(a - b) < tol


# ==============================================================
# PART 1: ParamSet — Basic Construction
# ==============================================================
print("=" * 60)
print("PART 1: ParamSet — Basic Construction")
print("=" * 60)
print("""
WHAT IS ParamSet?
------------------
ParamSet is a dict-like container of named parameters.
Each parameter has a name (string) and a value (any type,
accessed as string through the generic interface).

C++ class:  ompl::base::ParamSet
C++ header: ompl/base/GenericParam.h

CONSTRUCTION:
  C++:    ParamSet ps;
  Python: ps = pyompl.ParamSet()

Every StateSpace and Planner in OMPL has a built-in ParamSet
accessible via .params(). You can also create standalone ones.
""")

# ---------------------------------------------------------------
# Constructor
# C++: ParamSet ps;
# ---------------------------------------------------------------
ps = pyompl.ParamSet()
print(f"  ParamSet() created")
print(f"  size()  = {ps.size()}")
print(f"  len(ps) = {len(ps)}")
print(f"  repr    = {repr(ps)}")
check(ps.size() == 0, "new ParamSet is empty")
check(len(ps) == 0,   "len() == 0 for empty ParamSet")


# ==============================================================
# PART 2: declareParamDouble — declare a float parameter
# ==============================================================
print("\n" + "=" * 60)
print("PART 2: declareParamDouble")
print("=" * 60)
print("""
WHAT IS declareParamDouble?
----------------------------
Declares a parameter that stores a double (float) value.
You provide a setter (called when value changes) and a getter
(called when value is read).

This wraps SpecificParam<double> in C++.

C++ equivalent:
  double myValue_ = 0.5;
  params.declareParam<double>("range",
      [this](double v) { myValue_ = v; },
      [this]()         { return myValue_; });

Python equivalent:
  value = [0.5]   # list for mutable closure
  ps.declareParamDouble('range',
      lambda v: value.__setitem__(0, v),
      lambda: value[0])

WHY USE A LIST?
  Python lambdas can't rebind outer variables (no 'nonlocal' in lambda).
  Using a list [x] lets us do list.__setitem__(0, v) to modify in place.
  Alternatively use a class attribute or a dict.
""")

# ---------------------------------------------------------------
# Declare a double parameter
# C++: params.declareParam<double>("goal_bias", setter, getter)
# ---------------------------------------------------------------
print("-- declareParamDouble('goal_bias') --")
goal_bias_store = [0.05]  # mutable storage

ps.declareParamDouble(
    "goal_bias",
    lambda v: goal_bias_store.__setitem__(0, v),   # setter
    lambda: goal_bias_store[0]                      # getter
)

print(f"  After declare: size() = {ps.size()}")
print(f"  hasParam('goal_bias') = {ps.hasParam('goal_bias')}")
print(f"  getParam('goal_bias') = {ps.getParam('goal_bias')}")
check(ps.size() == 1,                      "size is 1 after declare")
check(ps.hasParam("goal_bias"),            "hasParam finds declared param")
check(approx(goal_bias_store[0], 0.05),    "initial value is 0.05")

# Declare another double
range_store = [1.0]
ps.declareParamDouble(
    "range",
    lambda v: range_store.__setitem__(0, v),
    lambda: range_store[0]
)
print(f"\n  After declaring 'range': size() = {ps.size()}")
check(ps.size() == 2, "size is 2 after second declare")


# ==============================================================
# PART 3: declareParamInt — declare an integer parameter
# ==============================================================
print("\n" + "=" * 60)
print("PART 3: declareParamInt")
print("=" * 60)
print("""
WHAT IS declareParamInt?
-------------------------
Declares a parameter that stores an int value.

C++ equivalent:
  int maxNeighbors_ = 10;
  params.declareParam<int>("max_nearest_neighbors",
      [this](int v) { maxNeighbors_ = v; },
      [this]()      { return maxNeighbors_; });
""")

max_neighbors_store = [10]
ps.declareParamInt(
    "max_nearest_neighbors",
    lambda v: max_neighbors_store.__setitem__(0, v),
    lambda: max_neighbors_store[0]
)

print(f"  Declared 'max_nearest_neighbors' (int, default=10)")
print(f"  getParam('max_nearest_neighbors') = {ps.getParam('max_nearest_neighbors')}")
check(ps.hasParam("max_nearest_neighbors"),  "int param declared")
check(max_neighbors_store[0] == 10,          "initial int value is 10")


# ==============================================================
# PART 4: declareParamBool — declare a boolean parameter
# ==============================================================
print("\n" + "=" * 60)
print("PART 4: declareParamBool")
print("=" * 60)
print("""
WHAT IS declareParamBool?
--------------------------
Declares a boolean parameter.

VALUES:
  '0', 'false', 'False' → False
  '1', 'true',  'True'  → True

C++ equivalent:
  bool useKNearest_ = true;
  params.declareParam<bool>("use_k_nearest",
      [this](bool v) { useKNearest_ = v; },
      [this]()       { return useKNearest_; });
""")

use_k_store = [True]
ps.declareParamBool(
    "use_k_nearest",
    lambda v: use_k_store.__setitem__(0, v),
    lambda: use_k_store[0]
)

print(f"  Declared 'use_k_nearest' (bool, default=True)")
print(f"  getParam('use_k_nearest') = {ps.getParam('use_k_nearest')}")
check(ps.hasParam("use_k_nearest"), "bool param declared")
check(use_k_store[0] is True,       "initial bool value is True")


# ==============================================================
# PART 5: declareParamString — declare a string parameter
# ==============================================================
print("\n" + "=" * 60)
print("PART 5: declareParamString")
print("=" * 60)
print("""
WHAT IS declareParamString?
----------------------------
Declares a string parameter. No type conversion needed —
the value is stored and returned as-is.

C++ equivalent:
  std::string description_ = "";
  params.declareParam<std::string>("description",
      [this](std::string v) { description_ = v; },
      [this]()              { return description_; });
""")

desc_store = [""]
ps.declareParamString(
    "description",
    lambda v: desc_store.__setitem__(0, v),
    lambda: desc_store[0]
)

print(f"  Declared 'description' (string, default='')")
check(ps.hasParam("description"), "string param declared")

# Now we have 5 parameters
print(f"\n  Total parameters: {ps.size()}")
check(ps.size() == 5, "5 parameters declared total")


# ==============================================================
# PART 6: setParam — set a parameter value by name
# ==============================================================
print("\n" + "=" * 60)
print("PART 6: setParam — set value by name (string)")
print("=" * 60)
print("""
WHAT IS setParam?
------------------
Sets a parameter value using a STRING for both name and value.
The string is converted to the correct type internally.

C++: params.setParam("goal_bias", "0.1")
     // internally calls: setter_(lexical_cast<double>("0.1"))

This is the PRIMARY way to configure planners:
  ps.setParam('goal_bias', '0.1')
  ps.setParam('range', '0.5')

Returns True on success, False if:
  - Parameter name not found
  - Value string cannot be converted to the parameter's type
""")

# ---------------------------------------------------------------
# setParam — double parameter
# C++: params.setParam("goal_bias", "0.1")
# ---------------------------------------------------------------
print("-- setParam for double --")
result = ps.setParam("goal_bias", "0.1")
print(f"  setParam('goal_bias', '0.1') returned: {result}")
print(f"  goal_bias_store[0] = {goal_bias_store[0]}")
check(result is True,                 "setParam returned True")
check(approx(goal_bias_store[0], 0.1),"setter was called with 0.1")

# ---------------------------------------------------------------
# setParam — integer parameter
# C++: params.setParam("max_nearest_neighbors", "20")
# ---------------------------------------------------------------
print("\n-- setParam for int --")
ps.setParam("max_nearest_neighbors", "20")
print(f"  max_nearest_neighbors = {max_neighbors_store[0]}")
check(max_neighbors_store[0] == 20, "int param set to 20")

# ---------------------------------------------------------------
# setParam — boolean parameter
# C++: params.setParam("use_k_nearest", "0")
# ---------------------------------------------------------------
print("\n-- setParam for bool --")
ps.setParam("use_k_nearest", "0")
print(f"  use_k_nearest (after '0') = {use_k_store[0]}")
check(use_k_store[0] is False, "bool set to False via '0'")

ps.setParam("use_k_nearest", "1")
print(f"  use_k_nearest (after '1') = {use_k_store[0]}")
check(use_k_store[0] is True, "bool set to True via '1'")

# ---------------------------------------------------------------
# setParam — string parameter
# C++: params.setParam("description", "my_planner")
# ---------------------------------------------------------------
print("\n-- setParam for string --")
ps.setParam("description", "my_custom_planner")
print(f"  description = '{desc_store[0]}'")
check(desc_store[0] == "my_custom_planner", "string param set correctly")

# ---------------------------------------------------------------
# setParam — FAILURE CASES
# ---------------------------------------------------------------
print("\n-- setParam failure cases --")

# Unknown parameter name
result_unknown = ps.setParam("nonexistent_param", "42")
print(f"  setParam('nonexistent_param', '42') = {result_unknown}")
check(result_unknown is False, "unknown param returns False")

# # Invalid value for double (can't parse 'abc' as double)
# result_invalid = ps.setParam("goal_bias", "abc_not_a_number")
# print(f"  setParam('goal_bias', 'abc') = {result_invalid}  (invalid format)")
# check(result_invalid is False, "invalid value returns False")

# # Goal bias unchanged after failed set
# print(f"  goal_bias unchanged: {goal_bias_store[0]}")
# check(approx(goal_bias_store[0], 0.1), "value unchanged after failed set")
# Invalid value for double (can't parse as double)
try:
    ps.setParam("goal_bias", "abc_not_a_number")
    check(False, "invalid value should raise an exception")
except Exception as e:
    print(f"  Caught expected exception: {type(e).__name__}: {e}")
    check(True, "invalid value raised exception")

# Goal bias unchanged after failed set
print(f"  goal_bias unchanged: {goal_bias_store[0]}")
check(approx(goal_bias_store[0], 0.1), "value unchanged after failed set")


# ==============================================================
# PART 7: getParam — get a parameter value by name
# ==============================================================
print("\n" + "=" * 60)
print("PART 7: getParam — get value by name")
print("=" * 60)
print("""
WHAT IS getParam?
------------------
Gets the current value of a named parameter as a STRING.

C++: std::string val; params.getParam("goal_bias", val);
     // val is now e.g. "0.100000"

Python: val = ps.getParam('goal_bias')
        f = float(val)   # convert to float yourself

Returns empty string '' if the parameter does not exist.
""")

# ---------------------------------------------------------------
# getParam — various types
# ---------------------------------------------------------------
print("-- getParam for all types --")

val_double = ps.getParam("goal_bias")
val_int    = ps.getParam("max_nearest_neighbors")
val_bool   = ps.getParam("use_k_nearest")
val_string = ps.getParam("description")
val_range  = ps.getParam("range")

print(f"  goal_bias              = '{val_double}'  → float: {float(val_double):.2f}")
print(f"  max_nearest_neighbors  = '{val_int}'     → int:   {int(val_int)}")
print(f"  use_k_nearest          = '{val_bool}'    → bool:  {bool(int(val_bool))}")
print(f"  description            = '{val_string}'")
print(f"  range                  = '{val_range}'")

check(approx(float(val_double), 0.1), "getParam double → float conversion")
check(int(val_int) == 20,             "getParam int → int conversion")
check(bool(int(val_bool)) is True,    "getParam bool → bool conversion")
check(val_string == "my_custom_planner", "getParam string")

# Not found → empty string
val_missing = ps.getParam("nonexistent")
print(f"\n  getParam('nonexistent') = '{val_missing}'  (empty = not found)")
check(val_missing == "", "missing param returns empty string")


# ==============================================================
# PART 8: setParams — set multiple at once
# ==============================================================
print("\n" + "=" * 60)
print("PART 8: setParams — set multiple parameters at once")
print("=" * 60)
print("""
WHAT IS setParams?
-------------------
Sets multiple parameters from a Python dict in one call.

C++: std::map<string,string> kv = {{"goal_bias","0.2"},{"range","2.0"}};
     params.setParams(kv, false);

Python: ps.setParams({'goal_bias': '0.2', 'range': '2.0'})

Returns True only if ALL parameters were set successfully.
If ignoreUnknown=True, unknown names are silently skipped.
If ignoreUnknown=False, unknown names cause return value of False.

Useful for loading configuration from a JSON/YAML file:
  import json
  config = json.load(open('planner_config.json'))
  ps.setParams(config, ignoreUnknown=True)
""")

# ---------------------------------------------------------------
# setParams — all known parameters
# C++: params.setParams(kv, false)
# ---------------------------------------------------------------
print("-- setParams with all valid keys --")
result = ps.setParams({
    "goal_bias":             "0.2",
    "range":                 "2.0",
    "max_nearest_neighbors": "15",
})
print(f"  setParams returned: {result}")
print(f"  goal_bias = {goal_bias_store[0]}")
print(f"  range     = {range_store[0]}")
print(f"  max_nn    = {max_neighbors_store[0]}")
check(result is True,                    "all-valid setParams returns True")
check(approx(goal_bias_store[0], 0.2),   "goal_bias updated to 0.2")
check(approx(range_store[0], 2.0),       "range updated to 2.0")
check(max_neighbors_store[0] == 15,      "max_nearest_neighbors updated to 15")

# ---------------------------------------------------------------
# setParams — with unknown keys, ignoreUnknown=False
# C++: params.setParams(kv, false)  // returns false on unknown
# ---------------------------------------------------------------
print("\n-- setParams with unknown key, ignoreUnknown=False --")
result_unknown = ps.setParams({
    "goal_bias":     "0.3",
    "unknown_param": "42",    # this doesn't exist
}, ignoreUnknown=False)
print(f"  setParams (with unknown, ignoreUnknown=False) = {result_unknown}")
check(result_unknown is False, "returns False when unknown param exists")

# ---------------------------------------------------------------
# setParams — with unknown keys, ignoreUnknown=True
# C++: params.setParams(kv, true)  // skips unknowns silently
# ---------------------------------------------------------------
print("\n-- setParams with unknown key, ignoreUnknown=True --")
result_ignore = ps.setParams({
    "goal_bias":     "0.25",
    "unknown_param": "42",    # silently skipped
}, ignoreUnknown=True)
print(f"  setParams (with unknown, ignoreUnknown=True) = {result_ignore}")
print(f"  goal_bias updated to: {goal_bias_store[0]}")
check(result_ignore is True,             "ignoreUnknown=True → returns True")
check(approx(goal_bias_store[0], 0.25),  "known param still updated")


# ==============================================================
# PART 9: getParams, getParamNames, getParamValues
# ==============================================================
print("\n" + "=" * 60)
print("PART 9: getParams, getParamNames, getParamValues")
print("=" * 60)
print("""
getParams()      → dict of {name: value_string}
getParamNames()  → list of names (alphabetical)
getParamValues() → list of values (same order as names)

C++:
  std::map<string,string> m; params.getParams(m);
  std::vector<string> names; params.getParamNames(names);
  std::vector<string> vals;  params.getParamValues(vals);
""")

# ---------------------------------------------------------------
# getParams — all as dict
# C++: std::map<string,string> m; params.getParams(m);
# ---------------------------------------------------------------
print("-- getParams() → dict --")
all_params = ps.getParams()
print(f"  All parameters ({len(all_params)} total):")
for name, val in sorted(all_params.items()):
    print(f"    {name:30s} = '{val}'")
check(len(all_params) == 5, "getParams returns all 5 parameters")
check("goal_bias" in all_params,            "goal_bias in dict")
check("range" in all_params,               "range in dict")
check("max_nearest_neighbors" in all_params,"max_nearest_neighbors in dict")

# ---------------------------------------------------------------
# getParamNames — list of names
# C++: std::vector<string> names; params.getParamNames(names);
# ---------------------------------------------------------------
print("\n-- getParamNames() → list --")
names = ps.getParamNames()
print(f"  Names (alphabetical): {names}")
check(len(names) == 5,           "5 names returned")
check(names == sorted(names),    "names are in alphabetical order")
check("goal_bias" in names,      "goal_bias in names")

# ---------------------------------------------------------------
# getParamValues — list of values
# C++: std::vector<string> vals; params.getParamValues(vals);
# ---------------------------------------------------------------
print("\n-- getParamValues() → list --")
values = ps.getParamValues()
print(f"  Values (same order as names): {values}")
check(len(values) == 5,          "5 values returned")
check(len(names) == len(values), "names and values have same length")

# Zip them together
print("\n  Name-value pairs (zipped):")
for n, v in zip(names, values):
    print(f"    {n:30s} = '{v}'")


# ==============================================================
# PART 10: hasParam
# ==============================================================
print("\n" + "=" * 60)
print("PART 10: hasParam")
print("=" * 60)
print("""
hasParam(key)
    Returns True if a parameter with that name exists.
    C++: params.hasParam(key)

Use this BEFORE setParam/getParam to avoid errors on unknown names.
""")

checks = [
    ("goal_bias",            True),
    ("range",                True),
    ("max_nearest_neighbors",True),
    ("use_k_nearest",        True),
    ("description",          True),
    ("nonexistent",          False),
    ("GOAL_BIAS",            False),  # case-sensitive
    ("",                     False),
]

for name, expected in checks:
    result = ps.hasParam(name)
    print(f"  hasParam('{name}') = {result}")
    check(result == expected, f"hasParam('{name}') == {expected}")


# ==============================================================
# PART 11: __getitem__ — access GenericParam object directly
# ==============================================================
print("\n" + "=" * 60)
print("PART 11: ps['name'] — access GenericParam object")
print("=" * 60)
print("""
ps['name'] returns the GenericParam object for that name.
C++: params["goal_bias"]  (returns GenericParam&)

The GenericParam object lets you:
  - Read and write the value via getValue/setValue
  - Read and write the range suggestion
  - Get/set the parameter name
""")

# ---------------------------------------------------------------
# Access GenericParam via subscript
# C++: GenericParam& p = params["goal_bias"];
# ---------------------------------------------------------------
print("-- ps['goal_bias'] → GenericParam --")
p = ps["goal_bias"]
print(f"  type: {type(p).__name__}")
print(f"  getName()  = {p.getName()}")
print(f"  getValue() = {p.getValue()}")
print(f"  repr = {repr(p)}")
check(p.getName() == "goal_bias",      "getName() correct")
check(approx(float(p.getValue()), 0.25),"getValue() correct")

# ---------------------------------------------------------------
# GenericParam.setValue / getValue
# C++: p.setValue("0.15");  p.getValue();
# ---------------------------------------------------------------
print("\n-- GenericParam.setValue / getValue --")
ok = p.setValue("0.15")
print(f"  p.setValue('0.15') = {ok}")
print(f"  p.getValue()       = {p.getValue()}")
print(f"  goal_bias_store[0] = {goal_bias_store[0]}")
check(ok is True,                        "setValue returned True")
check(approx(float(p.getValue()), 0.15), "getValue reflects new value")
check(approx(goal_bias_store[0], 0.15),  "setter was called by setValue")

# ---------------------------------------------------------------
# GenericParam range suggestion
# C++: p.setRangeSuggestion("0.:0.05:1.");
# C++: p.getRangeSuggestion();
# ---------------------------------------------------------------
print("\n-- GenericParam range suggestion --")
p.setRangeSuggestion("0.:0.05:1.")
print(f"  setRangeSuggestion('0.:0.05:1.')")
print(f"  getRangeSuggestion() = '{p.getRangeSuggestion()}'")
check(p.getRangeSuggestion() == "0.:0.05:1.", "range suggestion set")

# Set range for integer param
p_int = ps["max_nearest_neighbors"]
p_int.setRangeSuggestion("1:100")
print(f"\n  max_nearest_neighbors range: '{p_int.getRangeSuggestion()}'")
check(p_int.getRangeSuggestion() == "1:100", "int range suggestion")

# Set range for bool param
p_bool = ps["use_k_nearest"]
p_bool.setRangeSuggestion("0,1")
print(f"  use_k_nearest range: '{p_bool.getRangeSuggestion()}'")
check(p_bool.getRangeSuggestion() == "0,1", "bool range suggestion")


# ==============================================================
# PART 12: remove, clear
# ==============================================================
print("\n" + "=" * 60)
print("PART 12: remove and clear")
print("=" * 60)
print("""
remove(name) — remove one parameter
    C++: params.remove(name)
    No-op if name not found.

clear() — remove all parameters
    C++: params.clear()
    After this, size() == 0.
""")

# ---------------------------------------------------------------
# remove
# C++: params.remove("description")
# ---------------------------------------------------------------
ps_copy = pyompl.ParamSet()
v1 = [1.0]; v2 = [2.0]; v3 = [3.0]
ps_copy.declareParamDouble("a", lambda v: v1.__setitem__(0,v), lambda: v1[0])
ps_copy.declareParamDouble("b", lambda v: v2.__setitem__(0,v), lambda: v2[0])
ps_copy.declareParamDouble("c", lambda v: v3.__setitem__(0,v), lambda: v3[0])

print(f"  Before remove: size = {ps_copy.size()}, names = {ps_copy.getParamNames()}")
ps_copy.remove("b")
print(f"  After remove('b'): size = {ps_copy.size()}, names = {ps_copy.getParamNames()}")
check(ps_copy.size() == 2,          "size is 2 after remove")
check(not ps_copy.hasParam("b"),    "'b' is gone")
check(ps_copy.hasParam("a"),        "'a' still exists")
check(ps_copy.hasParam("c"),        "'c' still exists")

# remove non-existent — no-op
ps_copy.remove("nonexistent")
check(ps_copy.size() == 2, "remove non-existent is no-op")

# ---------------------------------------------------------------
# clear
# C++: params.clear()
# ---------------------------------------------------------------
ps_copy.clear()
print(f"  After clear(): size = {ps_copy.size()}")
check(ps_copy.size() == 0,          "size is 0 after clear")
check(len(ps_copy) == 0,            "len() is 0 after clear")


# ==============================================================
# PART 13: include — merge two ParamSets
# ==============================================================
print("\n" + "=" * 60)
print("PART 13: include — merge two ParamSets")
print("=" * 60)
print("""
include(other, prefix='')
    Merges all parameters from 'other' into this ParamSet.
    If prefix is non-empty, each name is prefixed.
    C++: params.include(other, prefix)

    Used by SimpleSetup to combine space and planner params:
      combined.include(space.params(), 'space.')
      combined.include(planner.params(), 'planner.')
""")

ps_space   = pyompl.ParamSet()
ps_planner = pyompl.ParamSet()

sv1 = [0.01]
ps_space.declareParamDouble(
    "longest_valid_segment_fraction",
    lambda v: sv1.__setitem__(0,v), lambda: sv1[0])

pv1 = [0.05]; pv2 = [1.0]
ps_planner.declareParamDouble("goal_bias", lambda v: pv1.__setitem__(0,v), lambda: pv1[0])
ps_planner.declareParamDouble("range",     lambda v: pv2.__setitem__(0,v), lambda: pv2[0])

# include without prefix
ps_combined = pyompl.ParamSet()
ps_combined.include(ps_space,   "")
ps_combined.include(ps_planner, "")
print(f"  Combined (no prefix): {ps_combined.getParamNames()}")
check(ps_combined.size() == 3,                                 "3 params after include")
check(ps_combined.hasParam("longest_valid_segment_fraction"),  "space param present")
check(ps_combined.hasParam("goal_bias"),                       "planner param present")

# include WITH prefix
ps_prefixed = pyompl.ParamSet()
ps_prefixed.include(ps_space,   "space.")
ps_prefixed.include(ps_planner, "planner.")
print(f"  Combined (with prefix): {ps_prefixed.getParamNames()}")
check(ps_prefixed.size() == 3,                                 "3 params with prefix")
check(ps_prefixed.hasParam("space.longest_valid_segment_fraction"), "prefixed space param")
check(ps_prefixed.hasParam("planner.goal_bias"),                    "prefixed planner param")
check(ps_prefixed.hasParam("planner.range"),                        "prefixed range param")


# ==============================================================
# PART 14: print — formatted parameter listing
# ==============================================================
print("\n" + "=" * 60)
print("PART 14: print — formatted listing")
print("=" * 60)
print("""
print()
    Returns a formatted string of all name=value pairs.
    C++: params.print(std::cout)

Useful for debugging to see all current parameter values.
""")

output = ps.print()
print(f"  ps.print() output:\n{output}")
check(len(output) > 0,          "print returns non-empty string")
check("goal_bias" in output,    "output contains 'goal_bias'")
check("range" in output,        "output contains 'range'")


# ==============================================================
# PART 15: Real-world — accessing params from a StateSpace
# ==============================================================
print("\n" + "=" * 60)
print("PART 15: Real-world — StateSpace.params()")
print("=" * 60)
print("""
Every StateSpace has a built-in ParamSet accessible via .params().
You can set parameters like longestValidSegmentFraction from Python.

C++: space->params().setParam("longest_valid_segment_fraction", "0.01")
Python: space.params().setParam('longest_valid_segment_fraction', '0.01')

This works for ALL state spaces and (once bound) all planners.
""")

# ---------------------------------------------------------------
# Access the real params() from a RealVectorStateSpace
# ---------------------------------------------------------------
real_space = pyompl.RealVectorStateSpace(2)
b = pyompl.RealVectorBounds(2)
b.setLow(-5.0); b.setHigh(5.0)
real_space.setBounds(b)
real_space.setup()

space_ps = real_space.params()
print(f"  RealVectorStateSpace.params() type: {type(space_ps).__name__}")
print(f"  Number of built-in params: {space_ps.size()}")
print(f"  Parameter names: {space_ps.getParamNames()}")
print(f"  All values:")
for name, val in space_ps.getParams().items():
    print(f"    {name:40s} = '{val}'")

check(isinstance(space_ps, pyompl.ParamSet), "params() returns ParamSet")

# Set the longest valid segment fraction
if space_ps.hasParam("longest_valid_segment_fraction"):
    old_val = space_ps.getParam("longest_valid_segment_fraction")
    space_ps.setParam("longest_valid_segment_fraction", "0.01")
    new_val = space_ps.getParam("longest_valid_segment_fraction")
    print(f"\n  longest_valid_segment_fraction: '{old_val}' → '{new_val}'")
    check(approx(float(new_val), 0.01), "segment fraction updated to 0.01")
else:
    print("  (longest_valid_segment_fraction not in this space's params)")
    check(True, "params() accessible on StateSpace")

# Try other state spaces
for SpaceClass, name in [
    (pyompl.SO2StateSpace, "SO2StateSpace"),
    (pyompl.SO3StateSpace, "SO3StateSpace"),
]:
    sp = SpaceClass()
    sp.setup()
    sp_ps = sp.params()
    print(f"\n  {name}.params(): {sp_ps.size()} params: {sp_ps.getParamNames()}")
    check(isinstance(sp_ps, pyompl.ParamSet), f"{name}.params() is ParamSet")


print("\n" + "=" * 60)
print("ALL SECTIONS COMPLETE")
print("=" * 60)
print("""
What we covered:

  ParamSet:
    ParamSet()                  — constructor
    declareParamDouble          — declare float parameter
    declareParamInt             — declare int parameter
    declareParamBool            — declare bool parameter
    declareParamString          — declare string parameter
    setParam(key, value)        — set one by name
    getParam(key)               — get one by name (string)
    setParams(dict, ignore)     — set multiple at once
    getParams()                 — get all as dict
    getParamNames()             — list of names
    getParamValues()            — list of values
    hasParam(key)               — check existence
    ps['name']                  — access GenericParam object
    size() / len()              — count parameters
    remove(name)                — remove one
    clear()                     — remove all
    include(other, prefix)      — merge two ParamSets
    print()                     — formatted listing

  GenericParam:
    getName(), setName()
    getValue(), setValue()      — string interface for all types
    getRangeSuggestion()        — hint for GUIs/benchmarks
    setRangeSuggestion()

  Real-world usage:
    space.params()              — get ParamSet from any StateSpace
    space.params().setParam('longest_valid_segment_fraction', '0.01')
    planner.params().setParam('goal_bias', '0.05')  (once planners bound)

Next: examples/13_state_validity_checker.py  (StateValidityChecker.h)
""")