import math
import pyompl
# import pyompl.datastructures as datastructures   # convenience alias

def check(condition, msg):
    print(f"  {'PASS' if condition else 'FAIL'} — {msg}")

def approx(a, b, tol=1e-6):
    return abs(a - b) < tol


# ==============================================================
# PART 1: AdjacencyList (ompl/datastructures/AdjacencyList.h)
# ==============================================================
print("=" * 60)
print("PART 1: AdjacencyList")
print("=" * 60)
print("""
WHAT IS AdjacencyList?
------------------------
An undirected, weighted graph with connected-component tracking
and Dijkstra shortest-path search. Vertices cannot be removed
(only added), which keeps vertex ids stable.

C++ class:  ompl::AdjacencyList
Python:     pyompl.datastructures.AdjacencyList
""")

# -- Constructors --
print("-- Constructors --")
g_empty = pyompl.datastructures.AdjacencyList()
g5 = pyompl.datastructures.AdjacencyList(5)
print(f"  AdjacencyList() -> numVertices = {g_empty.numVertices()}")
print(f"  AdjacencyList(5) -> numVertices = {g5.numVertices()}")
check(g_empty.numVertices() == 0, "empty constructor has 0 vertices")
check(g5.numVertices() == 5, "AdjacencyList(5) has 5 vertices")

# -- addVertex / numVertices / vertexExists --
print("\n-- addVertex() / numVertices() / vertexExists() --")
v0 = g_empty.addVertex()
v1 = g_empty.addVertex()
v2 = g_empty.addVertex()
print(f"  Added vertices: {v0}, {v1}, {v2}")
print(f"  numVertices() = {g_empty.numVertices()}")
check(g_empty.numVertices() == 3, "3 vertices added")
check(g_empty.vertexExists(v0), "vertex 0 exists")
check(not g_empty.vertexExists(99), "vertex 99 does not exist")

# -- addEdge / edgeExists / numEdges --
print("\n-- addEdge() / edgeExists() / numEdges() --")
ok1 = g_empty.addEdge(v0, v1, 2.5)
ok2 = g_empty.addEdge(v1, v2)            # default weight = 1.0
dup = g_empty.addEdge(v0, v1, 9.0)       # duplicate edge -> should fail
print(f"  addEdge(v0, v1, 2.5) -> {ok1}")
print(f"  addEdge(v1, v2)      -> {ok2} (default weight)")
print(f"  addEdge(v0, v1) again -> {dup} (duplicate, should be False)")
check(ok1 is True, "first edge added")
check(ok2 is True, "second edge added")
check(dup is False, "duplicate edge rejected")
check(g_empty.numEdges() == 2, "2 edges total")
check(g_empty.edgeExists(v0, v1), "edge v0-v1 exists")
check(not g_empty.edgeExists(v0, v2), "edge v0-v2 does not exist (no direct edge)")

# -- getEdgeWeight / setEdgeWeight --
print("\n-- getEdgeWeight() / setEdgeWeight() --")
w = g_empty.getEdgeWeight(v0, v1)
print(f"  getEdgeWeight(v0, v1) = {w}")
check(approx(w, 2.5), "edge weight matches what was set")
g_empty.setEdgeWeight(v0, v1, 7.0)
w2 = g_empty.getEdgeWeight(v0, v1)
print(f"  after setEdgeWeight(v0, v1, 7.0) -> {w2}")
check(approx(w2, 7.0), "edge weight updated")

# -- numNeighbors / getNeighbors / getNeighborsWeighted --
print("\n-- numNeighbors() / getNeighbors() / getNeighborsWeighted() --")
n = g_empty.numNeighbors(v1)
nbrs = g_empty.getNeighbors(v1)
nbrs_w = g_empty.getNeighborsWeighted(v1)
print(f"  numNeighbors(v1) = {n}")
print(f"  getNeighbors(v1) = {nbrs}")
print(f"  getNeighborsWeighted(v1) = {nbrs_w}")
check(n == 2, "v1 has 2 neighbors (v0 and v2)")
check(sorted(nbrs) == sorted([v0, v2]), "neighbor ids match")
check(all(isinstance(pair, tuple) and len(pair) == 2 for pair in nbrs_w),
      "weighted neighbors are (id, weight) pairs")

# -- inSameComponent / numConnectedComponents / getComponentID --
print("\n-- inSameComponent() / numConnectedComponents() / getComponentID() --")
print(f"  inSameComponent(v0, v2) = {g_empty.inSameComponent(v0, v2)}")
print(f"  numConnectedComponents() = {g_empty.numConnectedComponents()}")
print(f"  getComponentID(v0) = {g_empty.getComponentID(v0)}")
check(g_empty.inSameComponent(v0, v2), "v0 and v2 connected via v1")
check(g_empty.numConnectedComponents() == 1, "one connected component")

# isolated vertex -> separate component
v3 = g_empty.addVertex()
print(f"  Added isolated vertex v3={v3}")
print(f"  numConnectedComponents() now = {g_empty.numConnectedComponents()}")
check(g_empty.numConnectedComponents() == 2, "isolated vertex forms its own component")
check(not g_empty.inSameComponent(v0, v3), "v0 and v3 are not connected")

# -- dijkstra (v1, v2) form --
print("\n-- dijkstra(v1, v2) — shortest path between two vertices --")
found, path = g_empty.dijkstra(v0, v2)
print(f"  dijkstra(v0, v2) -> found={found}, path={path}")
check(found is True, "path found between v0 and v2")
check(path[0] == v0 and path[-1] == v2, "path starts at v0 and ends at v2")

found_none, path_none = g_empty.dijkstra(v0, v3)
print(f"  dijkstra(v0, v3) -> found={found_none}, path={path_none} (no connection)")
check(found_none is False, "no path between disconnected vertices")

# -- dijkstraAll(vtx) form --
print("\n-- dijkstraAll(vtx) — shortest paths to every vertex --")
predecessors, distance = g_empty.dijkstraAll(v0)
print(f"  dijkstraAll(v0) -> predecessors={predecessors}, distance={distance}")
check(len(predecessors) == g_empty.numVertices(), "predecessors list covers all vertices")
check(len(distance) == g_empty.numVertices(), "distance list covers all vertices")
check(distance[v0] == 0.0, "distance to self is 0")
check(math.isinf(distance[v3]), "distance to unreachable vertex is infinite")

# -- removeEdge --
print("\n-- removeEdge() --")
removed = g_empty.removeEdge(v1, v2)
removed_again = g_empty.removeEdge(v1, v2)   # already gone -> False
print(f"  removeEdge(v1, v2) -> {removed}")
print(f"  removeEdge(v1, v2) again -> {removed_again} (should be False)")
check(removed is True, "edge removed successfully")
check(removed_again is False, "removing a non-existent edge returns False")
check(not g_empty.edgeExists(v1, v2), "edge v1-v2 no longer exists")

# -- clear --
print("\n-- clear() --")
g_empty.clear()
print(f"  after clear(): numVertices={g_empty.numVertices()}, numEdges={g_empty.numEdges()}")
check(g_empty.numVertices() == 0, "all vertices removed")
check(g_empty.numEdges() == 0, "all edges removed")


# ==============================================================
# PART 2: Permutation (ompl/datastructures/Permutation.h)
# ==============================================================
print("\n" + "=" * 60)
print("PART 2: Permutation")
print("=" * 60)
print("""
WHAT IS Permutation?
----------------------
A shuffled permutation of the indices 0..n-1. Derives from
std::vector<int> in C++; exposed here with manual container
protocol (len / index / iterate) since pybind11 does not
auto-inherit std::vector bindings.

C++ class:  ompl::Permutation
Python:     pyompl.datastructures.Permutation
""")

# -- Constructor --
print("-- Constructor --")
perm = pyompl.datastructures.Permutation(10)
print(f"  Permutation(10) -> len = {len(perm)}")
check(len(perm) == 10, "permutation has 10 elements")

# -- Contents are a permutation of 0..n-1 --
print("\n-- Contents check --")
values = [perm[i] for i in range(len(perm))]
print(f"  values: {values}")
check(sorted(values) == list(range(10)), "values are a permutation of 0..9")

# -- __iter__ --
print("\n-- __iter__() --")
iterated = list(perm)
print(f"  list(perm) = {iterated}")
check(iterated == values, "iteration matches indexed access")

# -- __setitem__ --
print("\n-- __setitem__() --")
perm[0] = 42
print(f"  perm[0] = 42 -> perm[0] now = {perm[0]}")
check(perm[0] == 42, "value written via __setitem__")

# -- __getitem__ out-of-range --
print("\n-- __getitem__() out-of-range --")
try:
    _ = perm[999]
    check(False, "IndexError expected for out-of-range access")
except IndexError:
    print("  IndexError raised as expected for perm[999]")
    check(True, "IndexError correctly raised")

# -- permute() re-shuffle --
print("\n-- permute() — re-shuffle --")
perm.permute(10)
values_after = [perm[i] for i in range(len(perm))]
print(f"  after permute(10): {values_after}")
check(sorted(values_after) == list(range(10)), "still a valid permutation of 0..9 after reshuffle")

# -- permute() to a different (larger) size --
print("\n-- permute() with larger n --")
perm.permute(15)
print(f"  after permute(15): len = {len(perm)}")
check(len(perm) >= 15, "permutation grows to accommodate n=15")
values_15 = [perm[i] for i in range(15)]
check(sorted(values_15) == list(range(15)), "first 15 entries are a permutation of 0..14")


# ==============================================================
# PART 3: DynamicSSSP (ompl/datastructures/DynamicSSSP.h)
# ==============================================================
print("\n" + "=" * 60)
print("PART 3: DynamicSSSP")
print("=" * 60)
print("""
WHAT IS DynamicSSSP?
-----------------------
An incrementally-updated single-source-shortest-path structure.
Vertex 0 is the implicit source (distance 0); all other vertices
start at infinity until connected. Adding/removing edges updates
distances incrementally rather than recomputing from scratch.

C++ class:  ompl::DynamicSSSP
Python:     pyompl.datastructures.DynamicSSSP

NOTE: Assumes no two paths share the same cost (valid when edge
weights have some randomness, as in sampling-based planners).
""")

# -- Constructor / addVertex --
print("-- Constructor / addVertex() --")
sssp = pyompl.datastructures.DynamicSSSP()
for i in range(4):
    sssp.addVertex(i)
print("  Added vertices 0, 1, 2, 3")
print(f"  getShortestPathCost(0) = {sssp.getShortestPathCost(0)} (source, expect 0)")
check(sssp.getShortestPathCost(0) == 0.0, "source vertex has cost 0")
print(f"  getShortestPathCost(1) = {sssp.getShortestPathCost(1)} (unconnected, expect inf)")
check(math.isinf(sssp.getShortestPathCost(1)), "unconnected vertex has infinite cost")

# -- addEdge without collecting affected vertices --
print("\n-- addEdge() with collect_vertices=False (default) --")
affected = sssp.addEdge(0, 1, 5.0)
print(f"  addEdge(0, 1, 5.0) -> affected = {affected} (expect empty list)")
check(affected == [], "no affected-vertices list returned when collect_vertices=False")
cost1 = sssp.getShortestPathCost(1)
print(f"  getShortestPathCost(1) after edge = {cost1}")
check(approx(cost1, 5.0), "cost to vertex 1 updated to 5.0")

# -- addEdge with collect_vertices=True --
print("\n-- addEdge() with collect_vertices=True --")
affected2 = sssp.addEdge(1, 2, 3.0, True)
print(f"  addEdge(1, 2, 3.0, collect_vertices=True) -> affected = {affected2}")
check(2 in affected2, "vertex 2's distance changed and was reported")
cost2 = sssp.getShortestPathCost(2)
print(f"  getShortestPathCost(2) = {cost2} (expect 5.0 + 3.0 = 8.0)")
check(approx(cost2, 8.0), "cost to vertex 2 is via vertex 1 (5 + 3)")

# -- getShortestPathParent --
print("\n-- getShortestPathParent() --")
parent2 = sssp.getShortestPathParent(2)
print(f"  getShortestPathParent(2) = {parent2} (expect 1)")
check(parent2 == 1, "vertex 2's parent on shortest-path tree is vertex 1")

# -- Adding a shorter alternate path updates distances --
print("\n-- addEdge() introducing a shorter path --")
sssp.addVertex(3) if False else None  # vertex 3 already added above
affected3 = sssp.addEdge(0, 2, 4.0, True)   # direct 0->2 shorter than 0->1->2 (8.0)
print(f"  addEdge(0, 2, 4.0, collect_vertices=True) -> affected = {affected3}")
cost2_new = sssp.getShortestPathCost(2)
print(f"  getShortestPathCost(2) now = {cost2_new} (expect 4.0, the shorter path)")
check(approx(cost2_new, 4.0), "shorter path correctly updates shortest-path cost")

# -- removeEdge with collect_vertices=False --
print("\n-- removeEdge() with collect_vertices=False (default) --")
removed_affected = sssp.removeEdge(0, 2)
print(f"  removeEdge(0, 2) -> affected = {removed_affected} (expect empty list)")
check(removed_affected == [], "no affected-vertices list returned when collect_vertices=False")
cost2_after_remove = sssp.getShortestPathCost(2)
print(f"  getShortestPathCost(2) after removing direct edge = {cost2_after_remove} (expect back to 8.0 via vertex 1)")
check(approx(cost2_after_remove, 8.0), "cost falls back to the remaining path (0->1->2)")

# -- removeEdge with collect_vertices=True --
print("\n-- removeEdge() with collect_vertices=True --")
removed_affected2 = sssp.removeEdge(1, 2, True)
print(f"  removeEdge(1, 2, collect_vertices=True) -> affected = {removed_affected2}")
check(2 in removed_affected2, "vertex 2 reported as affected after its only path is removed")
cost2_disconnected = sssp.getShortestPathCost(2)
print(f"  getShortestPathCost(2) now = {cost2_disconnected} (expect inf, fully disconnected)")
check(math.isinf(cost2_disconnected), "vertex 2 is unreachable after removing its only edge")

# -- clear() --
print("\n-- clear() --")
sssp.clear()
print(f"  after clear(): getShortestPathCost(0) = {sssp.getShortestPathCost(0)}")
# NOTE: clear() empties internal state; re-adding vertex 0 is required before
# further queries are meaningful. This just confirms clear() runs without error.
check(True, "clear() executed without raising")


print("\n" + "=" * 60)
print("All datastructures tests complete.")
print("=" * 60)