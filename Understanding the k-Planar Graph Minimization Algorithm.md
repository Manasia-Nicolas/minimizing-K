
## Executive Summary

This C++ program implements an advanced heuristic algorithm designed to compute **k-planar graph layouts** with minimized edge crossings. A k-planar graph is one where each edge can participate in at most k crossings. The algorithm uses multiple complementary heuristics to assign graph vertices to grid positions, progressively refining solutions through stress minimization and greedy vertex swapping to achieve visually cleaner, less-congested graph drawings.

---

## 1. Foundational Concepts

### 1.1 What is a k-Planar Graph?

A **k-planar graph** is a graph that admits a drawing in the plane where each edge participates in at most k crossings (intersections with other edges). This generalizes the classical notion of planar graphs:

- A **0-planar graph** (or simply planar graph) has zero edge crossings
- A **1-planar graph** allows each edge to cross at most once
- A **2-planar graph** allows each edge to cross at most twice
- And so on...

**Why does this matter?** In many real-world applications (VLSI circuit design, network visualization, social network analysis), graphs often cannot be drawn without crossings. Understanding k-planar graphs helps us quantify the minimum number of unavoidable crossings.

### 1.2 Edge Density and the k-Planar Bound

The program uses a fundamental theorem from graph theory relating the number of edges to the degree of planarity:

For a k-planar graph with V vertices and E edges, the following relationship holds:

```
E ≤ (k+3) × (V - 2)
```

This bound is derived from Euler's formula and generalizations of planarity constraints. The algorithm implements this check in the function `k_small()`:

```cpp
int k_small(long long V, long long E) {
    for(int k = 0; k <= 4; k++) {
        if(E <= (k+3)*(V-2))
            return k;
    }
    return 1000;
}
```

The function iterates through k values (0 through 4) and returns the **minimum k** for which the graph satisfies the k-planar bound. This tells us the "thickness" of the graph—how many layers would be needed if we could decompose it into planar subgraphs.

---

## 2. Core Algorithm Architecture

### 2.1 Multi-Heuristic Approach

The algorithm doesn't rely on a single solution method. Instead, it implements **four complementary heuristics**, each with different strengths:

1. **Spiral Assignment** - Deterministic grid-based ordering
2. **Degree-Greedy Assignment** - Heuristic prioritizing high-degree vertices
3. **Barycentric Assignment** - Force-directed approach using neighbor averages
4. **Distance Refinement** - Iterative local optimization minimizing edge length stress

This multi-heuristic approach is used because graph layout optimization is NP-hard no single polynomial-time algorithm guarantees optimal solutions. By computing multiple solutions in parallel, the algorithm increases the probability of finding a high-quality layout.

### 2.2 Grid-Based Vertex Positioning

All heuristics operate on an **r × r grid** where vertices are assigned to grid positions:

```cpp
long long r = (long long)ceil(sqrt((double)V));
```

The grid size is computed as the ceiling of the square root of the vertex count, ensuring the grid has enough positions for all vertices (at most r² ≥ V positions).

Grid coordinates are perturbed slightly to avoid degenerate configurations:

```cpp
uniform_real_distribution<double> d(-perturb, perturb);  // perturb = 0.15
double x = (double)j + d(rng);
double y = (double)i + d(rng);
```

This **perturbation** (±0.15 units) prevents perfectly aligned edges, which could artificially reduce or inflate crossing counts.

---

## 3. Four Heuristic Methods

### 3.1 Spiral Assignment

**Concept:** Order vertices in a spiral pattern starting from the grid center and spiraling outward.

**Implementation:**
```cpp
vector<int> spiralOrder(int r) {
    vector<int> ord;
    int top = 0, bottom = r-1;
    int left = 0, right = r-1;
    
    while (top <= bottom && left <= right) {
        // left to right (top row)
        for(int j = left; j <= right; j++)
            ord.push_back(top * r + j);
        top++;
        // ... continue spiraling inward ...
    }
    return ord;
}
```

**Advantages:**
- Simple, deterministic, no randomness
- Naturally clusters related vertices (spiral properties encourage locality)
- Serves as a baseline for comparison

**Disadvantages:**
- Ignores graph structure entirely
- May create poor layouts for complex topology

### 3.2 Degree-Greedy Assignment

**Concept:** Prioritize vertices with high degree (many neighbors) by assigning them to positions first.

**Mathematical Intuition:** High-degree vertices should be placed strategically because their many incident edges are likely to cause crossings. By placing them first in unconstrained positions, we minimize interference.

**Implementation:**
```cpp
vector<int> degree_greedy_assignment(int V, const vector<vector<int>>& adj, 
                                     const vector<int>& position_order) {
    vector<int> deg(V);
    for (int i = 0; i < V; i++)
        deg[i] = adj[i].size();
    
    vector<int> order(V);
    for (int i = 0; i < V; i++) order[i] = i;
    
    sort(order.begin(), order.end(),
         [&](int a, int b){ return deg[a] > deg[b]; });  // Sort descending by degree
}
```

**Advantages:**
- Respects graph structure
- Empirically performs well on many graph classes

**Disadvantages:**
- Greedy approach can lead to suboptimal later assignments
- Doesn't optimize for edge crossings directly

### 3.3 Barycentric Assignment

**Concept:** For each vertex, place it near the **center of mass** (barycenter) of its already-placed neighbors.

**Mathematical Foundation:** Barycentric coordinates date back to Möbius (19th century) and form the basis of Tutte embeddings. The principle is that a vertex should be attracted toward the weighted average position of its neighbors.

**Implementation:**
```cpp
for (int v = 0; v < V; v++) {
    // Find placed neighbors
    vector<int> placed;
    for (int u : adj[v])
        if (assignment[u] != -1)
            placed.push_back(u);
    
    if (!placed.empty()) {
        double avg = 0;
        for (int u : placed)
            avg += assignment[u];
        avg /= placed.size();  // Compute average (barycenter)
        
        // Find nearest unassigned position to barycenter
        double best = 1e18;
        for (int idx : position_order) {
            if (!used[idx]) {
                double d = fabs(idx - avg);
                if (d < best) {
                    best = d;
                    chosen = idx;
                }
            }
        }
    }
}
```

**Advantages:**
- Theoretically grounded in established graph drawing theory
- Creates drawings that keep connected vertices relatively close
- Reduces edge length variance

**Disadvantages:**
- Depends on assignment order (somewhat sequential)
- Local optimization can miss global structure

### 3.4 Distance Refinement (Novel 4th Heuristic)

**Concept:** Starting from an initial assignment (barycentric), iteratively improve by swapping vertices to reduce edge stress.

**Key Innovation:** This heuristic implements a **local search** with a **stress function**:

#### Stress Function Definition

For a vertex u, the **vertex stress** is the sum of all edge lengths incident to it:

```cpp
double get_vertex_stress(int u, const vector<vector<int>>& adj, 
                         const vector<int>& assignment, 
                         const vector<pair<double, double>>& coords) {
    double stress = 0.0;
    int u_pos = assignment[u];
    for (int v : adj[u]) {
        if (assignment[v] != -1) {
            stress += get_dist(u_pos, assignment[v], coords);
        }
    }
    return stress;
}
```

Where distance is Euclidean:
```
distance(pos1, pos2) = √[(x₁ - x₂)² + (y₁ - y₂)²]
```

#### Algorithm Steps

1. **Identify "Bad Edges"**: Find all edges longer than a target distance d

2. **Target Distance Calculation**:
```cpp
double target_d = sqrt((2.0 * E) / (M_PI * V));
```

This formula is derived from optimal packing theory:
- For a graph with E edges and V vertices in a 2D plane, the optimal average edge length is proportional to √(E/V)
- The coefficient √(2/π) ≈ 0.798 comes from circle packing theory

3. **Greedy Neighborhood Search**: For each bad edge (u,v):
   - Extract vertex u and position v_pos_idx
   - Search grid positions in a radius around v_pos_idx
   - For each candidate position p_c, find which vertex w currently occupies it
   - Compute **gain** = (stress_before_swap - stress_after_swap)
   - Execute the swap if gain > 0

4. **Iteration**: Repeat until no bad edges remain or max iterations (2500) exceeded

```cpp
for(int iter=0; iter<max_iterations; ++iter) {
    vector<pair<int, int>> bad_edges;
    for(auto& e : edges) {
        if(get_dist(A[e.first], A[e.second], coords) > target_d) {
            bad_edges.push_back(e);
        }
    }
    if(bad_edges.empty()) break;  // Converged!
    
    // ... perform swaps to improve layout ...
}
```

**Advantages:**
- Directly minimizes edge crossing proxy (edge length)
- Guaranteed monotonic improvement (if gains are positive)
- Empirically very effective after initial placement
- Convergence in finite time

**Disadvantages:**
- Requires initial good solution
- Local minima can trap the algorithm
- O(iterations × bad_edges × neighborhood_size) complexity

---

## 4. Mathematical Insights

### 4.1 Why Target Distance?

The target distance formula represents the **expected average edge length** for a uniformly distributed graph in 2D space:

```
d = √(2E / πV)
```

**Derivation intuition:**
- Total edge length should be roughly proportional to graph size: O(E × d_avg) ≈ constant
- In a 2D plane, achievable density follows from circle packing: area ∝ r²
- Optimal packing yields coefficient √(2/π)

### 4.2 Complexity Analysis

| Heuristic | Time Complexity | Space |
|-----------|-----------------|-------|
| Spiral | O(V) | O(V) |
| Degree-Greedy | O(V log V + E) | O(V) |
| Barycentric | O(V × E + V × r²) | O(V + r²) |
| Distance Refinement | O(I × B × N²) | O(V + E + r²) |

Where:
- I = max iterations (2500)
- B = average bad edges
- N = neighborhood radius (≈√d)

### 4.3 Convergence Guarantees

**Spiral/Degree-Greedy/Barycentric:** Deterministic, single-pass (no convergence guarantees on quality)

**Distance Refinement:** 
- Monotonic: stress cannot increase
- Terminates: either bad_edges = ∅ or max_iterations reached
- No optimality guarantee: local minima possible

---

## 5. Why Multiple Heuristics?

The program implements all four heuristics specifically because:

1. **Diversity**: Each captures different aspects of graph structure
2. **Robustness**: Different graph types favor different methods
3. **Combination**: Users can select best result post-computation
4. **Theoretical Appeal**: Illustrates multiple solution paradigms

No single heuristic dominates all graph classes—this is a fundamental result in graph drawing research.

---

## 6. Algorithm Flow Summary

```
Input: Graph G = (V, E) from file

Step 1: Read graph from "graph_input.in"
Step 2: Compute k-planar bound using k_small()
Step 3: Create r×r grid with perturbed coordinates
Step 4: Run Heuristic 1 (Spiral)
Step 5: Run Heuristic 2 (Degree-Greedy)
Step 6: Run Heuristic 3 (Barycentric)
Step 7: Run Heuristic 4 (Distance Refinement) using Heuristic 3 output
Step 8: Write all assignments to "graph_output.out"

Output: Grid coordinates and four distinct vertex assignments
```

---

## 7. Practical Applications

### 7.1 Software Engineering
Dependency graphs with better layouts improve code architecture comprehension.
### 7.2 Network Visualization
Fewer edge crossings make network diagrams (social networks, computer networks) more readable.

### 7.3 Bioinformatics
Protein interaction networks with minimized crossings help reveal functional relationships.

---

## 8. Conclusion

This algorithm represents a **sophisticated approach to NP-hard graph drawing** through:

- **Theoretical grounding** in Euler's formula, barycentric coordinates, and circle packing
- **Multiple complementary heuristics** trading quality vs. computation
- **Iterative refinement** using stress-based local search
- **Grid-based representation** enabling discrete optimization

The k-planar framework quantifies the trade-off between visual clarity (fewer crossings) and layout complexity, making it valuable for both theoretical graph theory and practical visualization systems.