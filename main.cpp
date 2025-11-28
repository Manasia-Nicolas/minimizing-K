#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <iomanip>
#include <fstream>
#include <algorithm>
#include <limits>

using namespace std;

// Fallback for M_PI if not defined
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

ifstream f("graph_input.in");
ofstream g("graph_output.out");

//------------------------------------------------------------
// Compute k = 0..4 (your provided formula)
//------------------------------------------------------------
int k_small(long long V, long long E) {
    for(int k = 0; k <= 4; k++) {
        if(E <= (k+3)*(V-2))
            return k;
    }
    return 1000;
}

//------------------------------------------------------------
// Build spiral order for an r x r grid of indices 0..r*r-1
//------------------------------------------------------------
vector<int> spiralOrder(int r) {
    vector<int> ord;
    ord.reserve(r*r);

    int top = 0, bottom = r-1;
    int left = 0, right = r-1;

    while (top <= bottom && left <= right) {
        // left to right (top row)
        for(int j = left; j <= right; j++)
            ord.push_back(top * r + j);
        top++;

        // top to bottom (right column)
        for(int i = top; i <= bottom; i++)
            ord.push_back(i * r + right);
        right--;

        if (top <= bottom) {
            // right to left (bottom row)
            for(int j = right; j >= left; j--)
                ord.push_back(bottom * r + j);
            bottom--;
        }

        if (left <= right) {
            // bottom to top (left column)
            for(int i = bottom; i >= top; i--)
                ord.push_back(i * r + left);
            left++;
        }
    }

    return ord;
}

//------------------------------------------------------------
// Greedy barycentric assignment
//------------------------------------------------------------
vector<int> barycentric_assignment(
        int V,
        const vector<vector<int>>& adj,
        const vector<int>& position_order)
{
    vector<int> assignment(V, -1);       // vertex -> grid index
    vector<bool> used(position_order.size(), false);

    auto next_free = [&]() -> int {
        for (int idx : position_order)
            if (!used[idx]) return idx;
        return -1;
    };

    for (int v = 0; v < V; v++) {
        // find placed neighbors
        vector<int> placed;
        for (int u : adj[v])
            if (assignment[u] != -1)
                placed.push_back(u);

        int chosen = -1;

        if (!placed.empty()) {
            double avg = 0;
            for (int u : placed)
                assignment[u] != -1 ? avg += assignment[u] : 0;
            avg /= placed.size();

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

        if (chosen == -1)
            chosen = next_free();

        assignment[v] = chosen;
        used[chosen] = true;
    }

    return assignment;
}

//------------------------------------------------------------
// Degree descending greedy assignment
//------------------------------------------------------------
vector<int> degree_greedy_assignment(
        int V,
        const vector<vector<int>>& adj,
        const vector<int>& position_order)
{
    vector<int> assignment(V, -1);
    vector<bool> used(position_order.size(), false);

    vector<int> deg(V);
    for (int i = 0; i < V; i++)
        deg[i] = adj[i].size();

    vector<int> order(V);
    for (int i = 0; i < V; i++) order[i] = i;

    sort(order.begin(), order.end(),
         [&](int a, int b){ return deg[a] > deg[b]; });

    int pos_ptr = 0;
    for (int v : order) {
        while (pos_ptr < (int)position_order.size() && used[position_order[pos_ptr]])
            pos_ptr++;
        int p = position_order[pos_ptr];
        assignment[v] = p;
        used[p] = true;
    }

    return assignment;
}

//------------------------------------------------------------
// Simple spiral assignment
//------------------------------------------------------------
vector<int> spiral_assignment(int V, const vector<int>& spiral) {
    vector<int> assignment(V);
    for (int v = 0; v < V; v++)
        assignment[v] = spiral[v];
    return assignment;
}

//------------------------------------------------------------
// --- START OF NEW 4TH HEURISTIC: DISTANCE REFINEMENT ---
//------------------------------------------------------------

double get_dist(int pos_idx1, int pos_idx2, const vector<pair<double, double>>& coords) {
    double dx = coords[pos_idx1].first - coords[pos_idx2].first;
    double dy = coords[pos_idx1].second - coords[pos_idx2].second;
    return sqrt(dx*dx + dy*dy);
}

// Calculates local "Stress" (sum of edge lengths) for a specific vertex u
double get_vertex_stress(int u, const vector<vector<int>>& adj, const vector<int>& assignment, const vector<pair<double, double>>& coords) {
    double stress = 0.0;
    int u_pos = assignment[u];
    for (int v : adj[u]) {
        if (assignment[v] != -1) {
            stress += get_dist(u_pos, assignment[v], coords);
        }
    }
    return stress;
}

// The new Heuristic Function
vector<int> distance_refinement_assignment(
    int V,
    const vector<vector<int>>& adj,
    const vector<int>& initial_assignment,
    const vector<pair<double, double>>& coords,
    double target_d,
    int r)
{
    // Start with the best previous assignment
    vector<int> A = initial_assignment;

    // Reverse Map: Position Index -> Vertex ID
    vector<int> pos_to_v(coords.size(), -1);
    for(int v=0; v<V; ++v) {
        if (A[v] < (int)pos_to_v.size()) {
            pos_to_v[A[v]] = v;
        }
    }

    // Collect all edges for easier iteration
    vector<pair<int, int>> edges;
    for(int u=0; u<V; ++u) {
        for(int v : adj[u]) {
            if(u < v) edges.push_back({u, v});
        }
    }

    int max_iterations = 2500;
    int neighborhood_radius = (int)ceil(target_d);

    // Add progress logging to avoid silence
    // cerr << "Starting Distance Refinement..." << endl;

    for(int iter=0; iter<max_iterations; ++iter) {
        // 1. Identify "Bad Edges" (Length > d)
        vector<pair<int, int>> bad_edges;
        for(auto& e : edges) {
            if(get_dist(A[e.first], A[e.second], coords) > target_d) {
                bad_edges.push_back(e);
            }
        }

        if(bad_edges.empty()) break; // Optimization achieved!

        // 2. Pick random bad edge (u, v)
        pair<int, int> bad = bad_edges[rand() % bad_edges.size()];
        int u = bad.first;
        int v = bad.second;

        int v_pos_idx = A[v];
        int v_grid_x = (int)coords[v_pos_idx].first;
        int v_grid_y = (int)coords[v_pos_idx].second;

        int best_w = -1;
        double best_gain = 0.0;

        // 3. Greedy Neighborhood Search
        // Check grid spots around v
        for(int dy = -neighborhood_radius; dy <= neighborhood_radius; ++dy) {
            for(int dx = -neighborhood_radius; dx <= neighborhood_radius; ++dx) {
                int nx = v_grid_x + dx;
                int ny = v_grid_y + dy;

                if(nx < 0 || nx >= r || ny < 0 || ny >= r) continue;

                int candidate_pos_idx = ny * r + nx;
                if(candidate_pos_idx >= (int)coords.size()) continue;

                int w = pos_to_v[candidate_pos_idx];

                // If w is -1 or same as u/v, skip
                if(w == -1 || w == u || w == v) continue;

                // --- Calculate Gain if we swap u and w ---
                double u_stress_before = get_vertex_stress(u, adj, A, coords);
                double w_stress_before = get_vertex_stress(w, adj, A, coords);

                // Actual swap logic using CURRENT assignments
                int p_u = A[u];
                int p_w = A[w];

                // Swap in assignment
                A[u] = p_w;
                A[w] = p_u;

                double u_stress_after = get_vertex_stress(u, adj, A, coords);
                double w_stress_after = get_vertex_stress(w, adj, A, coords);

                // Revert assignment for next iteration check
                A[u] = p_u;
                A[w] = p_w;

                double gain = (u_stress_before + w_stress_before) - (u_stress_after + w_stress_after);

                if(gain > best_gain) {
                    best_gain = gain;
                    best_w = w;
                }
            }
        }

        // 4. Execute Best Swap
        if(best_w != -1) {
            int p_u = A[u];
            int p_w = A[best_w];

            // Swap
            A[u] = p_w;
            A[best_w] = p_u;

            // Update reverse map
            pos_to_v[p_u] = best_w;
            pos_to_v[p_w] = u;
        }
    }
    return A;
}
//------------------------------------------------------------
// --- END OF NEW 4TH HEURISTIC ---
//------------------------------------------------------------

//------------------------------------------------------------
// Main
//------------------------------------------------------------
int main() {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    long long V, E;
    f >> V >> E;

    if (!f || V <= 0) {
        cerr << "Invalid input.\n";
        return 1;
    }

    vector<vector<int>> adj(V);
    for (long long i = 0; i < E; i++) {
        long long u, v;
        f >> u >> v;
        if (u >= 0 && u < V && v >= 0 && v < V) {
            adj[u].push_back(v);
            adj[v].push_back(u);
        }
    }

    double C = 4.108;
    double ratio = (double)E / (C * (double)V);
    long long k = (long long)ceil(ratio * ratio);

    long long r = (long long)ceil(sqrt((double)V));
    double perturb = 0.15;

    mt19937 rng(123456);
    uniform_real_distribution<double> d(-perturb, perturb);

    // FIX: coords must hold ALL grid positions (r*r)
    vector<pair<double,double>> coords;
    coords.reserve(r * r);

    for (long long i = 0; i < r; i++) {
        for (long long j = 0; j < r; j++) {
            double x = (double)j + d(rng);
            double y = (double)i + d(rng);
            coords.emplace_back(x, y);
        }
    }

    vector<int> spiral = spiralOrder(r);
    spiral.resize(V);

    vector<int> A_spiral      = spiral_assignment(V, spiral);
    vector<int> A_degree      = degree_greedy_assignment(V, adj, spiral);
    vector<int> A_barycentric = barycentric_assignment(V, adj, spiral);

    // 4th Heuristic Call
    double target_d = sqrt((2.0 * E) / (M_PI * V));
    vector<int> A_refined = distance_refinement_assignment(V, adj, A_barycentric, coords, target_d, (int)r);

    g << "Computed_k = " << k << "\n";
    g << "Grid size r = " << r << "\n";
    g << "Target Distance d = " << fixed << setprecision(3) << target_d << "\n\n";

    g << "--- Coordinates (Pi = (x,y)) ---\n";
    for (int i = 0; i < (int)coords.size(); i++)
        g << "P" << i << " = (" << fixed << setprecision(3)
          << coords[i].first << ", " << coords[i].second << ")\n";
    g << "\n";

    auto print_assignment = [&](const string& name,
                                const vector<int>& A)
    {
        g << "--- Assignment: " << name << " ---\n";
        for (int v = 0; v < V; v++)
            g << "v" << v << " -> P" << A[v] << "\n";
        g << "\n";
    };

    print_assignment("Spiral", A_spiral);
    print_assignment("Degree-Greedy", A_degree);
    print_assignment("Barycentric", A_barycentric);
    print_assignment("Distance-Refinement", A_refined);

    return 0;
}
