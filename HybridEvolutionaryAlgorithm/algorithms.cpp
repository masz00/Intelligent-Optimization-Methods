#include "algorithms.h"
#include <climits>
#include <algorithm>
#include <numeric>
#include <random>
#include <iostream>
#include <vector>
#include <queue>
#include <chrono>
#include <unordered_set>

using namespace std;

struct LSMove {
    enum Type { NONE, ADD, REMOVE, SWAP_EDGES } type = NONE;
    int n1 = -1, n2 = -1;
    int n1_next = -1, n2_next = -1;
    int u = -1;
    int delta = -INT_MAX;

    bool operator<(const LSMove& other) const {
        return delta < other.delta;
    }
};

inline int calc_2opt_delta(int a, int b, int c, int d, const vector<vector<int>>& dist) {
    return (dist[a][b] + dist[c][d]) - (dist[a][c] + dist[b][d]);
}

void reverse_segment(vector<int>& tour, int i1n, int i2) {
    int n = tour.size();
    if (i1n <= i2) {
        reverse(tour.begin() + i1n, tour.begin() + i2 + 1);
    } else {
        vector<int> tmp;
        for (int k = i1n; k < n; k++) tmp.push_back(tour[k]);
        for (int k = 0; k <= i2; k++) tmp.push_back(tour[k]);
        reverse(tmp.begin(), tmp.end());
        int cur = 0;
        for (int k = i1n; k < n; k++) tour[k] = tmp[cur++];
        for (int k = 0; k <= i2; k++) tour[k] = tmp[cur++];
    }
}

vector<int> phase2_remove(vector<int> cycle, const vector<vector<int>> &dist, const vector<PointData> &points)
{
    bool improvement_found = true;
    while (improvement_found && cycle.size() > 1)
    {
        improvement_found = false;
        int target_removal_idx = -1;
        int steepest_cost_delta = 0;
        int n_cyc = cycle.size();

        for (int i = 0; i < n_cyc; i++)
        {
            int node_u = cycle[(i + n_cyc - 1) % n_cyc];
            int node_v = cycle[i];
            int node_w = cycle[(i + 1) % n_cyc];

            int delta_cost = dist[node_u][node_w] - dist[node_u][node_v] - dist[node_v][node_w] + points[node_v].node_weight;

            if (n_cyc == 2) delta_cost = 0 - 2 * dist[node_u][node_v] + points[node_v].node_weight;

            if (delta_cost < steepest_cost_delta)
            {
                steepest_cost_delta = delta_cost;
                target_removal_idx = i;
            }
        }

        if (steepest_cost_delta < 0)
        {
            cycle.erase(cycle.begin() + target_removal_idx);
            improvement_found = true;
        }
    }
    return cycle;
}

vector<int> solve_random(int start_idx, const vector<vector<int>> &dist, const vector<PointData> &points)
{
    int max_nodes = points.size();
    int l_size = 1 + (rand() % max_nodes);
    vector<int> path(max_nodes);
    iota(path.begin(), path.end(), 0);
    swap(path[0], path[start_idx]);
    for (int i = max_nodes - 1; i > 1; i--) {
        int j = 1 + rand() % i;
        swap(path[i], path[j]);
    }
    path.resize(l_size);
    return path;
}

vector<int> solve_nn(int start_idx, bool use_weights, const vector<vector<int>> &dist, const vector<PointData> &points)
{
    int max_nodes = points.size();
    vector<bool> visited(max_nodes, false);
    vector<int> tour;
    tour.push_back(start_idx);
    visited[start_idx] = true;
    while (tour.size() < (size_t)max_nodes) {
        int curr = tour.back();
        int best_v = -1; int min_d = INT_MAX;
        for (int v = 0; v < max_nodes; v++) {
            if (visited[v]) continue;
            int d = dist[curr][v] - (use_weights ? points[v].node_weight : 0);
            if (d < min_d) { min_d = d; best_v = v; }
        }
        tour.push_back(best_v);
        visited[best_v] = true;
    }
    return tour;
}

vector<int> solve_greedy_cycle(int start_idx, bool use_weights, const vector<vector<int>> &dist, const vector<PointData> &points)
{
    int max_nodes = points.size();
    vector<bool> visited(max_nodes, false);
    vector<int> route;
    route.push_back(start_idx);
    visited[start_idx] = true;
    int scnd_node = -1; int min_first_edge = INT_MAX;
    for (int v = 0; v < max_nodes; v++) {
        if (visited[v]) continue;
        int e_val = dist[start_idx][v] - (use_weights ? points[v].node_weight : 0);
        if (e_val < min_first_edge) { min_first_edge = e_val; scnd_node = v; }
    }
    route.push_back(scnd_node);
    visited[scnd_node] = true;
    while (route.size() < (size_t)max_nodes) {
        int best_v = -1, best_pos = -1; int min_ins_cost = INT_MAX;
        for (int v = 0; v < max_nodes; v++) {
            if (visited[v]) continue;
            for (int i = 0; i < (int)route.size(); i++) {
                int n_a = route[i], n_b = route[(i+1)%route.size()];
                int c = dist[n_a][v] + dist[v][n_b] - dist[n_a][n_b] - (use_weights ? points[v].node_weight : 0);
                if (c < min_ins_cost) { min_ins_cost = c; best_v = v; best_pos = i; }
            }
        }
        route.insert(route.begin() + best_pos + 1, best_v);
        visited[best_v] = true;
    }
    return route;
}

vector<int> solve_regret(int start_idx, bool use_weights, double w1, double w2, const vector<vector<int>> &dist, const vector<PointData> &points)
{
    int max_nodes = points.size();
    vector<bool> visited(max_nodes, false);
    vector<int> route;
    route.reserve(max_nodes);
    route.push_back(start_idx);
    visited[start_idx] = true;
    int scnd_node = -1; int min_first_edge = INT_MAX;
    for (int v = 0; v < max_nodes; v++) {
        if (visited[v]) continue;
        int e_val = dist[start_idx][v] - (use_weights ? points[v].node_weight : 0);
        if (e_val < min_first_edge) { min_first_edge = e_val; scnd_node = v; }
    }
    route.push_back(scnd_node);
    visited[scnd_node] = true;
    while (route.size() < (size_t)max_nodes) {
        double top_score = -1e9; int r_cand = -1; int target_pos = -1;
        int r_len = route.size();
        for (int v = 0; v < max_nodes; v++) {
            if (visited[v]) continue;
            int lowest_val = INT_MAX, runner_up_val = INT_MAX, lowest_pos = 0;
            for (int i = 0; i < r_len; i++) {
                int n_a = route[i], n_b = route[(i + 1) % r_len];
                int c_val = dist[n_a][v] + dist[v][n_b];
                if (r_len >= 2) c_val -= dist[n_a][n_b];
                if (use_weights) c_val -= points[v].node_weight;
                if (c_val < lowest_val) { runner_up_val = lowest_val; lowest_val = c_val; lowest_pos = i; }
                else if (c_val < runner_up_val) runner_up_val = c_val;
            }
            double evaluated_score = w1 * (double)(runner_up_val - lowest_val) - w2 * (double)lowest_val;
            if (evaluated_score > top_score) { top_score = evaluated_score; r_cand = v; target_pos = lowest_pos; }
        }
        route.insert(route.begin() + target_pos + 1, r_cand);
        visited[r_cand] = true;
    }
    return route;
}


vector<int> solve_steepest_baseline(vector<int> tour, const vector<vector<int>> &dist, const vector<PointData> &points)
{
    bool improved = true;
    int n_all = points.size();
    while (improved) {
        improved = false;
        int n = tour.size();
        LSMove best;
        vector<bool> in_t(n_all, false); for (int v : tour) in_t[v] = true;
        for (int i = 0; i < n; i++) {
            int v = tour[i], w = tour[(i+1)%n];
            for (int u = 0; u < n_all; u++) {
                if (in_t[u]) continue;
                // Delta = NewObj - OldObj
                int d = points[u].node_weight - (dist[v][u] + dist[u][w] - dist[v][w]);
                if (d > best.delta) best = { LSMove::ADD, v, w, -1, -1, u, d };
            }
            if (n > 3) {
                int u_r = tour[i], v_p = tour[(i - 1 + n) % n], w_n = tour[(i + 1) % n];
                int d = -points[u_r].node_weight + (dist[v_p][u_r] + dist[u_r][w_n] - dist[v_p][w_n]);
                if (d > best.delta) best = { LSMove::REMOVE, u_r, -1, -1, -1, -1, d };
            }
        }
        for (int i = 0; i < n; i++) {
            for (int j = i + 2; j < n; j++) {
                if (i == 0 && j == n - 1) continue;
                int d = calc_2opt_delta(tour[i], tour[(i+1)%n], tour[j], tour[(j+1)%n], dist);
                if (d > best.delta) best = { LSMove::SWAP_EDGES, tour[i], tour[j], tour[(i+1)%n], tour[(j+1)%n], -1, d };
            }
        }
        if (best.delta > 0) {
            improved = true;
            if (best.type == LSMove::ADD) {
                for (int i = 0; i < (int)tour.size(); i++) if (tour[i] == best.n1 && tour[(i+1)%tour.size()] == best.n2) { tour.insert(tour.begin() + i + 1, best.u); break; }
            } else if (best.type == LSMove::REMOVE) {
                for (int i = 0; i < (int)tour.size(); i++) if (tour[i] == best.n1) { tour.erase(tour.begin() + i); break; }
            } else if (best.type == LSMove::SWAP_EDGES) {
                int i1n = -1, i2 = -1; for(int k=0; k<(int)tour.size(); k++) { if(tour[k] == best.n1_next) i1n = k; if(tour[k] == best.n2) i2 = k; }
                reverse_segment(tour, i1n, i2);
            }
        }
    }
    return tour;
}

vector<int> solve_steepest_lm(vector<int> tour, const vector<vector<int>> &dist, const vector<PointData> &points)
{
    int n_all = points.size();
    vector<bool> in_t(n_all, false);
    vector<int> pos(n_all, -1);
    for (int i = 0; i < (int)tour.size(); i++) { in_t[tour[i]] = true; pos[tour[i]] = i; }

    auto cmp = [](const LSMove& a, const LSMove& b) { return a.delta < b.delta; }; // Max-heap
    priority_queue<LSMove, vector<LSMove>, decltype(cmp)> PQ(cmp);

    auto reindex = [&](int start) { int n = (int)tour.size(); for (int i = start; i < n; i++) pos[tour[i]] = i; };
    auto reindex_segment = [&](int lo, int hi) {
        int n = (int)tour.size();
        if (lo <= hi) { for (int i = lo; i <= hi; i++) pos[tour[i]] = i; }
        else { for (int i = lo; i < n; i++) pos[tour[i]] = i; for (int i = 0; i <= hi; i++) pos[tour[i]] = i; }
    };
    auto edge_ok = [&](int a, int b) -> bool {
        int pa = pos[a]; if (pa < 0) return false;
        return tour[(pa + 1) % (int)tour.size()] == b;
    };

    auto push_edge = [&](int a, int b) {
        int n = (int)tour.size();
        for (int u = 0; u < n_all; u++) {
            if (in_t[u]) continue;
            int d = points[u].node_weight - (dist[a][u] + dist[u][b] - dist[a][b]);
            if (d > 0) PQ.push({LSMove::ADD, a, b, -1, -1, u, d});
        }
        for (int i = 0; i < n; i++) {
            int c = tour[i], dn = tour[(i+1)%n];
            if (c == a || dn == a || c == b || dn == b) continue;
            int delta = calc_2opt_delta(a, b, c, dn, dist);
            if (delta > 0) {
                PQ.push({LSMove::SWAP_EDGES, a, c, b, dn, -1, delta});
                PQ.push({LSMove::SWAP_EDGES, c, a, dn, b, -1, delta});
            }
        }
    };

    auto push_remove = [&](int u) {
        int n = (int)tour.size();
        if (n <= 3 || !in_t[u]) return;
        int pu = pos[u];
        int vp = tour[(pu - 1 + n) % n], wn = tour[(pu + 1) % n];
        int d = -points[u].node_weight + (dist[vp][u] + dist[u][wn] - dist[vp][wn]);
        if (d > 0) PQ.push({LSMove::REMOVE, u, -1, -1, -1, -1, d});
    };

    // Initial full scan
    int n_init = (int)tour.size();
    for (int i = 0; i < n_init; i++) {
        int a = tour[i], b = tour[(i+1)%n_init];
        for (int u = 0; u < n_all; u++) {
            if (in_t[u]) continue;
            int d = points[u].node_weight - (dist[a][u] + dist[u][b] - dist[a][b]);
            if (d > 0) PQ.push({LSMove::ADD, a, b, -1, -1, u, d});
        }
        for (int j = i+2; j < n_init; j++) {
            if (i == 0 && j == n_init-1) continue;
            int c = tour[j], dn = tour[(j+1)%n_init];
            int delta = calc_2opt_delta(a, b, c, dn, dist);
            if (delta > 0) {
                PQ.push({LSMove::SWAP_EDGES, a, c, b, dn, -1, delta});
                PQ.push({LSMove::SWAP_EDGES, c, a, dn, b, -1, delta});
            }
        }
        if (n_init > 3) {
            int vp = tour[(i-1+n_init)%n_init], wn = tour[(i+1)%n_init];
            int d = -points[tour[i]].node_weight + (dist[vp][tour[i]] + dist[tour[i]][wn] - dist[vp][wn]);
            if (d > 0) PQ.push({LSMove::REMOVE, tour[i], -1, -1, -1, -1, d});
        }
    }

    while (!PQ.empty()) {
        LSMove m = PQ.top(); PQ.pop();
        int n = (int)tour.size();
        if (m.type == LSMove::ADD) {
            if (!in_t[m.u] && edge_ok(m.n1, m.n2)) {
                int f = pos[m.n1]; tour.insert(tour.begin() + f + 1, m.u);
                in_t[m.u] = true; pos[m.u] = f + 1; reindex(f + 2);
                push_edge(m.n1, m.u); push_edge(m.u, m.n2); push_remove(m.n1); push_remove(m.u); push_remove(m.n2);
            }
        } else if (m.type == LSMove::REMOVE) {
            if (in_t[m.n1] && n > 3) {
                int pu = pos[m.n1]; int vp = tour[(pu - 1 + n) % n], wn = tour[(pu + 1) % n];
                int actual_delta = -points[m.n1].node_weight + (dist[vp][m.n1] + dist[m.n1][wn] - dist[vp][wn]);
                if (actual_delta > 0) {
                    tour.erase(tour.begin() + pu); in_t[m.n1] = false; pos[m.n1] = -1; reindex(pu);
                    push_edge(vp, wn); push_remove(vp); push_remove(wn);
                }
            }
        } else if (m.type == LSMove::SWAP_EDGES) {
            int i1 = pos[m.n1], i1n = pos[m.n1_next], i2 = pos[m.n2], i2n = pos[m.n2_next];
            if (i1 >= 0 && i1n >= 0 && i2 >= 0 && i2n >= 0 && i1n == (i1 + 1) % n && i2n == (i2 + 1) % n) {
                reverse_segment(tour, i1n, i2); reindex_segment(i1n, i2);
                push_edge(m.n1, m.n2); push_edge(m.n1_next, m.n2_next);
                if (i1n <= i2) { for (int p = i1n; p < i2; p++) push_edge(tour[p], tour[(p+1)%n]); }
                else { for (int p = i1n; p < n; p++) push_edge(tour[p], tour[(p+1)%n]); for (int p = 0; p < i2; p++) push_edge(tour[p], tour[(p+1)%n]); }
                push_remove(m.n1); push_remove(m.n2); push_remove(m.n1_next); push_remove(m.n2_next);
            }
        }
    }
    return tour;
}
vector<vector<int>> buildNearestNeighborsList(const vector<vector<int>>& dist, int k_neighbors) {
    int n = dist.size();
    vector<vector<int>> nearest(n);
    for (int i = 0; i < n; i++) {
        vector<pair<int, int>> ds;
        for (int j = 0; j < n; j++) {
            if (i == j) continue;
            ds.push_back({ dist[i][j], j });
        }
        sort(ds.begin(), ds.end());
        for (int k = 0; k < k_neighbors && k < (int)ds.size(); k++) {
            nearest[i].push_back(ds[k].second);
        }
    }
    return nearest;
}
vector<int> solve_steepest_candidate(vector<int> tour, const vector<vector<int>> &dist, const vector<PointData> &points, const vector<vector<int>> &nearest)
{
    int n_all = points.size();
    bool improved = true;

    while (improved) {
        improved = false;
        int n = tour.size();
        if (n < 3) break;
        
        LSMove best;
        best.delta = 0;

        vector<int> pos(n_all, -1);
        for (int i = 0; i < n; i++) pos[tour[i]] = i;

        // ADD i SWAP_EDGES – tylko spośród k=10 najbliższych sąsiadów
        for (int i = 0; i < n; i++) {
            int v = tour[i];
            int v_next = tour[(i + 1) % n];
            
            for (int neighbor : nearest[v]) {
                int neighbor_pos = pos[neighbor];
                
                if (neighbor_pos == -1) {
                    // ADD: wstaw neighbor między v a v_next
                    int d = points[neighbor].node_weight - (dist[v][neighbor] + dist[neighbor][v_next] - dist[v][v_next]);
                    if (d > best.delta)
                        best = { LSMove::ADD, v, v_next, -1, -1, neighbor, d };
                } else {
                    // 2-OPT: zamień krawędzie (v, v_next) i (neighbor, neighbor_next)
                    int w = neighbor;
                    int w_next = tour[(neighbor_pos + 1) % n];
                    if (w != v && w != v_next && w_next != v) {
                        int d = (dist[v][v_next] + dist[w][w_next]) - (dist[v][w] + dist[v_next][w_next]);
                        if (d > best.delta)
                            best = { LSMove::SWAP_EDGES, v, w, v_next, w_next, -1, d };
                    }
                }
            }
        }
        
        // REMOVE – pełny przegląd (kandydaty nie pokrywają usunięć)
        for (int i = 0; i < n; i++) {
            int u = tour[i];
            int prev = tour[(i - 1 + n) % n];
            int next = tour[(i + 1) % n];
            int d = (dist[prev][u] + dist[u][next] - dist[prev][next]) - points[u].node_weight;
            if (d > best.delta)
                best = { LSMove::REMOVE, u, -1, -1, -1, -1, d };
        }

        if (best.delta > 0) {
            improved = true;
            if (best.type == LSMove::ADD) {
                tour.insert(tour.begin() + pos[best.n1] + 1, best.u);
            } else if (best.type == LSMove::REMOVE) {
                tour.erase(tour.begin() + pos[best.n1]);
            } else if (best.type == LSMove::SWAP_EDGES) {
                reverse_segment(tour, pos[best.n1_next], pos[best.n2]);
            }
        }
    }
    return tour;
}

// ZADANIE 4

// Perturbacja ILS: 3 losowe ruchy 2-OPT + usunięcie ≤2 węzłów + wstawienie 2 nowych
void perturb_ils(vector<int> &tour, const vector<PointData> &points) {
    int n = tour.size();
    if (n >= 4) {
        for (int i = 0; i < 3; i++) {
            int idx1 = rand() % n;
            int idx2 = rand() % n;
            if (abs(idx1 - idx2) > 1)
                reverse_segment(tour, min(idx1, idx2), max(idx1, idx2));
        }
    }
    int to_remove = min(2, max(0, (int)tour.size() - 3));
    for (int i = 0; i < to_remove; i++)
        tour.erase(tour.begin() + (rand() % tour.size()));
    for (int i = 0; i < 2; i++) {
        int new_node = rand() % (int)points.size();
        bool exists = false;
        for (int v : tour) if (v == new_node) { exists = true; break; }
        if (!exists) tour.push_back(new_node);
    }
}

// Naprawa LNS: heurystyka żalu 2-Regret – wstawia wszystkie brakujące węzły
void repair_lns(vector<int> &tour, const vector<vector<int>> &dist, const vector<PointData> &points) {
    int n_all = points.size();
    vector<bool> visited(n_all, false);
    for (int v : tour) visited[v] = true;

    while (tour.size() < (size_t)n_all) {
        double max_regret = -1.0;
        int best_node = -1;
        int best_pos = -1;

        for (int i = 0; i < n_all; i++) {
            if (visited[i]) continue;

            int min_cost = INT_MAX;
            int second_min_cost = INT_MAX;
            int temp_pos = -1;

            for (int j = 0; j < (int)tour.size(); j++) {
                int u = tour[j];
                int w = tour[(j + 1) % tour.size()];
                int cost = dist[u][i] + dist[i][w] - dist[u][w] - points[i].node_weight;

                if (cost < min_cost) {
                    second_min_cost = min_cost;
                    min_cost = cost;
                    temp_pos = j + 1;
                } else if (cost < second_min_cost) {
                    second_min_cost = cost;
                }
            }

            double regret = (double)(second_min_cost - min_cost);
            if (regret > max_regret) {
                max_regret = regret;
                best_node = i;
                best_pos = temp_pos;
            }
        }

        if (best_node != -1) {
            if (best_pos < 0) best_pos = 0;
            tour.insert(tour.begin() + best_pos, best_node);
            visited[best_node] = true;
        } else break;
    }
}
// MSLS: 200 niezależnych startów SLS, zwraca najlepszy wynik
vector<int> solve_msls(const vector<vector<int>> &dist, const vector<PointData> &points, double &measured_time_ms) {
    auto start_timer = std::chrono::high_resolution_clock::now();
    vector<int> best_overall;
    int best_obj = -INT_MAX;

    for (int i = 0; i < 200; i++) {
        vector<int> current = solve_random(rand() % points.size(), dist, points);
        current = solve_steepest_baseline(current, dist, points);
        
        int obj = calculate_objective(current, dist, points);
        if (obj > best_obj) {
            best_obj = obj;
            best_overall = current;
        }
    }
    
    auto end_timer = std::chrono::high_resolution_clock::now();
    measured_time_ms = std::chrono::duration<double, std::milli>(end_timer - start_timer).count();
    return best_overall;
}

// ILS: perturbacja + SLS w pętli do wyczerpania limitu czasu
vector<int> solve_ils(const vector<vector<int>> &dist, const vector<PointData> &points, double time_limit_ms, int &iters_done) {
    auto start_timer = std::chrono::high_resolution_clock::now();
    iters_done = 0;

    vector<int> x = solve_random(rand() % points.size(), dist, points);
    x = solve_steepest_baseline(x, dist, points);
    vector<int> x_best = x;
    int best_obj = calculate_objective(x, dist, points);
    int x_obj = best_obj;

    while (std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - start_timer).count() < time_limit_ms) {
        vector<int> y = x;
        perturb_ils(y, points);
        y = solve_steepest_baseline(y, dist, points);

        int y_obj = calculate_objective(y, dist, points);

        if (y_obj > x_obj) {
            x = y;
            x_obj = y_obj;
        }
        if (y_obj > best_obj) {
            best_obj = y_obj;
            x_best = y;
        }
        iters_done++;
    }
    return x_best;
}

// LNS
vector<int> solve_lns(const vector<vector<int>> &dist, const vector<PointData> &points, double time_limit_ms, int &iters_done, bool use_local_search) {
    auto start_timer = std::chrono::high_resolution_clock::now();
    iters_done = 0;

    vector<int> x = solve_random(rand() % points.size(), dist, points);
    x = solve_steepest_baseline(x, dist, points);
    int best_obj = calculate_objective(x, dist, points);

    while (std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - start_timer).count() < time_limit_ms) {
        vector<int> y = x;
        int n = y.size();
        
        // Destroy
        int num_segments = 3 + (rand() % 2);
        int segment_len = (n * 3 / 10) / num_segments;
        for (int s = 0; s < num_segments && y.size() > segment_len + 3; s++) {
            int start_idx = rand() % y.size();
            for (int j = 0; j < segment_len; j++)
                y.erase(y.begin() + (start_idx % y.size()));
        }

        // Repair
        repair_lns(y, dist, points);

        if (use_local_search) {
            y = solve_steepest_baseline(y, dist, points);
        }
        
        int current_obj = calculate_objective(y, dist, points);
        if (current_obj > best_obj) {
            x = y;
            best_obj = current_obj;
        }
        iters_done++;
    }
    return x;
}

// ===== Zadanie 6: HAE =====

static int haeEdgeKey(int a, int b, int n_all) {
    if (a > b) swap(a, b);
    return a * n_all + b;
}

static unordered_set<int> buildEdgeSet(const vector<int>& tour, int n_all) {
    unordered_set<int> es;
    int k = tour.size();
    for (int i = 0; i < k; i++)
        es.insert(haeEdgeKey(tour[i], tour[(i + 1) % k], n_all));
    return es;
}

// Extract subpaths from tour1 using common vertex set cv and common edge set ce.
// Starting from a break point ensures correct cycle handling.
// keep_singles=true (Op1): isolated common vertices become length-1 subpaths.
// keep_singles=false (Op2): free vertices (no active edges on either side) are discarded.
static vector<vector<int>> extractSubpaths(
    const vector<int>& tour1,
    const unordered_set<int>& cv,
    const unordered_set<int>& ce,
    int n_all,
    bool keep_singles)
{
    int n = tour1.size();
    if (n == 0) return {};

    // Find a break point: position where vertex not common OR incoming edge not common
    int start = 0;
    for (int i = 0; i < n; i++) {
        int v  = tour1[i];
        int vp = tour1[(i - 1 + n) % n];
        if (!cv.count(v) || !ce.count(haeEdgeKey(v, vp, n_all))) {
            start = i;
            break;
        }
    }

    vector<vector<int>> subpaths;
    vector<int> current;

    for (int step = 0; step < n; step++) {
        int i  = (start + step) % n;
        int v  = tour1[i];
        int vp = tour1[(i - 1 + n) % n];
        bool extend = !current.empty() && cv.count(v) && ce.count(haeEdgeKey(v, vp, n_all));

        if (extend) {
            current.push_back(v);
        } else {
            if (!current.empty()) {
                if (keep_singles || current.size() > 1) subpaths.push_back(current);
                current.clear();
            }
            if (cv.count(v)) current = {v};
        }
    }
    if (!current.empty() && (keep_singles || current.size() > 1))
        subpaths.push_back(current);

    return subpaths;
}

static vector<int> joinSubpaths(vector<vector<int>> subpaths) {
    for (int i = subpaths.size() - 1; i > 0; i--)
        swap(subpaths[i], subpaths[rand() % (i + 1)]);
    vector<int> result;
    for (auto& sp : subpaths) {
        if (rand() % 2) reverse(sp.begin(), sp.end());
        result.insert(result.end(), sp.begin(), sp.end());
    }
    return result;
}

vector<int> recombine_op1(const vector<int>& p1, const vector<int>& p2,
                          const vector<vector<int>>& dist, const vector<PointData>& pts) {
    int n_all = pts.size();
    unordered_set<int> v2(p2.begin(), p2.end());
    unordered_set<int> cv;
    for (int v : p1) if (v2.count(v)) cv.insert(v);
    auto e1 = buildEdgeSet(p1, n_all);
    auto e2 = buildEdgeSet(p2, n_all);
    unordered_set<int> ce;
    for (int k : e1) if (e2.count(k)) ce.insert(k);

    auto child = joinSubpaths(extractSubpaths(p1, cv, ce, n_all, true));
    repair_lns(child, dist, pts);
    return phase2_remove(child, dist, pts);
}

vector<int> recombine_op2(const vector<int>& p1, const vector<int>& p2,
                          const vector<vector<int>>& dist, const vector<PointData>& pts) {
    int n_all = pts.size();
    unordered_set<int> v2(p2.begin(), p2.end());
    unordered_set<int> cv;
    for (int v : p1) if (v2.count(v)) cv.insert(v);
    auto e1 = buildEdgeSet(p1, n_all);
    auto e2 = buildEdgeSet(p2, n_all);
    unordered_set<int> ce;
    for (int k : e1) if (e2.count(k)) ce.insert(k);

    auto child = joinSubpaths(extractSubpaths(p1, cv, ce, n_all, false));
    repair_lns(child, dist, pts);
    return phase2_remove(child, dist, pts);
}

vector<int> recombine_op3(const vector<int>& p1, const vector<int>& p2,
                          const vector<vector<int>>& dist, const vector<PointData>& pts) {
    unordered_set<int> v2(p2.begin(), p2.end());
    vector<int> child;
    for (int v : p1) if (v2.count(v)) child.push_back(v);
    repair_lns(child, dist, pts);
    return phase2_remove(child, dist, pts);
}

vector<int> solve_hae(const vector<vector<int>>& dist, const vector<PointData>& pts,
                      double time_limit_ms, int& iters_done, int op, bool use_ls) {
    auto t0 = chrono::high_resolution_clock::now();
    iters_done = 0;
    int n_all = pts.size();

    struct Ind { vector<int> tour; int obj; };
    vector<Ind> pop;
    pop.reserve(20);

    // Build initial population of 20 unique (by objective) solutions
    int attempts = 0;
    while ((int)pop.size() < 20 && attempts < 400) {
        vector<int> t = solve_random(rand() % n_all, dist, pts);
        t = solve_steepest_baseline(t, dist, pts);
        int obj = calculate_objective(t, dist, pts);
        bool dup = false;
        for (auto& m : pop) if (m.obj == obj) { dup = true; break; }
        if (!dup) pop.push_back({t, obj});
        attempts++;
    }
    while ((int)pop.size() < 20) {
        vector<int> t = solve_random(rand() % n_all, dist, pts);
        t = solve_steepest_baseline(t, dist, pts);
        pop.push_back({t, calculate_objective(t, dist, pts)});
    }

    int best_obj = INT_MIN;
    vector<int> best_tour;
    for (auto& m : pop) if (m.obj > best_obj) { best_obj = m.obj; best_tour = m.tour; }

    while (chrono::duration<double, milli>(chrono::high_resolution_clock::now() - t0).count() < time_limit_ms) {
        int pa = rand() % 20;
        int pb; do { pb = rand() % 20; } while (pb == pa);

        vector<int> child;
        if (op == 1)      child = recombine_op1(pop[pa].tour, pop[pb].tour, dist, pts);
        else if (op == 2) child = recombine_op2(pop[pa].tour, pop[pb].tour, dist, pts);
        else              child = recombine_op3(pop[pa].tour, pop[pb].tour, dist, pts);

        if (use_ls) child = solve_steepest_baseline(child, dist, pts);

        int child_obj = calculate_objective(child, dist, pts);

        // Find worst member
        int worst_idx = 0;
        for (int i = 1; i < 20; i++)
            if (pop[i].obj < pop[worst_idx].obj) worst_idx = i;

        if (child_obj > pop[worst_idx].obj) {
            bool dup = false;
            for (auto& m : pop) if (m.obj == child_obj) { dup = true; break; }
            if (!dup) {
                pop[worst_idx] = {child, child_obj};
                if (child_obj > best_obj) { best_obj = child_obj; best_tour = child; }
            }
        }
        iters_done++;
    }
    return best_tour;
}

vector<int> solve_greedy_construct(const vector<vector<int>>& dist, const vector<PointData>& pts) {
    vector<int> tour = {rand() % (int)pts.size()};
    repair_lns(tour, dist, pts);
    return phase2_remove(tour, dist, pts);
}