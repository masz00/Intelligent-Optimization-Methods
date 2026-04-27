#include "algorithms.h"
#include <climits>
#include <algorithm>
#include <numeric>
#include <random>
#include <iostream>
#include <vector>
#include <queue>

using namespace std;

struct LSMove {
    enum Type { NONE, ADD, REMOVE, SWAP_EDGES } type = NONE;
    int n1 = -1, n2 = -1;
    int n1_next = -1, n2_next = -1;
    int u = -1;
    int delta = -INT_MAX;

    bool operator<(const LSMove& other) const {
        return delta < other.delta; // Max-heap behavior
    }
};

inline int calc_2opt_delta(int a, int b, int c, int d, const vector<vector<int>>& dist) {
    // Delta = OldDist - NewDist (Maximization of Profit - Dist)
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
    std::random_device rd;
    std::mt19937 g(rd());
    shuffle(path.begin() + 1, path.end(), g);
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

vector<int> solve_steepest_candidate(vector<int> tour, const vector<vector<int>> &dist, const vector<PointData> &points, int k_neighbors)
{
    int n_all = points.size();
    vector<vector<int>> nearest(n_all);
    vector<vector<bool>> is_candidate(n_all, vector<bool>(n_all, false));
    for (int i = 0; i < n_all; i++) {
        vector<pair<int, int>> ds; for (int j = 0; j < n_all; j++) { if (i == j) continue; ds.push_back({ dist[i][j], j }); }
        sort(ds.begin(), ds.end());
        for (int k = 0; k < k_neighbors && k < (int)ds.size(); k++) {
            nearest[i].push_back(ds[k].second);
            is_candidate[i][ds[k].second] = is_candidate[ds[k].second][i] = true;
        }
    }
    bool improved = true;
    while (improved) {
        improved = false; int n = tour.size(); LSMove best; vector<bool> in_t(n_all, false); for(int v:tour) in_t[v]=true;
        for (int i = 0; i < n; i++) {
            int v = tour[i], v_n = tour[(i+1)%n];
            for (int neighbor : nearest[v]) {
                if (!in_t[neighbor]) {
                    int d = points[neighbor].node_weight - (dist[v][neighbor] + dist[neighbor][v_n] - dist[v][v_n]);
                    if (d > best.delta) best = { LSMove::ADD, v, v_n, -1, -1, neighbor, d };
                } else {
                    int idx_n = -1; for(int k=0; k<n; k++) if(tour[k] == neighbor) { idx_n = k; break; }
                    int w = neighbor, w_n = tour[(idx_n + 1) % n];
                    if (w != v && w != v_n && w_n != v) {
                        int d = (dist[v][v_n] + dist[w][w_n]) - (dist[v][w] + dist[v_n][w_n]);
                        if (d > best.delta) best = { LSMove::SWAP_EDGES, v, w, v_n, w_n, -1, d };
                    }
                }
            }
        }
        if (n > 3) {
            for (int i = 0; i < n; i++) {
                int u = tour[i], v_p = tour[(i - 1 + n) % n], w_n = tour[(i + 1) % n];
                if (!is_candidate[v_p][w_n]) continue;
                int d = -points[u].node_weight + (dist[v_p][u] + dist[u][w_n] - dist[v_p][w_n]);
                if (d > best.delta) best = { LSMove::REMOVE, u, -1, -1, -1, -1, d };
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
