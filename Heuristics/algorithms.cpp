#include "algorithms.h"
#include <climits>
#include <algorithm>
#include <numeric>
#include <random>

using namespace std;

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

            if (n_cyc == 2)
            {
                delta_cost = 0 - 2 * dist[node_u][node_v] + points[node_v].node_weight;
            }

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
    std::shuffle(path.begin() + 1, path.end(), g);

    path.resize(l_size);
    return path;
}

vector<int> solve_nn(int start_idx, bool use_weights, const vector<vector<int>> &dist, const vector<PointData> &points)
{
    int max_nodes = points.size();
    vector<bool> visited(max_nodes, false);
    vector<int> route;
    route.reserve(max_nodes);

    route.push_back(start_idx);
    visited[start_idx] = true;

    while (route.size() < (size_t)max_nodes)
    {
        int r_tail = route.back();
        int b_cand = -1;
        int min_val = INT_MAX;

        for (int v = 0; v < max_nodes; v++)
        {
            if (visited[v])
                continue;
            int insert_val = dist[r_tail][v];
            if (use_weights)
                insert_val -= points[v].node_weight;

            if (insert_val < min_val)
            {
                min_val = insert_val;
                b_cand = v;
            }
        }

        route.push_back(b_cand);
        visited[b_cand] = true;
    }

    return route;
}

vector<int> solve_greedy_cycle(int start_idx, bool use_weights, const vector<vector<int>> &dist, const vector<PointData> &points)
{
    int max_nodes = points.size();
    vector<bool> visited(max_nodes, false);
    vector<int> route;
    route.reserve(max_nodes);

    route.push_back(start_idx);
    visited[start_idx] = true;

    int scnd_node = -1;
    int min_first_edge = INT_MAX;
    for (int v = 0; v < max_nodes; v++)
    {
        if (visited[v])
            continue;
        int e_val = dist[start_idx][v] - (use_weights ? points[v].node_weight : 0);
        if (e_val < min_first_edge)
        {
            min_first_edge = e_val;
            scnd_node = v;
        }
    }
    route.push_back(scnd_node);
    visited[scnd_node] = true;

    while (route.size() < (size_t)max_nodes)
    {
        int selected_target_pos = 0;
        int selected_candidate = -1;
        int global_lowest_cost = INT_MAX;

        int r_len = route.size();
        for (int i = 0; i < r_len; i++)
        {
            int n_a = route[i];
            int n_b = route[(i + 1) % r_len];

            for (int v = 0; v < max_nodes; v++)
            {
                if (visited[v])
                    continue;
                int current_ins_cost = dist[n_a][v] + dist[v][n_b];
                if (r_len >= 2)
                    current_ins_cost -= dist[n_a][n_b];
                if (use_weights)
                    current_ins_cost -= points[v].node_weight;

                if (current_ins_cost < global_lowest_cost)
                {
                    global_lowest_cost = current_ins_cost;
                    selected_candidate = v;
                    selected_target_pos = i;
                }
            }
        }
        route.insert(route.begin() + selected_target_pos + 1, selected_candidate);
        visited[selected_candidate] = true;
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

    int scnd_node = -1;
    int min_first_edge = INT_MAX;
    for (int v = 0; v < max_nodes; v++)
    {
        if (visited[v])
            continue;
        int e_val = dist[start_idx][v] - (use_weights ? points[v].node_weight : 0);
        if (e_val < min_first_edge)
        {
            min_first_edge = e_val;
            scnd_node = v;
        }
    }
    route.push_back(scnd_node);
    visited[scnd_node] = true;

    while (route.size() < (size_t)max_nodes)
    {
        double top_score = -1e9;
        int r_cand = -1;
        int target_pos = -1;
        int r_len = route.size();

        for (int v = 0; v < max_nodes; v++)
        {
            if (visited[v])
                continue;

            int lowest_val = INT_MAX;
            int runner_up_val = INT_MAX;
            int lowest_pos = 0;

            for (int i = 0; i < r_len; i++)
            {
                int n_a = route[i];
                int n_b = route[(i + 1) % r_len];

                int c_val = dist[n_a][v] + dist[v][n_b];
                if (r_len >= 2)
                    c_val -= dist[n_a][n_b];
                if (use_weights)
                    c_val -= points[v].node_weight;

                if (c_val < lowest_val)
                {
                    runner_up_val = lowest_val;
                    lowest_val = c_val;
                    lowest_pos = i;
                }
                else if (c_val < runner_up_val)
                {
                    runner_up_val = c_val;
                }
            }

            double evaluated_score = w1 * (double)(runner_up_val - lowest_val) - w2 * (double)lowest_val;
            if (evaluated_score > top_score)
            {
                top_score = evaluated_score;
                r_cand = v;
                target_pos = lowest_pos;
            }
        }

        route.insert(route.begin() + target_pos + 1, r_cand);
        visited[r_cand] = true;
    }
    return route;
}
