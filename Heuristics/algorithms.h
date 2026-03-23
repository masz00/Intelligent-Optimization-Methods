#pragma once
#include <vector>
#include "tsp_utils.h"

std::vector<int> phase2_remove(std::vector<int> cycle, const std::vector<std::vector<int>> &dist, const std::vector<PointData> &points);

std::vector<int> solve_random(int start_idx, const std::vector<std::vector<int>> &dist, const std::vector<PointData> &points);

std::vector<int> solve_nn(int start_idx, bool use_weights, const std::vector<std::vector<int>> &dist, const std::vector<PointData> &points);

std::vector<int> solve_greedy_cycle(int start_idx, bool use_weights, const std::vector<std::vector<int>> &dist, const std::vector<PointData> &points);

std::vector<int> solve_regret(int start_idx, bool use_weights, double w1, double w2, const std::vector<std::vector<int>> &dist, const std::vector<PointData> &points);
