#pragma once
#include <vector>
#include "tsp_utils.h"

std::vector<int> phase2_remove(std::vector<int> cycle, const std::vector<std::vector<int>> &dist, const std::vector<PointData> &points);

std::vector<int> solve_random(int start_idx, const std::vector<std::vector<int>> &dist, const std::vector<PointData> &points);

std::vector<int> solve_nn(int start_idx, bool use_weights, const std::vector<std::vector<int>> &dist, const std::vector<PointData> &points);

std::vector<int> solve_greedy_cycle(int start_idx, bool use_weights, const std::vector<std::vector<int>> &dist, const std::vector<PointData> &points);

std::vector<int> solve_regret(int start_idx, bool use_weights, double w1, double w2, const std::vector<std::vector<int>> &dist, const std::vector<PointData> &points);

std::vector<int> solve_steepest_baseline(std::vector<int> tour, const std::vector<std::vector<int>> &dist, const std::vector<PointData> &points);

std::vector<int> solve_steepest_lm(std::vector<int> tour, const std::vector<std::vector<int>> &dist, const std::vector<PointData> &points);

std::vector<std::vector<int>> buildNearestNeighborsList(const std::vector<std::vector<int>>& dist, int k_neighbors);

std::vector<int> solve_steepest_candidate(std::vector<int> tour,
                                          const std::vector<std::vector<int>> &dist,
                                          const std::vector<PointData> &points,
                                          const std::vector<std::vector<int>> &nearest);

// Zadanie 4 

void perturb_ils(std::vector<int> &tour, const std::vector<PointData> &points);

void repair_lns(std::vector<int> &tour, const std::vector<std::vector<int>> &dist, const std::vector<PointData> &points);

std::vector<int> solve_msls(const std::vector<std::vector<int>> &dist,
                            const std::vector<PointData> &points,
                            double &measured_time_ms);

std::vector<int> solve_ils(const std::vector<std::vector<int>> &dist,
                           const std::vector<PointData> &points,
                           double time_limit_ms,
                           int &iters_done);

std::vector<int> solve_lns(const std::vector<std::vector<int>> &dist,
                           const std::vector<PointData> &points,
                           double time_limit_ms,
                           int &iters_done,
                           bool use_local_search);
