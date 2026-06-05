#pragma once
#include <vector>
#include <fstream>
#include "tsp_utils.h"

// Optional convergence logger: when non-null, solve_hae and solve_hae_cls append
extern std::ofstream* g_convergence_log;

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


// Op1: common vertices + edges → subpaths → join randomly → repair
std::vector<int> recombine_op1(const std::vector<int>& p1, const std::vector<int>& p2,
                               const std::vector<std::vector<int>>& dist, const std::vector<PointData>& pts);

// Op2: common vertices + edges → subpaths (discard free vertices) → join randomly → repair
std::vector<int> recombine_op2(const std::vector<int>& p1, const std::vector<int>& p2,
                               const std::vector<std::vector<int>>& dist, const std::vector<PointData>& pts);

// Op3: keep only shared vertices in p1 order → repair (no joining needed)
std::vector<int> recombine_op3(const std::vector<int>& p1, const std::vector<int>& p2,
                               const std::vector<std::vector<int>>& dist, const std::vector<PointData>& pts);

std::vector<int> solve_hae(const std::vector<std::vector<int>>& dist, const std::vector<PointData>& pts,
                           double time_limit_ms, int& iters_done, int op, bool use_ls);

// Greedy construction baseline: random start node → repair_lns → phase2_remove
std::vector<int> solve_greedy_construct(const std::vector<std::vector<int>>& dist, const std::vector<PointData>& pts);



struct HaeCustomCfg {
    int op = 1;                     // 1/2/3 recombination operator (when use_alns=false)
    bool use_ls = true;
    // M1: candidate-based LS in place of baseline
    bool use_candidate_ls = false;
    int candidate_k = 10;
    // alt LS: move-list (LM) variant
    bool use_lm_ls = false;
    // NN init: use solve_nn(use_weights=true) for population init instead of random
    bool use_nn_init = false;
    // M2: SA acceptance
    bool use_sa_accept = false;
    double sa_T0 = 200.0;
    double sa_alpha = 0.999;
    // M3: ALNS adaptive operator selection
    bool use_alns = false;
    double alns_reward_better = 5.0;
    double alns_reward_accepted = 2.0;
    double alns_decay = 0.9;
    int alns_window = 20;
    // M4: memetic deep intensification
    bool use_memetic = false;
    int memetic_every = 50;
    int memetic_top_k = 3;
    int memetic_perturb_count = 5;
    // population
    int pop_size = 20;
    // Path-relinking flavor: every pr_every iters, pick parents = top-2 elite (not random)
    bool use_pr = false;
    int pr_every = 10;
};

std::vector<int> solve_hae_custom(const std::vector<std::vector<int>>& dist,
                                   const std::vector<PointData>& pts,
                                   double time_limit_ms,
                                   int& iters_done,
                                   const HaeCustomCfg& cfg,
                                   const std::vector<std::vector<int>>* nearest = nullptr);

// Final method: HAE-CLS: op1 recombination,
// pop_size=15, candidate_k=15. Hardcoded
std::vector<int> solve_hae_cls(const std::vector<std::vector<int>>& dist,
                                const std::vector<PointData>& pts,
                                double time_limit_ms,
                                int& iters_done,
                                const std::vector<std::vector<int>>& nearest);
