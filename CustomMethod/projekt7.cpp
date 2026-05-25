#include <iostream>
#include <vector>
#include <chrono>
#include <algorithm>
#include <iomanip>
#include <fstream>
#include <filesystem>
#include <cstdlib>
#include <ctime>
#include <climits>
#include "tsp_utils.h"
#include "algorithms.h"

using namespace std;
using namespace chrono;

static constexpr double TIME_TSPA = 1420.0;
static constexpr double TIME_TSPB = 1578.0;

struct Stats {
    long long min_obj = (long long)2e18, max_obj = (long long)-2e18;
    long long sum_obj = 0;
    double    sum_time_ms = 0;
    long long sum_iters   = 0;
    int       n_runs      = 0;
    vector<int> best_tour;

    void update(long long obj, double time_ms, int iters, const vector<int>& tour) {
        if (obj < min_obj) min_obj = obj;
        if (obj > max_obj) { max_obj = obj; best_tour = tour; }
        sum_obj      += obj;
        sum_time_ms  += time_ms;
        sum_iters    += iters;
        n_runs++;
    }
    long long avg() const { return n_runs ? sum_obj / n_runs : 0; }
    double    avg_time() const { return n_runs ? sum_time_ms / n_runs : 0; }
    long long avg_iters() const { return n_runs ? sum_iters / n_runs : 0; }
};

struct Variant {
    string name;
    HaeCustomCfg cfg;
    bool needs_nearest;
};

vector<Variant> build_variants_screening() {
    vector<Variant> V;
    { HaeCustomCfg c; c.op = 2; c.use_ls = true;
      V.push_back({"HAE2_base", c, false}); }
    { HaeCustomCfg c; c.op = 2; c.use_ls = true; c.use_candidate_ls = true; c.candidate_k = 10;
      V.push_back({"M1_candLS", c, true}); }
    { HaeCustomCfg c; c.op = 2; c.use_ls = true; c.use_sa_accept = true; c.sa_T0 = 200; c.sa_alpha = 0.999;
      V.push_back({"M2_SA", c, false}); }
    { HaeCustomCfg c; c.use_ls = true; c.use_alns = true;
      V.push_back({"M3_ALNS", c, false}); }
    { HaeCustomCfg c; c.op = 2; c.use_ls = true; c.use_memetic = true; c.memetic_every = 50; c.memetic_top_k = 3; c.memetic_perturb_count = 5;
      V.push_back({"M4_meme", c, false}); }
    { HaeCustomCfg c; c.use_ls = true; c.use_candidate_ls = true; c.candidate_k = 10; c.use_alns = true;
      V.push_back({"M5_M1+M3", c, true}); }
    return V;
}

vector<Variant> build_variants_eval() {
    vector<Variant> V;
    { HaeCustomCfg c; c.op = 2; c.use_ls = true;
      V.push_back({"HAE2_base", c, false}); }
    { HaeCustomCfg c; c.op = 2; c.use_ls = true; c.use_candidate_ls = true; c.candidate_k = 10;
      V.push_back({"M1_candLS", c, true}); }
    { HaeCustomCfg c; c.use_ls = true; c.use_candidate_ls = true; c.candidate_k = 10; c.use_alns = true;
      V.push_back({"M5_M1+M3", c, true}); }
    return V;
}

vector<Variant> build_variants_explore() {
    // Ablation against HAE-CLS winner cfg (op=1, k=15, pop=15, CandLS).
    // Each variant differs from ref by EXACTLY ONE knob — clean ablation.
    // Pop/k variants intentionally omitted (covered by Phase II grid search).
    vector<Variant> V;
    { HaeCustomCfg c; c.op = 1; c.use_ls = true; c.use_candidate_ls = true; c.candidate_k = 15; c.pop_size = 15;
      V.push_back({"HAE-CLS_ref",  c, true}); }
    { HaeCustomCfg c; c.op = 1; c.use_ls = true; c.use_candidate_ls = true; c.candidate_k = 15; c.pop_size = 15;
      c.use_nn_init = true;
      V.push_back({"HAE-CLS_NN",   c, true}); }
    { HaeCustomCfg c; c.op = 1; c.use_ls = true; c.use_lm_ls = true; c.pop_size = 15;
      V.push_back({"HAE-CLS_LM",   c, false}); }
    { HaeCustomCfg c; c.op = 3; c.use_ls = true; c.use_candidate_ls = true; c.candidate_k = 15; c.pop_size = 15;
      V.push_back({"HAE-CLS_op3",  c, true}); }
    { HaeCustomCfg c; c.op = 1; c.use_ls = true; c.use_candidate_ls = true; c.candidate_k = 15; c.pop_size = 15;
      c.use_sa_accept = true; c.sa_T0 = 200; c.sa_alpha = 0.999;
      V.push_back({"HAE-CLS_SA",   c, true}); }
    { HaeCustomCfg c; c.use_ls = true; c.use_candidate_ls = true; c.candidate_k = 15; c.pop_size = 15;
      c.use_alns = true;
      V.push_back({"HAE-CLS_ALNS", c, true}); }
    return V;
}

static string g_mode = "screening";
vector<Variant> build_variants() {
    if (g_mode == "eval") return build_variants_eval();
    if (g_mode == "explore") return build_variants_explore();
    return build_variants_screening();
}

void run_screening(const string& filename, double time_limit_ms, int runs) {
    auto points = load_tsp_instance(filename);
    auto dist   = build_distance_matrix(points);
    auto nearest = buildNearestNeighborsList(dist, 10);

    string inst = filename.substr(filename.find_last_of('/') + 1);
    inst = inst.substr(0, inst.find_last_of('.'));

    cout << "\n=== " << inst << " (time=" << time_limit_ms << "ms, runs=" << runs << ") ===" << endl;

    filesystem::create_directories("solutions/all");
    filesystem::create_directories("solutions/best");
    filesystem::create_directories("viz");

    auto variants = build_variants();
    vector<Stats> S(variants.size());

    for (size_t vi = 0; vi < variants.size(); vi++) {
        const auto& V = variants[vi];
        for (int r = 0; r < runs; r++) {
            cout << V.name << " " << (r+1) << "/" << runs << "...\r" << flush;
            int iters = 0;
            const vector<vector<int>>* nn = V.needs_nearest ? &nearest : nullptr;
            auto t0  = high_resolution_clock::now();
            auto res = solve_hae_custom(dist, points, time_limit_ms, iters, V.cfg, nn);
            double t = duration<double, milli>(high_resolution_clock::now() - t0).count();
            S[vi].update(calculate_objective(res, dist, points), t, iters, res);
            save_cycle_to_file("solutions/all/" + inst + "_" + V.name + "_run" + to_string(r) + ".txt", res, dist, points);
        }
        cout << V.name << " done.            " << endl;
    }

    // Table
    cout << "\nMetoda      | Avg (min - max)              | iters  | t[ms]" << endl;
    cout << string(80, '-') << endl;
    for (size_t i = 0; i < variants.size(); i++) {
        const auto& V = variants[i];
        const auto& s = S[i];
        cout << left << setw(11) << V.name
             << " | " << setw(28) << (to_string(s.avg()) + " (" + to_string(s.min_obj) + " - " + to_string(s.max_obj) + ")")
             << " | " << setw(6) << s.avg_iters()
             << " | " << fixed << setprecision(0) << s.avg_time() << endl;
    }

    // Save best per variant
    for (size_t i = 0; i < variants.size(); i++) {
        const auto& V = variants[i];
        generate_svg("viz/" + inst + "_" + V.name + ".svg", S[i].best_tour, points);
        save_cycle_to_file("solutions/best/" + inst + "_" + V.name + ".txt", S[i].best_tour, dist, points);
    }
}

// Single-config mode: run one cfg × N runs × 2 instances, print one CSV row.
// Usage: main.exe single <runs> <op> <k> <pop> <use_alns:0/1> <use_memetic:0/1> <use_sa:0/1>
int run_single(int runs, int op, int k, int pop, int use_alns, int use_memetic, int use_sa) {
    HaeCustomCfg cfg;
    cfg.op = op;
    cfg.use_ls = true;
    cfg.use_candidate_ls = true;
    cfg.candidate_k = k;
    cfg.pop_size = pop;
    cfg.use_alns = use_alns != 0;
    cfg.use_memetic = use_memetic != 0;
    cfg.use_sa_accept = use_sa != 0;

    auto run_one = [&](const string& filename, double tlim) {
        auto pts = load_tsp_instance(filename);
        auto dist = build_distance_matrix(pts);
        auto nearest = buildNearestNeighborsList(dist, k);
        long long sum_obj = 0, mn = (long long)2e18, mx = (long long)-2e18;
        for (int r = 0; r < runs; r++) {
            int iters = 0;
            auto t0 = high_resolution_clock::now();
            auto res = solve_hae_custom(dist, pts, tlim, iters, cfg, &nearest);
            double t = duration<double, milli>(high_resolution_clock::now() - t0).count();
            (void)t;
            long long o = calculate_objective(res, dist, pts);
            sum_obj += o;
            if (o < mn) mn = o;
            if (o > mx) mx = o;
        }
        return tuple<long long, long long, long long>(sum_obj / runs, mn, mx);
    };

    auto [a_avg, a_min, a_max] = run_one("instances/TSPA.csv", TIME_TSPA);
    auto [b_avg, b_min, b_max] = run_one("instances/TSPB.csv", TIME_TSPB);
    // CSV: op,k,pop,alns,memetic,sa,runs,a_avg,a_min,a_max,b_avg,b_min,b_max
    cout << op << "," << k << "," << pop << "," << use_alns << "," << use_memetic << "," << use_sa
         << "," << runs
         << "," << a_avg << "," << a_min << "," << a_max
         << "," << b_avg << "," << b_min << "," << b_max << endl;
    return 0;
}

// PR test: winner cfg + path-relinking on top-2 elite every pr_every iter
int run_pr_test(int runs, int pr_every) {
    auto run_one = [&](const string& filename, double tlim, bool with_pr) {
        auto pts = load_tsp_instance(filename);
        auto dist = build_distance_matrix(pts);
        auto nearest = buildNearestNeighborsList(dist, 15);
        HaeCustomCfg cfg;
        cfg.op = 1;
        cfg.use_ls = true;
        cfg.use_candidate_ls = true;
        cfg.candidate_k = 15;
        cfg.pop_size = 15;
        cfg.use_pr = with_pr;
        cfg.pr_every = pr_every;
        long long sum_obj = 0, mn = (long long)2e18, mx = (long long)-2e18;
        for (int r = 0; r < runs; r++) {
            int iters = 0;
            auto res = solve_hae_custom(dist, pts, tlim, iters, cfg, &nearest);
            long long o = calculate_objective(res, dist, pts);
            sum_obj += o;
            if (o < mn) mn = o;
            if (o > mx) mx = o;
        }
        return tuple<long long, long long, long long>(sum_obj / runs, mn, mx);
    };

    cout << ">>> PR test: pr_every=" << pr_every << " runs=" << runs << endl;
    {
        auto [a, mn, mx] = run_one("instances/TSPA.csv", TIME_TSPA, false);
        cout << "TSPA winner (no PR): avg=" << a << " (" << mn << "-" << mx << ")" << endl;
    }
    {
        auto [a, mn, mx] = run_one("instances/TSPA.csv", TIME_TSPA, true);
        cout << "TSPA winner+PR:      avg=" << a << " (" << mn << "-" << mx << ")" << endl;
    }
    {
        auto [a, mn, mx] = run_one("instances/TSPB.csv", TIME_TSPB, false);
        cout << "TSPB winner (no PR): avg=" << a << " (" << mn << "-" << mx << ")" << endl;
    }
    {
        auto [a, mn, mx] = run_one("instances/TSPB.csv", TIME_TSPB, true);
        cout << "TSPB winner+PR:      avg=" << a << " (" << mn << "-" << mx << ")" << endl;
    }
    return 0;
}

// Convergence mode: run HAE2 and HAE-CLS N times per instance with per-iter
// logging. Produces N × 4 CSVs in convergence/ for aggregate plotting.
int run_convergence(int runs = 20) {
    filesystem::create_directories("convergence");

    auto run_n = [&](const string& filename, double tlim,
                     const string& algo, const string& inst) {
        auto pts = load_tsp_instance(filename);
        auto dist = build_distance_matrix(pts);
        auto nearest = buildNearestNeighborsList(dist, 15);  // shared for all HAE-CLS runs

        long long best_final = LLONG_MIN, sum_final = 0;
        for (int r = 0; r < runs; r++) {
            cout << inst << " " << algo << " " << (r + 1) << "/" << runs << "...\r" << flush;
            string outpath = "convergence/" + inst + "_" + algo + "_run" + to_string(r) + ".csv";
            ofstream log(outpath);
            log << "time_ms,best_obj\n";
            g_convergence_log = &log;

            int iters = 0;
            long long obj = 0;
            if (algo == "HAE-CLS") {
                auto res = solve_hae_cls(dist, pts, tlim, iters, nearest);
                obj = calculate_objective(res, dist, pts);
            } else {
                auto res = solve_hae(dist, pts, tlim, iters, 2, true);
                obj = calculate_objective(res, dist, pts);
            }

            g_convergence_log = nullptr;
            log.close();
            sum_final += obj;
            if (obj > best_final) best_final = obj;
        }
        cout << inst << " " << algo << " done. avg_final=" << (sum_final / runs)
             << " max_final=" << best_final << "             " << endl;
    };

    cout << ">>> Convergence logging (runs=" << runs << " per algo per inst)" << endl;
    run_n("instances/TSPA.csv", TIME_TSPA, "HAE2",    "TSPA");
    run_n("instances/TSPA.csv", TIME_TSPA, "HAE-CLS", "TSPA");
    run_n("instances/TSPB.csv", TIME_TSPB, "HAE2",    "TSPB");
    run_n("instances/TSPB.csv", TIME_TSPB, "HAE-CLS", "TSPB");
    return 0;
}

// Final mode: run solve_hae_cls 20× on both instances, save best + SVGs
int run_final(int runs) {
    filesystem::create_directories("solutions/all");
    filesystem::create_directories("solutions/best");
    filesystem::create_directories("viz");

    auto run_inst = [&](const string& filename, double tlim) {
        auto pts = load_tsp_instance(filename);
        auto dist = build_distance_matrix(pts);
        auto nearest = buildNearestNeighborsList(dist, 15);
        string inst = filename.substr(filename.find_last_of('/') + 1);
        inst = inst.substr(0, inst.find_last_of('.'));

        Stats s;
        for (int r = 0; r < runs; r++) {
            cout << inst << " HAE-CLS " << (r + 1) << "/" << runs << "...\r" << flush;
            int iters = 0;
            auto t0 = high_resolution_clock::now();
            auto res = solve_hae_cls(dist, pts, tlim, iters, nearest);
            double t = duration<double, milli>(high_resolution_clock::now() - t0).count();
            s.update(calculate_objective(res, dist, pts), t, iters, res);
            save_cycle_to_file("solutions/all/" + inst + "_HAE-CLS_run" + to_string(r) + ".txt",
                               res, dist, pts);
        }
        cout << inst << " HAE-CLS done.        " << endl;
        cout << "  " << inst << ": avg=" << s.avg() << " min=" << s.min_obj << " max=" << s.max_obj
             << " iters=" << s.avg_iters() << " t=" << (int)s.avg_time() << "ms" << endl;

        generate_svg("viz/" + inst + "_HAE-CLS.svg", s.best_tour, pts);
        save_cycle_to_file("solutions/best/" + inst + "_HAE-CLS.txt", s.best_tour, dist, pts);
    };

    cout << ">>> Final eval: solve_hae_cls (runs=" << runs << ")" << endl;
    run_inst("instances/TSPA.csv", TIME_TSPA);
    run_inst("instances/TSPB.csv", TIME_TSPB);
    return 0;
}

int main(int argc, char** argv) {
    srand(42);
    if (argc > 1 && string(argv[1]) == "final") {
        int runs = (argc > 2) ? atoi(argv[2]) : 20;
        return run_final(runs);
    }
    if (argc > 1 && string(argv[1]) == "convergence") {
        int runs = (argc > 2) ? atoi(argv[2]) : 20;
        return run_convergence(runs);
    }
    if (argc > 1 && string(argv[1]) == "pr") {
        int runs = (argc > 2) ? atoi(argv[2]) : 10;
        int pre  = (argc > 3) ? atoi(argv[3]) : 10;
        return run_pr_test(runs, pre);
    }
    if (argc > 1 && string(argv[1]) == "single") {
        // single <runs> <op> <k> <pop> <alns> <memetic> <sa>
        int runs = (argc > 2) ? atoi(argv[2]) : 5;
        int op   = (argc > 3) ? atoi(argv[3]) : 2;
        int k    = (argc > 4) ? atoi(argv[4]) : 10;
        int pop  = (argc > 5) ? atoi(argv[5]) : 20;
        int alns = (argc > 6) ? atoi(argv[6]) : 0;
        int meme = (argc > 7) ? atoi(argv[7]) : 0;
        int sa   = (argc > 8) ? atoi(argv[8]) : 0;
        return run_single(runs, op, k, pop, alns, meme, sa);
    }
    int runs = (argc > 1) ? atoi(argv[1]) : 5;
    if (argc > 2) g_mode = argv[2];
    cout << ">>> mode=" << g_mode << " runs=" << runs << endl;
    run_screening("instances/TSPA.csv", TIME_TSPA, runs);
    run_screening("instances/TSPB.csv", TIME_TSPB, runs);
    return 0;
}
