#include <iostream>
#include <vector>
#include <chrono>
#include <algorithm>
#include <iomanip>
#include <fstream>
#include <filesystem>
#include <cstdlib>
#include <ctime>
#include "tsp_utils.h"
#include "algorithms.h"

using namespace std;
using namespace chrono;

static constexpr double TIME_TSPA = 1420.0;
static constexpr double TIME_TSPB = 1578.0;
static constexpr int    RUNS      = 20;

struct Stats {
    long long min_obj = 2e18, max_obj = -2e18;
    long long sum_obj = 0;
    double    sum_time_ms = 0;
    long long sum_iters   = 0;
    vector<int> best_tour;

    void update(long long obj, double time_ms, int iters, const vector<int>& tour) {
        if (obj < min_obj) min_obj = obj;
        if (obj > max_obj) { max_obj = obj; best_tour = tour; }
        sum_obj      += obj;
        sum_time_ms  += time_ms;
        sum_iters    += iters;
    }
};

void run_task6(const string& filename, double time_limit_ms) {
    auto points = load_tsp_instance(filename);
    auto dist   = build_distance_matrix(points);

    string inst = filename.substr(filename.find_last_of('/') + 1);
    inst = inst.substr(0, inst.find_last_of('.'));

    cout << "\n=== Badanie instancji: " << inst << " ===" << endl;

    filesystem::create_directories("solutions/all");
    filesystem::create_directories("solutions/best");
    filesystem::create_directories("viz");

    Stats s_greedy, s_basls;
    Stats s_hae1, s_hae2, s_hae2nls, s_hae3, s_hae3nls;

    // Greedy construction baseline (repair_lns + phase2_remove, no LS)
    for (int i = 0; i < RUNS; i++) {
        cout << "Greedy " << i+1 << "/" << RUNS << "...\r" << flush;
        auto t0  = high_resolution_clock::now();
        auto res = solve_greedy_construct(dist, points);
        double t = duration<double, milli>(high_resolution_clock::now() - t0).count();
        s_greedy.update(calculate_objective(res, dist, points), t, 1, res);
        save_cycle_to_file("solutions/all/" + inst + "_Greedy_run" + to_string(i) + ".txt", res, dist, points);
    }

    // Baseline local search (random start + steepest baseline)
    for (int i = 0; i < RUNS; i++) {
        cout << "BasLS  " << i+1 << "/" << RUNS << "...\r" << flush;
        auto t0  = high_resolution_clock::now();
        auto res = solve_steepest_baseline(solve_random(rand() % points.size(), dist, points), dist, points);
        double t = duration<double, milli>(high_resolution_clock::now() - t0).count();
        s_basls.update(calculate_objective(res, dist, points), t, 1, res);
        save_cycle_to_file("solutions/all/" + inst + "_BasLS_run" + to_string(i) + ".txt", res, dist, points);
    }

    // HAE variants
    auto run_hae = [&](Stats& s, const string& name, int op, bool use_ls) {
        for (int i = 0; i < RUNS; i++) {
            cout << name << " " << i+1 << "/" << RUNS << "...\r" << flush;
            int iters = 0;
            auto t0  = high_resolution_clock::now();
            auto res = solve_hae(dist, points, time_limit_ms, iters, op, use_ls);
            double t = duration<double, milli>(high_resolution_clock::now() - t0).count();
            s.update(calculate_objective(res, dist, points), t, iters, res);
            save_cycle_to_file("solutions/all/" + inst + "_" + name + "_run" + to_string(i) + ".txt", res, dist, points);
        }
        cout << name << " gotowe.              " << endl;
    };

    run_hae(s_hae1,    "HAE1",    1, true);
    run_hae(s_hae2,    "HAE2",    2, true);
    run_hae(s_hae2nls, "HAE2nLS", 2, false);
    run_hae(s_hae3,    "HAE3",    3, true);
    run_hae(s_hae3nls, "HAE3nLS", 3, false);

    // ---- Print results ----
    cout << "\nTabela 1: Wyniki funkcji celu i iteracje" << endl;
    cout << left << setw(12) << "Metoda"
         << " | " << setw(40) << "Srednia (min - max)"
         << " | " << "Sr. Iteracji" << endl;
    cout << string(72, '-') << endl;

    auto print_row = [&](const string& name, const Stats& s) {
        string range = to_string((int)(s.sum_obj / RUNS))
            + " (" + to_string(s.min_obj) + " - " + to_string(s.max_obj) + ")";
        cout << left << setw(12) << name
             << " | " << setw(40) << range
             << " | " << (s.sum_iters / RUNS) << endl;
    };

    print_row("Greedy",   s_greedy);
    print_row("BasLS",    s_basls);
    print_row("HAE1",     s_hae1);
    print_row("HAE2",     s_hae2);
    print_row("HAE2nLS",  s_hae2nls);
    print_row("HAE3",     s_hae3);
    print_row("HAE3nLS",  s_hae3nls);

    cout << "\nTabela 2: Srednie czasy obliczen [ms]" << endl;
    cout << left << setw(12) << "Metoda" << " | " << "Czas [ms]" << endl;
    cout << string(30, '-') << endl;

    auto print_time = [&](const string& name, const Stats& s) {
        cout << left << setw(12) << name
             << " | " << fixed << setprecision(1) << (s.sum_time_ms / RUNS) << endl;
    };

    print_time("Greedy",   s_greedy);
    print_time("BasLS",    s_basls);
    print_time("HAE1",     s_hae1);
    print_time("HAE2",     s_hae2);
    print_time("HAE2nLS",  s_hae2nls);
    print_time("HAE3",     s_hae3);
    print_time("HAE3nLS",  s_hae3nls);

    // Save best solutions and SVGs
    auto save_best = [&](const string& name, const Stats& s) {
        generate_svg("viz/" + inst + "_" + name + ".svg", s.best_tour, points);
        save_cycle_to_file("solutions/best/" + inst + "_" + name + ".txt", s.best_tour, dist, points);
    };

    save_best("Greedy",   s_greedy);
    save_best("BasLS",    s_basls);
    save_best("HAE1",     s_hae1);
    save_best("HAE2",     s_hae2);
    save_best("HAE2nLS",  s_hae2nls);
    save_best("HAE3",     s_hae3);
    save_best("HAE3nLS",  s_hae3nls);

    cout << "Gotowe dla " << inst << "!" << endl;
}

int main() {
    srand(42);
    run_task6("instances/TSPA.csv", TIME_TSPA);
    run_task6("instances/TSPB.csv", TIME_TSPB);
    return 0;
}
