#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <random>
#include <algorithm>
#include <numeric>
#include <climits>
#include <cstdlib>
#include <ctime>
#include "tsp_utils.h"
#include "algorithms.h"

using namespace std;

struct AlgoResult
{
    string name;
    vector<int> phase1_costs;
    vector<int> phase2_costs;
    vector<int> best_tour;
    int best_objective;
    int best_start;
};

AlgoResult run_algo(const string &name,
                    const vector<vector<int>> &dist,
                    const vector<PointData> &points,
                    int total_profit,
                    auto algo_func)
{
    int n = points.size();
    AlgoResult res;
    res.name = name;
    res.best_objective = INT_MIN;
    res.best_start = 0;

    for (int s = 0; s < n; s++)
    {
        vector<int> tour = algo_func(s);

        int cost1 = calculate_objective(tour, dist, points);
        int obj1 = total_profit - cost1;
        res.phase1_costs.push_back(obj1);

        tour = phase2_remove(tour, dist, points);
        int cost2 = calculate_objective(tour, dist, points);
        int obj2 = total_profit - cost2;
        res.phase2_costs.push_back(obj2);

        if (obj2 > res.best_objective)
        {
            res.best_objective = obj2;
            res.best_tour = tour;
            res.best_start = s;
        }
    }
    return res;
}

vector<AlgoResult> run_instance(const vector<vector<int>> &dist,
                                const vector<PointData> &points,
                                int total_profit)
{
    int n = points.size();
    vector<AlgoResult> results;

    // 1. Random - 200 uruchomien
    {
        AlgoResult res;
        res.name = "Random";
        res.best_objective = INT_MIN;
        res.best_start = 0;
        mt19937 rng(42);

        for (int i = 0; i < 200; i++)
        {
            int start = rng() % n;
            auto tour = solve_random(start, dist, points);

            int cost1 = calculate_objective(tour, dist, points);
            int obj1 = total_profit - cost1;
            res.phase1_costs.push_back(obj1);

            tour = phase2_remove(tour, dist, points);
            int cost2 = calculate_objective(tour, dist, points);
            int obj2 = total_profit - cost2;
            res.phase2_costs.push_back(obj2);

            if (obj2 > res.best_objective)
            {
                res.best_objective = obj2;
                res.best_tour = tour;
                res.best_start = i;
            }
        }
        results.push_back(res);
    }

    results.push_back(run_algo("NNa", dist, points, total_profit,
                               [&](int s)
                               { return solve_nn(s, false, dist, points); }));
    results.push_back(run_algo("NNb", dist, points, total_profit,
                               [&](int s)
                               { return solve_nn(s, true, dist, points); }));
    results.push_back(run_algo("GCa", dist, points, total_profit,
                               [&](int s)
                               { return solve_greedy_cycle(s, false, dist, points); }));
    results.push_back(run_algo("GCb", dist, points, total_profit,
                               [&](int s)
                               { return solve_greedy_cycle(s, true, dist, points); }));
    results.push_back(run_algo("RegretGCa", dist, points, total_profit,
                               [&](int s)
                               { return solve_regret(s, false, 1.0, 0.0, dist, points); }));
    results.push_back(run_algo("RegretGCb", dist, points, total_profit,
                               [&](int s)
                               { return solve_regret(s, true, 1.0, 0.0, dist, points); }));
    results.push_back(run_algo("WRegretGCa", dist, points, total_profit,
                               [&](int s)
                               { return solve_regret(s, false, 1.0, 1.0, dist, points); }));
    results.push_back(run_algo("WRegretGCb", dist, points, total_profit,
                               [&](int s)
                               { return solve_regret(s, true, 1.0, 1.0, dist, points); }));

    return results;
}

void print_stats_col(const vector<int> &vals)
{
    int mn = *min_element(vals.begin(), vals.end());
    int mx = *max_element(vals.begin(), vals.end());
    double avg = accumulate(vals.begin(), vals.end(), 0.0) / vals.size();
    cout << setw(8) << (int)avg << " (" << setw(7) << mn << " - " << setw(7) << mx << ")";
}

int main()
{
    int num_algos = 9;

    auto pointsA = load_tsp_instance("instances/TSPA.csv");
    auto distA = build_distance_matrix(pointsA);
    int profitA = 0;
    for (auto &p : pointsA)
        profitA += p.node_weight;

    auto pointsB = load_tsp_instance("instances/TSPB.csv");
    auto distB = build_distance_matrix(pointsB);
    int profitB = 0;
    for (auto &p : pointsB)
        profitB += p.node_weight;

    // Uruchom algorytmy na obu instancjach
    auto resA = run_instance(distA, pointsA, profitA);
    auto resB = run_instance(distB, pointsB, profitB);

    // Tabela Faza I
    cout << "Wyniki po I fazie\n";
    cout << "srednia (min - max)\n";
    cout << setw(14) << left << "Metoda" << right
         << "TSPA                          TSPB\n";
    cout << string(74, '-') << "\n";
    for (int i = 0; i < num_algos; i++)
    {
        cout << setw(14) << left << resA[i].name << right;
        print_stats_col(resA[i].phase1_costs);
        cout << "   ";
        print_stats_col(resB[i].phase1_costs);
        cout << "\n";
    }

    cout << "\nWyniki po II fazie\n";
    cout << "srednia (min - max)\n";
    cout << setw(14) << left << "Metoda" << right
         << "TSPA                          TSPB\n";
    cout << string(74, '-') << "\n";
    for (int i = 0; i < num_algos; i++)
    {
        cout << setw(14) << left << resA[i].name << right;
        print_stats_col(resA[i].phase2_costs);
        cout << "   ";
        print_stats_col(resB[i].phase2_costs);
        cout << "\n";
    }

    cout << "\nNajlepsze rozwiazania\n";
    for (int i = 0; i < num_algos; i++)
    {
        cout << "" << setw(14) << left << resA[i].name << right
             << "TSPA: wynik:" << setw(7) << resA[i].best_objective
             << "  n=" << setw(3) << resA[i].best_tour.size()
             << "    TSPB: wynik:" << setw(7) << resB[i].best_objective
             << "  n=" << setw(3) << resB[i].best_tour.size() << "\n";

        generate_svg("viz/viz_TSPA_" + resA[i].name + ".svg", resA[i].best_tour, pointsA);
        generate_svg("viz/viz_TSPB_" + resB[i].name + ".svg", resB[i].best_tour, pointsB);
    }

    // Zapisz rozwiazania dla kazdej kombinacji instancja x algorytm
    for (int i = 0; i < num_algos; i++)
    {
        save_cycle_to_file("solution/solution_TSPA_" + resA[i].name + ".txt", resA[i].best_tour, distA, pointsA);
        save_cycle_to_file("solution/solution_TSPB_" + resB[i].name + ".txt", resB[i].best_tour, distB, pointsB);
    }

    return 0;
}
