#include <iostream>
#include <vector>
#include <chrono>
#include <algorithm>
#include <iomanip>
#include <fstream>
#include <random>
#include "tsp_utils.h"
#include "algorithms.h"

using namespace std;
using namespace std::chrono;

struct Stats {
    long long min_obj = -1, max_obj = -1;
    double avg_obj = 0;
    double avg_time_ms = 0;
    vector<int> best_tour;
};

void run_experiment(const string& filename) {
    auto points = load_tsp_instance(filename);
    auto dist = build_distance_matrix(points);
    int n = points.size();

    vector<string> alg_names = {"Steepest LS", "Move List", "Candidate Moves", "Regret"};
    vector<Stats> results(alg_names.size());

    cout << "\nExperimenting on instance: " << filename << endl;
    cout << string(80, '-') << endl;
    cout << left << setw(20) << "Algorithm" << " | " 
         << setw(30) << "Objective (Min - Max)" << " | " 
         << setw(15) << "Avg Time (ms)" << endl;
    cout << string(80, '-') << endl;

    for (int alg_idx = 0; alg_idx < alg_names.size(); alg_idx++) {
        long long sum_obj = 0;
        double sum_time = 0;
        results[alg_idx].min_obj = 2e18;
        results[alg_idx].max_obj = -2e18;

        for (int i = 0; i < 100; i++) {
            vector<int> tour;
            auto start = high_resolution_clock::now();
            
            if (alg_idx < 3) {
                vector<int> initial;
                for(int j=0; j<n; j++) initial.push_back(j);
                std::mt19937 g(i);
                shuffle(initial.begin(), initial.end(), g);
                initial.resize((n+1)/2);

                if (alg_idx == 0) tour = solve_steepest_baseline(initial, dist, points);
                else if (alg_idx == 1) tour = solve_steepest_lm(initial, dist, points);
                else if (alg_idx == 2) tour = solve_steepest_candidate(initial, dist, points, 10);
            } else {
                tour = solve_regret(i % n, true, 1.0, 0.5, dist, points);
                tour = phase2_remove(tour, dist, points);
            }

            auto end = high_resolution_clock::now();
            long long obj = calculate_objective(tour, dist, points);
            double time_ms = duration<double, milli>(end - start).count();

            sum_obj += obj;
            sum_time += time_ms;
            
            if (obj > results[alg_idx].max_obj) {
                results[alg_idx].max_obj = obj;
                results[alg_idx].best_tour = tour;
            }
            if (obj < results[alg_idx].min_obj) {
                results[alg_idx].min_obj = obj;
            }
        }

        results[alg_idx].avg_obj = (double)sum_obj / 100.0;
        results[alg_idx].avg_time_ms = sum_time / 100.0;

        string obj_str = to_string((int)results[alg_idx].avg_obj) + " (" + to_string(results[alg_idx].min_obj) + " - " + to_string(results[alg_idx].max_obj) + ")";
        cout << left << setw(20) << alg_names[alg_idx] << " | " 
             << setw(30) << obj_str << " | " 
             << setw(15) << fixed << setprecision(2) << results[alg_idx].avg_time_ms << endl;

        // Generate SVG for the best tour
        string instance_name = filename.substr(filename.find_last_of('/') + 1);
        instance_name = instance_name.substr(0, instance_name.find('.'));
        string svg_filename = "viz/" + instance_name + "_" + alg_names[alg_idx] + ".svg";
        for(char &c : svg_filename) if(c == ' ' && &c > &svg_filename[3]) c = '_';
        generate_svg(svg_filename, results[alg_idx].best_tour, points);
    }
}

int main() {
    srand(42);
    run_experiment("instances/TSPA.csv");
    run_experiment("instances/TSPB.csv");
    return 0;
}
