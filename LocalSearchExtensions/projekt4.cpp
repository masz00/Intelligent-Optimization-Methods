#include <iostream>
#include <vector>
#include <chrono>
#include <algorithm>
#include <iomanip>
#include <fstream>
#include <random>
#include "tsp_utils.h"
#include "algorithms.h"
#include <cstdlib>
#include <ctime>

using namespace std;
using namespace std::chrono;
vector<PointData> nodes; 
vector<vector<int>> dist_matrix;
struct Stats {
    long long min_obj = 2e18, max_obj = -2e18;
    long long sum_obj = 0;
    double sum_time_ms = 0;
    long long sum_iters = 0;
    vector<int> best_tour;

    void update(long long obj, double time_ms, int iters, const vector<int>& tour) {
    if (obj < min_obj) min_obj = obj;
    if (obj > max_obj) {
        max_obj = obj;
        best_tour = tour;
    }
    
    sum_obj += obj;
    sum_time_ms += time_ms;
    sum_iters += iters;
}
};


   

void run_task4(const string& filename) {
    auto points = load_tsp_instance(filename);
    auto dist = build_distance_matrix(points);

    string inst_name = filename.substr(filename.find_last_of('/') + 1);
    inst_name = inst_name.substr(0, inst_name.find_last_of('.'));
    
    cout << "\n=== Badanie instancji: " << filename << " ===" << endl;

    Stats s_msls, s_ils, s_lns, s_lnsa;

    // MSLS
    for (int i = 0; i < 20; i++) {
        cout << "MSLS " << i + 1 << "/20..." << flush << "\r";
        double t;
        auto res = solve_msls(dist, points, t);
        s_msls.update(calculate_objective(res, dist, points), t, 200, res);
        save_cycle_to_file("solutions/all/" + inst_name + "_MSLS_run" + to_string(i) + ".txt", res, dist, points);
    }
    double avg_time_limit = s_msls.sum_time_ms / 20.0;

    // ILS
    for (int i = 0; i < 20; i++) {
        cout << "ILS " << i + 1 << "/20..." << flush << "\r";
        int iters;
        auto t0 = high_resolution_clock::now();
        auto res = solve_ils(dist, points, avg_time_limit, iters);
        double t = duration<double, milli>(high_resolution_clock::now() - t0).count();
        long long obj = calculate_objective(res, dist, points);
        s_ils.update(obj, t, iters, res);
        save_cycle_to_file("solutions/all/" + inst_name + "_ILS_run" + to_string(i) + ".txt", res, dist, points);
        verify_and_print_solution(res, dist, points, obj, "ILS_Run_" + to_string(i));
    }

    // LNS
    for (int i = 0; i < 20; i++) {
        cout << "LNS " << i + 1 << "/20..." << flush << "\r";
        int iters;
        auto t0 = high_resolution_clock::now();
        auto res = solve_lns(dist, points, avg_time_limit, iters, true);
        double t = duration<double, milli>(high_resolution_clock::now() - t0).count();
        s_lns.update(calculate_objective(res, dist, points), t, iters, res);
        save_cycle_to_file("solutions/all/" + inst_name + "_LNS_run" + to_string(i) + ".txt", res, dist, points);
    }

    // LNSa (bez SLS po fazie Repair)
    for (int i = 0; i < 20; i++) {
        cout << "LNSa " << i + 1 << "/20..." << flush << "\r";
        int iters;
        auto t0 = high_resolution_clock::now();
        auto res = solve_lns(dist, points, avg_time_limit, iters, false);
        double t = duration<double, milli>(high_resolution_clock::now() - t0).count();
        s_lnsa.update(calculate_objective(res, dist, points), t, iters, res);
        save_cycle_to_file("solutions/all/" + inst_name + "_LNSa_run" + to_string(i) + ".txt", res, dist, points);
    }

    cout << "\nTabela 1: Wyniki funkcji celu i iteracje" << endl;
    cout << left << setw(10) << "Metoda" << " | " << setw(35) << "Srednia (min - max)" << " | " << "Sr. Iteracji" << endl;
    cout << string(65, '-') << endl;

    auto print_tab1 = [](string name, const Stats& s) {
        string obj_range = to_string((int)(s.sum_obj/20)) + " (" + to_string(s.min_obj) + " - " + to_string(s.max_obj) + ")";
        cout << left << setw(10) << name << " | " << setw(35) << obj_range << " | " << (s.sum_iters/20) << endl;
    };

    print_tab1("MSLS", s_msls);
    print_tab1("ILS", s_ils);
    print_tab1("LNS", s_lns);
    print_tab1("LNSa", s_lnsa);

    cout << "\nTabela 2: Srednie czasy obliczen [ms]" << endl;
    cout << left << setw(20) << "Metoda" << " | " << "Czas [ms]" << endl;
    cout << string(35, '-') << endl;
    cout << left << setw(20) << "MSLS (Benchmark)" << " | " << (s_msls.sum_time_ms/20) << endl;
    cout << left << setw(20) << "ILS" << " | " << (s_ils.sum_time_ms/20) << endl;
    cout << left << setw(20) << "LNS" << " | " << (s_lns.sum_time_ms/20) << endl;
    cout << left << setw(20) << "LNSa" << " | " << (s_lnsa.sum_time_ms/20) << endl;

    cout << "\nZapisywanie wizualizacji do folderu viz/ i rozwiazan do solutions/..." << endl;

    generate_svg("viz/" + inst_name + "_MSLS.svg", s_msls.best_tour, points);
    generate_svg("viz/" + inst_name + "_ILS.svg", s_ils.best_tour, points);
    generate_svg("viz/" + inst_name + "_LNS.svg", s_lns.best_tour, points);
    generate_svg("viz/" + inst_name + "_LNSa.svg", s_lnsa.best_tour, points);

    save_cycle_to_file("solutions/best/" + inst_name + "_MSLS.txt", s_msls.best_tour, dist, points);
    save_cycle_to_file("solutions/best/" + inst_name + "_ILS.txt", s_ils.best_tour, dist, points);
    save_cycle_to_file("solutions/best/" + inst_name + "_LNS.txt", s_lns.best_tour, dist, points);
    save_cycle_to_file("solutions/best/" + inst_name + "_LNSa.txt", s_lnsa.best_tour, dist, points);

    cout << "Gotowe dla " << inst_name << "!" << endl;
}

int main() {
    srand(42);
    run_task4("instances/TSPA.csv");
    run_task4("instances/TSPB.csv");
    return 0;
}
