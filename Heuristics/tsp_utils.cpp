#include "tsp_utils.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <numeric>

using namespace std;

vector<PointData> load_tsp_instance(const string &filepath) {
    ifstream file(filepath);
    if (!file.is_open()) {
        cerr << "BLAD: Nie moge otworzyc pliku instancji: " << filepath << "\n";
        exit(1);
    }

    vector<PointData> dataset;
    string current_line;
    int index = 0;

    while (getline(file, current_line)) {
        if (current_line.empty()) continue;
        replace(current_line.begin(), current_line.end(), ';', ' ');
        stringstream parser(current_line);
        int px, py, pw;
        if (parser >> px >> py >> pw) {
            dataset.push_back({index++, px, py, pw});
        }
    }
    return dataset;
}

vector<vector<int>> build_distance_matrix(const vector<PointData> &points) {
    int total_nodes = points.size();
    vector<vector<int>> matrix(total_nodes, vector<int>(total_nodes, 0));

    for (int i = 0; i < total_nodes; i++) {
        for (int j = 0; j < total_nodes; j++) {
            if (i == j) {
                matrix[i][j] = 0;
            } else {
                double diff_x = points[i].x - points[j].x;
                double diff_y = points[i].y - points[j].y;
                matrix[i][j] = static_cast<int>(round(hypot(diff_x, diff_y)));
            }
        }
    }
    return matrix;
}

int calculate_objective(const vector<int> &cycle, const vector<vector<int>> &dist_matrix, const vector<PointData> &points) {
    if (cycle.empty()) {
        int total_weight = 0;
        for (const auto &p : points) total_weight += p.node_weight;
        return total_weight; // Skrajny przypadek: wszystkie wezly to kara
    }

    int distance_sum = 0;
    int cycle_length = cycle.size();

    // Suma dlugosci cyklu
    if (cycle_length > 1) {
        for (int step = 0; step < cycle_length; step++) {
            int from_node = cycle[step];
            int to_node = cycle[(step + 1) % cycle_length];
            distance_sum += dist_matrix[from_node][to_node];
        }
    }

    // Kara za pominiete wierzcholki (nie bedace w cyklu)
    vector<bool> in_cycle(points.size(), false);
    for (int node_idx : cycle) {
        in_cycle[node_idx] = true;
    }

    int penalty_sum = 0;
    for (size_t i = 0; i < points.size(); i++) {
        if (!in_cycle[i]) {
            penalty_sum += points[i].node_weight;
        }
    }

    return distance_sum + penalty_sum;
}

void generate_svg(const string &filename, const vector<int> &cycle, const vector<PointData> &points) {
    if (points.empty()) return;

    int bound_min_x = points[0].x, bound_max_x = points[0].x;
    int bound_min_y = points[0].y, bound_max_y = points[0].y;
    int min_w = points[0].node_weight, max_w = points[0].node_weight;

    for (const auto &pt : points) {
        bound_min_x = min(bound_min_x, pt.x);
        bound_max_x = max(bound_max_x, pt.x);
        bound_min_y = min(bound_min_y, pt.y);
        bound_max_y = max(bound_max_y, pt.y);
        min_w = min(min_w, pt.node_weight);
        max_w = max(max_w, pt.node_weight);
    }

    const int WIN_W = 1920;
    const int WIN_H = 1080;
    const int MARGIN = 40;

    double s_x = (double)(WIN_W - 2 * MARGIN) / (bound_max_x - bound_min_x + 1);
    double s_y = (double)(WIN_H - 2 * MARGIN) / (bound_max_y - bound_min_y + 1);
    double global_scale = min(s_x, s_y);

    auto get_svg_x = [&](int val) { return MARGIN + (val - bound_min_x) * global_scale; };
    auto get_svg_y = [&](int val) { return WIN_H - MARGIN - (val - bound_min_y) * global_scale; };
    
    auto get_radius = [&](int w) {
        if (max_w == min_w) return 6.0;
        return 3.0 + ((double)(w - min_w) / (max_w - min_w)) * 12.0;
    };

    ofstream out(filename);
    if (!out) return;

    out << "<svg width=\"" << WIN_W << "\" height=\"" << WIN_H << "\" xmlns=\"http://www.w3.org/2000/svg\">\n";
    out << "  <rect width=\"100%\" height=\"100%\" fill=\"#EAEAEA\"/>\n";

    auto matrix = build_distance_matrix(points);
    int cost = calculate_objective(cycle, matrix, points);
    int total_profit = 0;
    for (const auto &p : points) total_profit += p.node_weight;
    int objective = total_profit - cost;

    out << "  <text x=\"" << MARGIN << "\" y=\"" << MARGIN / 2.0 << "\" font-family=\"monospace\" font-size=\"14\" fill=\"#111\">\n";
    out << "    Funkcja celu: " << objective << " | Wierzcholki w cyklu: " << cycle.size() << " / " << points.size() << "\n";
    out << "  </text>\n";

    int nC = cycle.size();
    if (nC > 1) {
        for (int i = 0; i < nC; ++i) {
            int u = cycle[i];
            int v = cycle[(i + 1) % nC];

            out << "  <line x1=\"" << get_svg_x(points[u].x) << "\" y1=\"" << get_svg_y(points[u].y) 
                << "\" x2=\"" << get_svg_x(points[v].x) << "\" y2=\"" << get_svg_y(points[v].y) << "\"\n"
                << "        stroke=\"#0066CC\" stroke-width=\"2\" stroke-dasharray=\"5,3\"/>\n";
        }
    }

    int root_node = cycle.empty() ? -1 : cycle[0];
    for (size_t i = 0; i < points.size(); ++i) {
        double cx = get_svg_x(points[i].x);
        double cy = get_svg_y(points[i].y);
        double r = get_radius(points[i].node_weight);

        bool is_selected = (find(cycle.begin(), cycle.end(), i) != cycle.end());

        string fill_c = "#D3D3D3"; string stroke_c = "#A9A9A9"; double line_w = 1.0; 
        
        if (i == root_node) {
            fill_c = "#32CD32"; stroke_c = "#006400"; line_w = 3.0; // Poczatek cyklu
        } else if (is_selected) {
            fill_c = "#FF4500"; stroke_c = "#8B0000"; line_w = 1.5; // Środek cyklu
        } else {
            fill_c = "#555555"; stroke_c = "#333333"; line_w = 1.0; // Odrzucony (ukarany)
        }

        out << "  <circle cx=\"" << cx << "\" cy=\"" << cy << "\" r=\"" << r << "\"\n"
            << "          fill=\"" << fill_c << "\" stroke=\"" << stroke_c << "\" stroke-width=\"" << line_w << "\">\n"
            << "    <title>Wezel " << i << " (Waga: " << points[i].node_weight << ")</title>\n"
            << "  </circle>\n";
    }

    out << "</svg>\n";
    out.close();
}

void print_algorithm_statistics(const string &algo_name, const vector<int> &phase1_lengths, const vector<int> &phase2_objectives) {
    if (phase2_objectives.empty()) return;

    int p1_min = *min_element(phase1_lengths.begin(), phase1_lengths.end());
    int p1_max = *max_element(phase1_lengths.begin(), phase1_lengths.end());
    long double p1_avg = accumulate(phase1_lengths.begin(), phase1_lengths.end(), 0.0L) / phase1_lengths.size();

    int obj_min = *min_element(phase2_objectives.begin(), phase2_objectives.end());
    int obj_max = *max_element(phase2_objectives.begin(), phase2_objectives.end());
    long double obj_avg = accumulate(phase2_objectives.begin(), phase2_objectives.end(), 0.0L) / phase2_objectives.size();

    cout << "\n=== " << algo_name << " ===\n";
    cout << "  Faza I  - dlugosc sciezki : " << (int)p1_avg << " (" << p1_min << " - " << p1_max << ")\n";
    cout << "  Faza II - funkcja celu    : " << (int)obj_avg << " (" << obj_min << " - " << obj_max << ")\n";
    cout << "=================================================\n";
}

void save_cycle_to_file(const string &filename, const vector<int> &cycle,
                        const vector<vector<int>> &dist_matrix,
                        const vector<PointData> &points) {
    ofstream out(filename);
    if (out.is_open()) {
        int cost = calculate_objective(cycle, dist_matrix, points);
        int total_profit = 0;
        for (const auto &p : points) total_profit += p.node_weight;
        int objective = total_profit - cost;

        out << objective << "\n";
        for (size_t i = 0; i < cycle.size(); ++i) {
            out << cycle[i] << "\n";
        }
        out.close();
    }
}
