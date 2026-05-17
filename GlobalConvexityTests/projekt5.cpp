#include "tsp_utils.h"
#include "algorithms.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

using namespace std;

// ILS time limits derived from Lab 4 MSLS measurements
static constexpr double ILS_TIME_TSPA = 1420.0;
static constexpr double ILS_TIME_TSPB = 1578.0;
static constexpr int    LOCAL_RUNS    = 1000;

// ---- Similarity helpers ----

static vector<uint64_t> makeVertexBits(const vector<int>& tour, int n_all) {
    vector<uint64_t> bits((n_all + 63) / 64, 0);
    for (int v : tour) bits[v / 64] |= (1ULL << (v % 64));
    return bits;
}

static vector<uint64_t> makeEdgeBits(const vector<int>& tour, int n_all) {
    int total = n_all * n_all;
    vector<uint64_t> bits((total + 63) / 64, 0);
    int k = tour.size();
    for (int i = 0; i < k; i++) {
        int a = tour[i], b = tour[(i + 1) % k];
        if (a > b) swap(a, b);
        int id = a * n_all + b;
        bits[id / 64] |= (1ULL << (id % 64));
    }
    return bits;
}

static int commonBits(const vector<uint64_t>& a, const vector<uint64_t>& b) {
    int result = 0;
    for (size_t i = 0; i < a.size(); i++)
        result += __builtin_popcountll(a[i] & b[i]);
    return result;
}

static double pearsonCorrelation(const vector<double>& x, const vector<double>& y) {
    int n = x.size();
    double mx = 0, my = 0;
    for (int i = 0; i < n; i++) { mx += x[i]; my += y[i]; }
    mx /= n; my /= n;
    double num = 0, vx = 0, vy = 0;
    for (int i = 0; i < n; i++) {
        double xi = x[i] - mx, yi = y[i] - my;
        num += xi * yi; vx += xi * xi; vy += yi * yi;
    }
    if (vx == 0.0 || vy == 0.0) return 0.0;
    return num / sqrt(vx * vy);
}

// ---- SVG scatter plot ----

struct ScatterPoint { double x, y; };

static void writeScatterSvg(const string& path, const vector<ScatterPoint>& pts, const string& title,
                            const string& xlabel, const string& ylabel) {
    const int W = 900, H = 620;
    const int L = 90, R = 30, T = 55, B = 75;
    const int PW = W - L - R, PH = H - T - B;

    double minX = pts[0].x, maxX = pts[0].x;
    double minY = pts[0].y, maxY = pts[0].y;
    for (auto& p : pts) {
        minX = min(minX, p.x); maxX = max(maxX, p.x);
        minY = min(minY, p.y); maxY = max(maxY, p.y);
    }
    if (fabs(maxX - minX) < 1e-9) { minX -= 1; maxX += 1; }
    if (fabs(maxY - minY) < 1e-9) { minY -= 1; maxY += 1; }

    auto sx = [&](double x) { return L + (x - minX) / (maxX - minX) * PW; };
    auto sy = [&](double y) { return T + (1.0 - (y - minY) / (maxY - minY)) * PH; };

    ofstream out(path);
    out << fixed << setprecision(2);
    out << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << W << "\" height=\"" << H
        << "\" viewBox=\"0 0 " << W << " " << H << "\">\n";
    out << "<rect width=\"100%\" height=\"100%\" fill=\"white\"/>\n";
    out << "<text x=\"" << W/2 << "\" y=\"30\" text-anchor=\"middle\" "
        << "font-family=\"Arial\" font-size=\"18\" fill=\"#111\">" << title << "</text>\n";
    out << "<line x1=\"" << L << "\" y1=\"" << T+PH << "\" x2=\"" << L+PW << "\" y2=\"" << T+PH << "\" stroke=\"#222\"/>\n";
    out << "<line x1=\"" << L << "\" y1=\"" << T << "\" x2=\"" << L << "\" y2=\"" << T+PH << "\" stroke=\"#222\"/>\n";
    out << "<text x=\"" << L+PW/2 << "\" y=\"" << H-20
        << "\" text-anchor=\"middle\" font-family=\"Arial\" font-size=\"13\" fill=\"#111\">" << xlabel << "</text>\n";
    out << "<text x=\"22\" y=\"" << T+PH/2
        << "\" text-anchor=\"middle\" transform=\"rotate(-90 22 " << T+PH/2
        << ")\" font-family=\"Arial\" font-size=\"13\" fill=\"#111\">" << ylabel << "</text>\n";

    for (int i = 0; i <= 5; i++) {
        double t = i / 5.0;
        double gx = L + t * PW, gy = T + PH - t * PH;
        double lx = minX + t * (maxX - minX);
        double ly = minY + t * (maxY - minY);
        out << "<line x1=\"" << gx << "\" y1=\"" << T << "\" x2=\"" << gx << "\" y2=\"" << T+PH << "\" stroke=\"#eee\"/>\n";
        out << "<line x1=\"" << L << "\" y1=\"" << gy << "\" x2=\"" << L+PW << "\" y2=\"" << gy << "\" stroke=\"#eee\"/>\n";
        out << "<text x=\"" << gx << "\" y=\"" << T+PH+20
            << "\" text-anchor=\"middle\" font-family=\"Arial\" font-size=\"10\" fill=\"#444\">"
            << static_cast<long long>(lx) << "</text>\n";
        out << "<text x=\"" << L-8 << "\" y=\"" << gy+4
            << "\" text-anchor=\"end\" font-family=\"Arial\" font-size=\"10\" fill=\"#444\">"
            << setprecision(1) << ly << setprecision(2) << "</text>\n";
    }

    for (auto& p : pts)
        out << "<circle cx=\"" << sx(p.x) << "\" cy=\"" << sy(p.y)
            << "\" r=\"2.8\" fill=\"#1f77b4\" fill-opacity=\"0.55\"/>\n";

    out << "</svg>\n";
}

// ---- CSV writers ----

static void writeSimilarityCsv(const string& path,
                                const vector<int>& objs,
                                const vector<int>& vToBest,
                                const vector<double>& avgVToOther,
                                const vector<int>& eToBest,
                                const vector<double>& avgEToOther) {
    ofstream out(path);
    out << "objective;vertices_to_best;avg_vertices_to_other;edges_to_best;avg_edges_to_other\n";
    out << fixed << setprecision(6);
    for (int i = 0; i < (int)objs.size(); i++)
        out << objs[i] << ';' << vToBest[i] << ';' << avgVToOther[i] << ';'
            << eToBest[i] << ';' << avgEToOther[i] << '\n';
}

// ---- Per-instance analysis ----

static void analyzeInstance(const string& name,
                            const vector<PointData>& points,
                            const vector<vector<int>>& dist,
                            double ils_time_ms,
                            ofstream& summary) {
    int n_all = points.size();
    cout << "[" << name << "] Running ILS (" << ils_time_ms << " ms)..." << flush;
    int ils_iters = 0;
    vector<int> best = solve_ils(dist, points, ils_time_ms, ils_iters);
    int best_obj = calculate_objective(best, dist, points);
    cout << " obj=" << best_obj << ", iters=" << ils_iters << "\n";

    vector<uint64_t> best_vbits = makeVertexBits(best, n_all);
    vector<uint64_t> best_ebits = makeEdgeBits(best, n_all);

    cout << "[" << name << "] Generating " << LOCAL_RUNS << " local optima..." << flush;

    struct Opt {
        int obj;
        vector<uint64_t> vbits;
        vector<uint64_t> ebits;
    };
    vector<Opt> opts;
    opts.reserve(LOCAL_RUNS);

    for (int i = 0; i < LOCAL_RUNS; i++) {
        vector<int> t = solve_random(rand() % n_all, dist, points);
        t = solve_greedy_ls(move(t), dist, points);
        int obj = calculate_objective(t, dist, points);
        opts.push_back({ obj, makeVertexBits(t, n_all), makeEdgeBits(t, n_all) });
        if ((i + 1) % 200 == 0) cout << " " << (i + 1) << flush;
    }
    cout << " done\n";

    // Compute similarity stats
    vector<int>    objs(LOCAL_RUNS), vToBest(LOCAL_RUNS), eToBest(LOCAL_RUNS);
    vector<double> avgVToOther(LOCAL_RUNS), avgEToOther(LOCAL_RUNS);

    cout << "[" << name << "] Computing pairwise similarities..." << flush;
    for (int i = 0; i < LOCAL_RUNS; i++) {
        objs[i]     = opts[i].obj;
        vToBest[i]  = commonBits(opts[i].vbits, best_vbits);
        eToBest[i]  = commonBits(opts[i].ebits, best_ebits);

        long long vsum = 0, esum = 0;
        for (int j = 0; j < LOCAL_RUNS; j++) {
            if (i == j) continue;
            vsum += commonBits(opts[i].vbits, opts[j].vbits);
            esum += commonBits(opts[i].ebits, opts[j].ebits);
        }
        avgVToOther[i] = (double)vsum / (LOCAL_RUNS - 1);
        avgEToOther[i] = (double)esum / (LOCAL_RUNS - 1);
    }
    cout << " done\n";

    // Correlation
    vector<double> objD(LOCAL_RUNS), vBestD(LOCAL_RUNS), eBestD(LOCAL_RUNS);
    for (int i = 0; i < LOCAL_RUNS; i++) {
        objD[i]    = objs[i];
        vBestD[i]  = vToBest[i];
        eBestD[i]  = eToBest[i];
    }
    double corrVBest  = pearsonCorrelation(objD, vBestD);
    double corrAvgV   = pearsonCorrelation(objD, avgVToOther);
    double corrEBest  = pearsonCorrelation(objD, eBestD);
    double corrAvgE   = pearsonCorrelation(objD, avgEToOther);

    int localBest  = *max_element(objs.begin(), objs.end());
    int localWorst = *min_element(objs.begin(), objs.end());
    double localAvg = accumulate(objs.begin(), objs.end(), 0LL) / (double)LOCAL_RUNS;

    // Write CSV
    filesystem::create_directories("solutions");
    filesystem::create_directories("viz");

    string csvPath = "solutions/" + name + "_global_convexity.csv";
    writeSimilarityCsv(csvPath, objs, vToBest, avgVToOther, eToBest, avgEToOther);
    cout << "[" << name << "] Saved " << csvPath << "\n";

    // Build scatter points
    auto makeScatter = [&](bool useEdges, bool useAvg) {
        vector<ScatterPoint> pts(LOCAL_RUNS);
        for (int i = 0; i < LOCAL_RUNS; i++) {
            pts[i].x = objs[i];
            pts[i].y = useAvg
                ? (useEdges ? avgEToOther[i] : avgVToOther[i])
                : (useEdges ? eToBest[i]     : vToBest[i]);
        }
        return pts;
    };

    writeScatterSvg("viz/" + name + "_vertices_to_best.svg",
        makeScatter(false, false), name + ": common vertices with ILS best",
        "objective", "common vertices");
    writeScatterSvg("viz/" + name + "_avg_vertices_to_other.svg",
        makeScatter(false, true), name + ": avg common vertices with other optima",
        "objective", "avg common vertices");
    writeScatterSvg("viz/" + name + "_edges_to_best.svg",
        makeScatter(true, false), name + ": common edges with ILS best",
        "objective", "common edges");
    writeScatterSvg("viz/" + name + "_avg_edges_to_other.svg",
        makeScatter(true, true), name + ": avg common edges with other optima",
        "objective", "avg common edges");
    cout << "[" << name << "] Saved 4 scatter SVGs to viz/\n";

    // Append to summary
    summary << fixed << setprecision(6)
            << name << ';'
            << ils_time_ms << ';'
            << ils_iters << ';'
            << best_obj << ';'
            << localBest << ';'
            << localAvg << ';'
            << localWorst << ';'
            << corrVBest << ';'
            << corrAvgV << ';'
            << corrEBest << ';'
            << corrAvgE << '\n';

    cout << "[" << name << "] corr(obj, v_to_best)=" << corrVBest
         << "  corr(obj, avg_v)=" << corrAvgV
         << "  corr(obj, e_to_best)=" << corrEBest
         << "  corr(obj, avg_e)=" << corrAvgE << "\n\n";
}

int main(int argc, char** argv) {
    string tspa_path = argc > 1 ? argv[1] : "instances/TSPA.csv";
    string tspb_path = argc > 2 ? argv[2] : "instances/TSPB.csv";
    unsigned int seed = argc > 3 ? (unsigned)atoi(argv[3]) : (unsigned)time(nullptr);
    srand(seed);
    cout << "Seed: " << seed << "\n\n";

    auto points_a = load_tsp_instance(tspa_path);
    auto dist_a   = build_distance_matrix(points_a);
    auto points_b = load_tsp_instance(tspb_path);
    auto dist_b   = build_distance_matrix(points_b);

    filesystem::create_directories("solutions");
    ofstream summary("solutions/summary.csv");
    summary << "instance;ils_time_ms;ils_iters;ils_obj;"
            << "best_local_obj;avg_local_obj;worst_local_obj;"
            << "corr_obj_vertices_to_best;corr_obj_avg_vertices;"
            << "corr_obj_edges_to_best;corr_obj_avg_edges\n";

    analyzeInstance("TSPA", points_a, dist_a, ILS_TIME_TSPA, summary);
    analyzeInstance("TSPB", points_b, dist_b, ILS_TIME_TSPB, summary);

    cout << "Done. Summary: solutions/summary.csv\n";
    return 0;
}
