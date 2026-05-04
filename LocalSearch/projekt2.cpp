#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <random>
#include <chrono>
#include <iomanip>
#include <fstream>

using namespace std;

// Struktura wierzchołka - zakładamy, że mamy odległości minimalizować a zysk maksymalizować.
struct Node {
    int id;
    double x, y;
    double profit;
};

// Typy ruchów używane w sąsiedztwie
enum MoveType { ADD, REMOVE, SWAP_NODES, SWAP_EDGES };

struct Move {
    MoveType type;
    int i, j;
    double delta;
};

// Funkcja oceny: f(x) = Dystans - Zysk
// Im mniejsze f(x), tym lepsze rozwiązanie
double calculate_objective(const vector<int>& tour, const vector<Node>& nodes, const vector<vector<double>>& dist) {
    if (tour.size() < 2) return 0.0;
    double d = 0, p = 0;
    for (size_t i = 0; i < tour.size(); ++i) {
        int v = tour[i];
        int next_v = tour[(i + 1) % tour.size()];
        d += dist[v][next_v];
        p += nodes[v].profit;
    }
    return d - p;
}

// Funkcja zapisująca podaną trasę do pliku z odpowiednimi metadanymi dla skryptu Pythona
void save_tour_to_csv(string filename, const vector<int>& tour, const vector<Node>& nodes) {
    ofstream out(filename);
    out << "x,y,is_tour\n"; // Nagłówek pliku

    // Zapisujemy wierzchołki, które NIE są w trasie (aby narysować je jako tło)
    vector<bool> in_tour(nodes.size(), false);
    for(int v : tour) in_tour[v] = true;
    
    for(size_t i = 0; i < nodes.size(); ++i) {
        if(!in_tour[i]) out << nodes[i].x << "," << nodes[i].y << ",0\n";
    }

    // Teraz zapisujemy wierzchołki z trasy w odpowiedniej kolejności tak, by można powiązać je ścieżkami
    for(int v : tour) {
        out << nodes[v].x << "," << nodes[v].y << ",1\n";
    }
    // Zamykamy cykl dodając jeszcze raz pierwszy punkt krzywej by ładnie się rysował obrys
    if(!tour.empty()) {
        out << nodes[tour[0]].x << "," << nodes[tour[0]].y << ",1\n";
    }
    out.close();
}


// Funkcja aplikująca ruch względem podanych węzłów
void apply_move(vector<int>& tour, const Move& m) {
    if (m.type == ADD) {
        tour.insert(tour.begin() + m.i + 1, m.j);
    } else if (m.type == REMOVE) {
        tour.erase(tour.begin() + m.i);
    } else if (m.type == SWAP_NODES) {
        swap(tour[m.i], tour[m.j]);
    } else if (m.type == SWAP_EDGES) {
        int l = m.i + 1, r = m.j;
        while(l < r) {
            swap(tour[l], tour[r]);
            l++; r--;
        }
    }
}

// ==============================================================================
// ZMODYFIKOWANA FUNKCJA: Heurystyka konstrukcyjna 2-Regret (2-żal) + Faza II
// ==============================================================================
vector<int> heuristic_solution(int n, const vector<vector<double>>& dist, const vector<Node>& nodes, int start_node) {
    vector<int> tour;
    vector<bool> visited(n, false);
    tour.push_back(start_node);
    visited[start_node] = true;
    
    // Inicjalizacja cyklu - wybór drugiego wierzchołka uwzględniającego zysk
    int best_second = -1;
    double min_init_cost = 1e18;
    for (int v = 0; v < n; ++v) {
        if (!visited[v]) {
            double cost = dist[start_node][v] * 2 - nodes[v].profit;
            if (cost < min_init_cost) {
                min_init_cost = cost;
                best_second = v;
            }
        }
    }
    tour.push_back(best_second);
    visited[best_second] = true;

    // Faza I - Algorytm 2-Regret
    while (tour.size() < (size_t)n) {
        double max_score = -1e18;
        int best_candidate = -1;
        int best_insert_pos = -1;

        for (int v = 0; v < n; ++v) {
            if (visited[v]) continue;

            double best_cost = 1e18;
            double second_best_cost = 1e18;
            int temp_best_pos = -1;

            // Szukamy dwóch najlepszych miejsc na wstawienie wierzchołka v
            for (size_t i = 0; i < tour.size(); ++i) {
                int a = tour[i];
                int b = tour[(i + 1) % tour.size()];
                // Koszt dodania v między a i b (uwzględnia zysk)
                double cost = dist[a][v] + dist[v][b] - dist[a][b] - nodes[v].profit;

                if (cost < best_cost) {
                    second_best_cost = best_cost;
                    best_cost = cost;
                    temp_best_pos = i;
                } else if (cost < second_best_cost) {
                    second_best_cost = cost;
                }
            }

            // Ocena = żal (regret). 
            // Dodajemy ułamek best_cost do łamania remisów (przydatne przy tour.size() == 2)
            double score = (second_best_cost - best_cost) - 0.0001 * best_cost;

            if (score > max_score) {
                max_score = score;
                best_candidate = v;
                best_insert_pos = temp_best_pos;
            }
        }

        tour.insert(tour.begin() + best_insert_pos + 1, best_candidate);
        visited[best_candidate] = true;
    }
    
    // Faza II - Usuwanie dające największy zysk (zmniejszające f(x))
    bool improved = true;
    while(improved && tour.size() > 3) {
        improved = false;
        int best_remove_idx = -1;
        double best_delta = 0;
        for(size_t i = 0; i < tour.size(); ++i) {
            int v_prev = tour[(i - 1 + tour.size()) % tour.size()];
            int v_curr = tour[i];
            int v_next = tour[(i + 1) % tour.size()];
            // Oblicznie delty z usunięcia węzła (delta dla f = dystans - profit)
            double delta = dist[v_prev][v_next] - dist[v_prev][v_curr] - dist[v_curr][v_next] + nodes[v_curr].profit;
            if(delta < best_delta - 1e-8) {
                best_delta = delta;
                best_remove_idx = i;
            }
        }
        if(best_remove_idx != -1) {
            tour.erase(tour.begin() + best_remove_idx);
            improved = true;
        }
    }
    return tour;
}
// ==============================================================================

// Główna funkcja lokalnego przeszukiwania
vector<int> local_search(vector<int> tour, const vector<vector<double>>& dist, const vector<Node>& nodes, bool is_steepest, bool use_edges, mt19937& rng) {
    bool improved = true;
    while(improved) {
        improved = false;
        vector<Move> moves;
        
        vector<bool> in_tour(nodes.size(), false);
        for(int v : tour) in_tour[v] = true;
        vector<int> unvisited;
        for(size_t v=0; v<nodes.size(); ++v) if(!in_tour[v]) unvisited.push_back(v);
        
        size_t n = tour.size();

        // 1. Ruchy zmiany zbiorów - DODANIE 
        for(size_t i=0; i<n; ++i) {
            int v_prev = tour[i];
            int v_next = tour[(i + 1) % n];
            for(int u : unvisited) {
                double delta = dist[v_prev][u] + dist[u][v_next] - dist[v_prev][v_next] - nodes[u].profit;
                moves.push_back({ADD, (int)i, u, delta});
            }
        }
        
        // 2. Ruchy zmiany zbiorów - USUNIĘCIE
        if(n > 3) {
            for(size_t i=0; i<n; ++i) {
                int v_prev = tour[(i - 1 + n) % n];
                int v_curr = tour[i];
                int v_next = tour[(i + 1) % n];
                double delta = dist[v_prev][v_next] - dist[v_prev][v_curr] - dist[v_curr][v_next] + nodes[v_curr].profit;
                moves.push_back({REMOVE, (int)i, -1, delta});
            }
        }
        
        // 3. Ruchy wewnątrztrasowe
        if (use_edges) {
            for(size_t i=0; i<n; ++i) {
                for(size_t j=i+2; j<n; ++j) {
                    if(i == 0 && j == n - 1) continue;
                    int v_i = tour[i], v_i_next = tour[(i + 1) % n];
                    int v_j = tour[j], v_j_next = tour[(j + 1) % n];
                    double delta = dist[v_i][v_j] + dist[v_i_next][v_j_next] - dist[v_i][v_i_next] - dist[v_j][v_j_next];
                    moves.push_back({SWAP_EDGES, (int)i, (int)j, delta});
                }
            }
        } else {
            for(size_t i=0; i<n; ++i) {
                for(size_t j=i+1; j<n; ++j) {
                    int prev_i = (i - 1 + n) % n;
                    int next_i = (i + 1) % n;
                    int prev_j = (j - 1 + n) % n;
                    int next_j = (j + 1) % n;
                    double delta = 0;
                    if (next_i == j) {
                        delta = dist[tour[prev_i]][tour[j]] + dist[tour[i]][tour[next_j]] - dist[tour[prev_i]][tour[i]] - dist[tour[j]][tour[next_j]];
                    } else if (next_j == i) {
                        delta = dist[tour[prev_j]][tour[i]] + dist[tour[j]][tour[next_i]] - dist[tour[prev_j]][tour[j]] - dist[tour[i]][tour[next_i]];
                    } else {
                        delta = dist[tour[prev_i]][tour[j]] + dist[tour[j]][tour[next_i]] + dist[tour[prev_j]][tour[i]] + dist[tour[i]][tour[next_j]]
                              - dist[tour[prev_i]][tour[i]] - dist[tour[i]][tour[next_i]] - dist[tour[prev_j]][tour[j]] - dist[tour[j]][tour[next_j]];
                    }
                    moves.push_back({SWAP_NODES, (int)i, (int)j, delta});
                }
            }
        }
        
        if (moves.empty()) break;

        // Metoda stroma wybiera globalnie najlepszy ruch z sąsiedztwa
        if (is_steepest) {
            Move best_move = {ADD, -1, -1, 0.0};
            for(const auto& m : moves) {
                if(m.delta < best_move.delta - 1e-8) {
                    best_move = m;
                }
            }
            if(best_move.delta < -1e-8) {
                apply_move(tour, best_move);
                improved = true;
            }
        } else {
            // Metoda zachłanna sprawdza w losowej kolejności do znalezienia PIERWSZEJ poprawy!
            std::shuffle(moves.begin(), moves.end(), rng);
            for(const auto& m : moves) {
                if(m.delta < -1e-8) {
                    apply_move(tour, m);
                    improved = true;
                    break;
                }
            }
        }
    }
    return tour;
}

// Algorytm losowego błądzenia ucinany po limicie czasowym `time_limit`
vector<int> random_walk(vector<int> tour, const vector<vector<double>>& dist, const vector<Node>& nodes, double time_limit, mt19937& rng, bool use_edges) {
    auto start_time = chrono::high_resolution_clock::now();
    vector<int> best_tour = tour;
    double best_obj = calculate_objective(tour, nodes, dist);
    
    while(true) {
        auto now = chrono::high_resolution_clock::now();
        chrono::duration<double> elapsed = now - start_time;
        if (elapsed.count() >= time_limit) break;
        
        vector<bool> in_tour(nodes.size(), false);
        for(int v : tour) in_tour[v] = true;
        vector<int> unvisited;
        for(size_t v=0; v<nodes.size(); ++v) if(!in_tour[v]) unvisited.push_back(v);
        
        int n_add = tour.size() * unvisited.size();
        int n_rem = tour.size() > 3 ? tour.size() : 0;
        int n_intra = 0;
        
        if (use_edges) {
            n_intra = tour.size() * (tour.size() - 3) / 2;
            if (n_intra < 0) n_intra = 0;
        } else {
            n_intra = tour.size() * (tour.size() - 1) / 2;
        }
        
        int total_moves = n_add + n_rem + n_intra;
        if (total_moves == 0) break;
        
        int r = uniform_int_distribution<int>(0, total_moves - 1)(rng);
        Move const_move;
        
        // Wybieranie odpowiedniego typu ruchu z prawdopodobieństem na bazie wielkości puli ruchów N(x)
        if (r < n_add) {
            int i = r / unvisited.size();
            int u_idx = r % unvisited.size();
            const_move = {ADD, i, unvisited[u_idx], 0.0};
        } else if (r < n_add + n_rem) {
            const_move = {REMOVE, r - n_add, -1, 0.0};
        } else {
            if (use_edges) {
                while(true) {
                    int i = uniform_int_distribution<int>(0, tour.size() - 1)(rng);
                    int j = uniform_int_distribution<int>(0, tour.size() - 1)(rng);
                    if (i > j) swap(i, j);
                    if (j >= i + 2 && !(i == 0 && j == tour.size() - 1)) {
                        const_move = {SWAP_EDGES, i, j, 0.0};
                        break;
                    }
                }
            } else {
                while(true) {
                    int i = uniform_int_distribution<int>(0, tour.size() - 1)(rng);
                    int j = uniform_int_distribution<int>(0, tour.size() - 1)(rng);
                    if (i < j) {
                        const_move = {SWAP_NODES, i, j, 0.0};
                        break;
                    }
                }
            }
        }
        
        apply_move(tour, const_move);
        
        double current_obj = calculate_objective(tour, nodes, dist);
        // Akceptujemy każdy wylosowany ruch, ale zapisujemy najlepsze rozwiązanie!
        if (current_obj < best_obj) {
            best_obj = current_obj;
            best_tour = tour;
        }
    }
    return best_tour;
}

// Konfigurator eksperymentu
struct Config {
    string name;
    bool is_steepest;
    bool use_edges;
    bool use_heuristic_start;
};

int main() {
    random_device rd;
    mt19937 rng(1337); // Seed
    int NUM_NODES = 100;
    
    // Gen instancji
    vector<Node> nodes(NUM_NODES);
    vector<vector<double>> dist(NUM_NODES, vector<double>(NUM_NODES, 0));
    uniform_real_distribution<double> coord_dist(0.0, 100.0);
    uniform_real_distribution<double> p_dist(10.0, 50.0);
    
    for(int i=0; i<NUM_NODES; ++i) {
        nodes[i] = {i, coord_dist(rng), coord_dist(rng), p_dist(rng)};
    }
    for(int i=0; i<NUM_NODES; ++i) {
        for(int j=0; j<NUM_NODES; ++j) {
            dist[i][j] = sqrt(pow(nodes[i].x - nodes[j].x, 2) + pow(nodes[i].y - nodes[j].y, 2));
        }
    }

    // 100 Losowych startów
    vector<vector<int>> starts_random(100);
    for(int i=0; i<100; ++i) {
        int k = uniform_int_distribution<int>(5, NUM_NODES)(rng);
        vector<int> start(NUM_NODES);
        iota(start.begin(), start.end(), 0);
        std::shuffle(start.begin(), start.end(), rng);
        start.resize(k);
        starts_random[i] = start;
    }

    // 100 Heurystycznych startów (dla różnorodności każdy zacznyna się od innego węzła z zakresu 0-100)
    vector<vector<int>> starts_heu(100);
    for(int i=0; i<100; ++i) {
        starts_heu[i] = heuristic_solution(NUM_NODES, dist, nodes, i % NUM_NODES);
    }

    vector<Config> configs = {
        {"Steepest _ Swap Nodes _ Random Start", true, false, false},
        {"Steepest _ Swap Nodes _ Heuristic Start", true, false, true},
        {"Steepest _ Swap Edges _ Random Start", true, true, false},
        {"Steepest _ Swap Edges _ Heuristic Start", true, true, true},
        {"Greedy   _ Swap Nodes _ Random Start", false, false, false},
        {"Greedy   _ Swap Nodes _ Heuristic Start", false, false, true},
        {"Greedy   _ Swap Edges _ Random Start", false, true, false},
        {"Greedy   _ Swap Edges _ Heuristic Start", false, true, true}
    };

    double max_avg_time = 0;
    
    cout << "--------------------------------- ALGORYTMY LS (100 uruchomień) ---------------------------------" << endl;
    cout << left << setw(45) << "Nazwa Wariantu" 
         << " | " << setw(22) << "Cel (min/max/avg)"  
         << " | Czas (avg) [ms]" << endl;

    for (const auto& cfg : configs) {
        vector<double> objectives;
        double total_time = 0;
        double min_obj = 1e9;
        vector<int> global_best_tour; // Zmienna śledząca mistrza
        
        for (int i = 0; i < 100; ++i) {
            vector<int> initial = cfg.use_heuristic_start ? starts_heu[i] : starts_random[i];
            
            auto begin_time = chrono::high_resolution_clock::now();
            vector<int> best = local_search(initial, dist, nodes, cfg.is_steepest, cfg.use_edges, rng);
            auto end_time = chrono::high_resolution_clock::now();
            
            chrono::duration<double> duration = end_time - begin_time;
            total_time += duration.count();
            
            double current_obj = calculate_objective(best, nodes, dist);
            objectives.push_back(current_obj);
            
            // Nadpisz jeśli trasa jest lepsza (czyli ma mniejszą funkcję celu dystans-zysk)
            if (current_obj < min_obj) {
                min_obj = current_obj;
                global_best_tour = best;
            }
        }
        
        double avg_t = total_time / 100.0;
        if (avg_t > max_avg_time) max_avg_time = avg_t;
        
        double max_obj = *max_element(objectives.begin(), objectives.end());
        double sum_obj = 0; for(double o : objectives) sum_obj += o;
        double avg_obj = sum_obj / 100.0;
        
        string obj_str = to_string((int)min_obj) + "/" + to_string((int)max_obj) + "/" + to_string((int)avg_obj);
            
        cout << left << setw(45) << cfg.name << " | " << setw(22) << obj_str 
             << " | " << (avg_t * 1000.0) << " ms" << endl;
             
        // Zapis najlepszej marszruty z tego batcha do dysku!
        string safe_name = cfg.name;
        replace(safe_name.begin(), safe_name.end(), ' ', '_'); 
        replace(safe_name.begin(), safe_name.end(), '|', '-');
        save_tour_to_csv("wynik_" + safe_name + ".csv", global_best_tour, nodes);
    }
    
    cout << "\n---------------------------------------- RANDOM WALK (Punkt Odniesienia) -----------------------------------" << endl;
    cout << "> Średni czas najwolniejszej wersji: " << (max_avg_time * 1000.0) << " ms na instancję." << endl;
    
    for(bool use_edges : {false, true}) {
        vector<double> rw_objectives;
        double min_obj = 1e9;
        vector<int> global_best_tour;
        
        for(int i=0; i<100; ++i) {
            vector<int> initial = starts_random[i];
            vector<int> best = random_walk(initial, dist, nodes, max_avg_time, rng, use_edges);
            double current_obj = calculate_objective(best, nodes, dist);
            rw_objectives.push_back(current_obj);
            
            if (current_obj < min_obj) {
                min_obj = current_obj;
                global_best_tour = best;
            }
        }
        
        double sum_obj = 0; for(double o : rw_objectives) sum_obj += o;
        double avg_obj = sum_obj / 100.0;
        
        cout << " - Wersja (" << (use_edges ? "Swap Edges" : "Swap Nodes") << "):  Wynik Średni f(x) = " << avg_obj << endl;
        
        // Zapis do CSV też dla Random Walków
        string rw_safe_name = use_edges ? "Random_Walk_Swap_Edges" : "Random_Walk_Swap_Nodes";
        save_tour_to_csv("wynik_" + rw_safe_name + ".csv", global_best_tour, nodes);
    }

    return 0;
}