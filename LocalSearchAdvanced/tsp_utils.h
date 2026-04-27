#pragma once
#include <vector>
#include <string>

struct PointData {
    int id;           
    int x;            
    int y;            
    int node_weight;  
};

std::vector<PointData> load_tsp_instance(const std::string &filepath);
std::vector<std::vector<int>> build_distance_matrix(const std::vector<PointData> &points);

int calculate_objective(const std::vector<int> &cycle, 
                       const std::vector<std::vector<int>> &dist_matrix, 
                       const std::vector<PointData> &points);

void generate_svg(const std::string &filename, const std::vector<int> &cycle, const std::vector<PointData> &points);
void print_algorithm_statistics(const std::string &algo_name, const std::vector<int> &phase1_lengths, const std::vector<int> &phase2_objectives);
void save_cycle_to_file(const std::string &filename, const std::vector<int> &cycle,
                        const std::vector<std::vector<int>> &dist_matrix,
                        const std::vector<PointData> &points);
