# Intelligent Optimization Methods

Projects completed in collaboration with **[Piotr Szymaczyk](https://github.com/PiotrSzymaczekkk)** as part of the **Intelligent Optimization Methods** course — MSc Computer Science, AI specialisation, Semester 1.

---

## The Problem

All projects tackle a variant of the **Selective (Profitable) Travelling Salesman Problem (S-TSP)** on two benchmark instances: `TSPA` and `TSPB`, each with **200 nodes** in a 2D Euclidean plane.

**Objective:** Select a subset of nodes and find a Hamiltonian cycle minimising the combined cost:

$$\text{cost} = \sum_{\text{edges}} d(i,j) - \sum_{\text{nodes in tour}} w_i$$

The algorithm must balance travel cost against the profit ($w_i$) gained from visiting each node.

---

## Repository Structure

```
.
├── Heuristics/            # Project 1 — Construction heuristics
├── LocalSearch/           # Project 2 — Local search (Piotr's contribution)
└── LocalSearchAdvanced/   # Project 3 — Advanced local search
```

---

## Projects Overview

### 1. `Heuristics` — Construction Heuristics
**Author:** Maciej Szymaniak

Initial approach using pure construction algorithms. Each algorithm is evaluated over **200 starts** (starting from every possible node).

*   **Algorithms:** Random, Nearest Neighbour, Greedy Cycle, 2-Regret, Weighted 2-Regret.
*   **Methodology:** All solutions undergo a **Phase 2 removal pass** to iteratively prune nodes that do not contribute positively to the objective.
*   **Outputs:** Quantitative statistics (min/avg/max) and SVG route visualisations.

### 2. `LocalSearch` — Local Search
**Author:** [Piotr Szymaczyk](https://github.com/PiotrSzymaczekkk/projekt2-io)

Implementation of standard Local Search strategies exploring various neighbourhoods: **node insertion/removal**, **node swap**, and **2-opt edge exchange**.

*   **Strategies:** Steepest Descent, First Improvement (Greedy), and Random Walk.
*   **Goal:** Compare the efficiency of search strategies and their dependence on the quality of starting solutions.

### 3. `LocalSearchAdvanced` — Advanced Local Search
**Author:** Maciej Szymaniak

Focuses on accelerating the Steepest Descent algorithm through advanced neighbourhood management.

*   **Optimisations:** 
    *   **Move List (LM):** Priority-based move management with lazy validation.
    *   **Candidate Moves:** Restricting search to $k=10$ nearest neighbours.
*   **Performance:** Significant reduction in runtime while maintaining high solution quality compared to the exhaustive baseline.

---

## Building & Running

All projects are written in **C++17** and can be compiled using `g++`. No external build systems are required.

### Project 1: Heuristics
```bash
cd Heuristics
g++ -O2 -std=c++17 -o main projekt1.cpp algorithms.cpp tsp_utils.cpp
./main
```

### Project 2: LocalSearch
```bash
cd LocalSearch
g++ -O2 -std=c++17 -o main projekt2.cpp
./main instances/TSPA.csv
```

### Project 3: LocalSearchAdvanced
```bash
cd LocalSearchAdvanced
g++ -O2 -std=c++17 -o main projekt3.cpp tsp_utils.cpp algorithms.cpp
./main
```

---

## Tech Stack

| Component | Technology |
|---|---|
| **Language** | C++17 |
| **Visualisation** | C++ (Native SVG generator) |
| **Data Format** | CSV (Nodes: x, y, profit) |
| **Documentation** | LaTeX & PDF reports |

---

## Authors

*   **Maciej Szymaniak** ([@masz00](https://github.com/masz00)) — Projects 1 & 3
*   **Piotr Szymaczyk** ([@PiotrSzymaczekkk](https://github.com/PiotrSzymaczekkk)) — Project 2
