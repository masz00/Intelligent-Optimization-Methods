import sys
import math

def calculate_distance(p1, p2):
    return round(math.hypot(p1[0] - p2[0], p1[1] - p2[1]))

def verify_solution(instance_file, solution_file):
    points = []
    with open(instance_file, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split(';')
            if len(parts) >= 3:
                x, y, w = int(parts[0]), int(parts[1]), int(parts[2])
                points.append((x, y, w))
            else:
                parts = line.split()
                if len(parts) >= 3:
                    x, y, w = int(parts[0]), int(parts[1]), int(parts[2])
                    points.append((x, y, w))

    num_nodes = len(points)

    # Wczytanie rozwiazania
    try:
        with open(solution_file, 'r') as f:
            lines = f.readlines()
    except FileNotFoundError:
        print(f"Brak pliku rozwiazania: {solution_file}")
        return False

    if not lines:
        print("Plik rozwiazania jest pusty.")
        return False

    try:
        reported_obj = int(lines[0].strip())
        cycle = [int(line.strip()) for line in lines[1:] if line.strip()]
    except ValueError:
        print("Niepoprawny format pliku rozwiazania.")
        return False

    # Sprawdzenie duplikatow
    if len(set(cycle)) != len(cycle):
        print(f"BLAD: Cykl zawiera zduplikowane wierzcholki.")
        return False

    # Sprawdzenie zakresu wierzcholkow
    for node in cycle:
        if node < 0 or node >= num_nodes:
            print(f"BLAD: Wierzcholek poza zakresem: {node}")
            return False

    # Obliczenie odleglosci i zyskow
    calc_profit = 0
    calc_dist = 0

    if len(cycle) > 1:
        for i in range(len(cycle)):
            u = cycle[i]
            v = cycle[(i + 1) % len(cycle)]
            calc_profit += points[u][2]
            calc_dist += calculate_distance(points[u], points[v])

    calc_obj = calc_profit - calc_dist

    if calc_obj != reported_obj:
        print(f"BLAD: Obliczona funkcja celu ({calc_obj}) rozni sie od zadeklarowanej ({reported_obj}).")
        print(f"Zysk: {calc_profit}, Dystans: {calc_dist}")
        return False

    print(f"OK. Rozwiazanie jest poprawne. Funkcja celu: {calc_obj}")
    return True

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Uzycie: python ver_sol.py <instancja> <rozwiazanie>")
        sys.exit(1)
    
    success = verify_solution(sys.argv[1], sys.argv[2])
    sys.exit(0 if success else 1)
