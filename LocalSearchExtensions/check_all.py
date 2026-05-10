import os
import sys
from ver_sol import verify_solution

def check_all():
    instances_dir = "instances"
    solutions_dir = os.path.join("solutions", "all")
    
    if not os.path.exists(instances_dir) or not os.path.exists(solutions_dir):
        print("Brak folderu instances lub solutions.")
        return
    
    solutions = [f for f in os.listdir(solutions_dir) if f.endswith(".txt")]
    
    if not solutions:
        print("Brak plikow rozwiazan w folderze solutions.")
        return
        
    all_ok = True
    print(f"Znaleziono {len(solutions)} plikow z rozwiazaniami. Rozpoczynam sprawdzanie...\n")
    
    for sol_file in solutions:
        inst_name = sol_file.split("_")[0]
        inst_path = os.path.join(instances_dir, inst_name + ".csv")
        
        sol_path = os.path.join(solutions_dir, sol_file)
        
        print(f"Sprawdzanie: {sol_file}")
        if not os.path.exists(inst_path):
            print(f"Nie znaleziono instancji {inst_path} dla rozwiazania {sol_file}")
            all_ok = False
            continue
            
        if not verify_solution(inst_path, sol_path):
            all_ok = False
        print("")
        
    if all_ok:
        print("Wszystkie rozwiazania sa poprawne.")
    else:
        print("Znaleziono bledy w niektorych rozwiazaniach.")

if __name__ == '__main__':
    check_all()
