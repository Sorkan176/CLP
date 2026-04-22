#include <iostream>

#include "data/load_data.hpp"
#include "structure/data_structure.hpp"
#include "exact_algorithm/branch_and_bound.hpp"
#include "heuristic_algorithm/grasp.hpp"
#include "visualization/visualize_3d.hpp"


void main_program(const std::string& file_name)
{
    auto data = load_data(file_name);

//    std::cout << "Container's coordinates: ";
//    for (int v : data[0])
//        std::cout << v << " ";
//    std::cout << std::endl;

    int container_w = data[0][0];
    int container_l = data[0][1];
    int container_d = data[0][2];

//    auto result = exact_algorithm(data);
//    save_solution_csv(result.best_placements, "exact_solution.csv");
    auto result = grasp(data);
    save_solution_csv(result.best_placements, "solution_75.csv");
}

int main()
{
    main_program("../data/tests/test_75.txt");
    return 0;
}
