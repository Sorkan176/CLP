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

//    int container_w = data[0][0];
//    int container_l = data[0][1];
//    int container_d = data[0][2];

//    auto result = exact_algorithm(data);
//    save_solution_csv(result.best_placements, "solution_10.csv");
    auto result = grasp(data);
    save_solution_csv(result.best_placements, "solution_75.csv");
}

int main()
{
    main_program("../data/tests/test_75.txt");
    return 0;
}


//#include <iostream>
//#include <filesystem>
//#include <fstream>
//#include "data/load_data.hpp"
//#include "structure/data_structure.hpp"
//#include "heuristic_algorithm/grasp.hpp"
//#include "exact_algorithm/branch_and_bound.hpp"
//#include "visualization/visualize_3d.hpp"
//
//namespace fs = std::filesystem;
//
//void main_program(const std::string& file_name)
//{
//    auto data = load_data(file_name);
//    auto result = exact_algorithm(data);;
//}
//
//int main()
//{
//    for (int i = 0; i <= 0; ++i)
//    {
//        std::string folder = "../data2/BR" + std::to_string(i);
//        std::string output_file = "solution_0_BR" + std::to_string(i) + ".txt";
//
//        std::ofstream out(output_file);
//
//        // сохраняем старый cout
//        std::streambuf* old_cout = std::cout.rdbuf();
//        std::cout.rdbuf(out.rdbuf());
//
//        for (const auto& entry : fs::directory_iterator(folder))
//        {
//            if (entry.is_regular_file() && entry.path().extension() == ".txt")
//            {
//                std::string file_name = entry.path().string();
//                std::cout << "Processing: " << file_name << std::endl;
//
//                main_program(file_name);
//            }
//        }
//
//        // возвращаем cout обратно
//        std::cout.rdbuf(old_cout);
//    }
//
//    return 0;
//}
