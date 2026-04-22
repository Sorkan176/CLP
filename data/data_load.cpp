#include "load_data.hpp"

#include <fstream>
#include <sstream>

std::vector<std::vector<int>> load_data(const std::string& filename)
{
    std::vector<std::vector<int>> data;
    std::ifstream file(filename);

    if (!file.is_open())
        throw std::runtime_error("Cannot open file: " + filename);

    std::string line;

    while (std::getline(file, line))
    {
        if (line.empty())
            continue;

        std::vector<int> row;
        std::stringstream ss(line);
        std::string value;

        while (std::getline(ss, value, ','))
        {
            row.push_back(std::stoi(value));
        }

        data.push_back(row);
    }

    return data;
}