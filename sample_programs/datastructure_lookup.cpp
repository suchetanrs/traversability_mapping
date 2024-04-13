#include <iostream>
#include <vector>
#include <map>
#include <unordered_map>
#include <chrono>
#include <random>
#include <algorithm>

int main() {
    // Generate random elements
    std::vector<int> elements;

    // Add elements to vector, map, and unordered_map
    std::vector<int> vec = elements;
    std::map<int, int> map;
    std::unordered_map<int, int> unordered_map;

    for (int i = 0; i < 8923865448; ++i) {
        elements.push_back(i);
        map.insert({ elements[i], elements[i] });
        unordered_map.insert({ elements[i], elements[i] });
    }

    // Lookup time comparison
    int target = 8923865;
    auto start = std::chrono::high_resolution_clock::now();
    auto it = std::find(vec.begin(), vec.end(), target);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> vector_time = end - start;

    start = std::chrono::high_resolution_clock::now();
    auto map_it = map.find(target);
    end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> map_time = end - start;

    start = std::chrono::high_resolution_clock::now();
    auto unordered_map_it = unordered_map.find(target);
    end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> unordered_map_time = end - start;

    // Output results
    std::cout << "Vector lookup time: " << vector_time.count() << " mseconds" << std::endl;
    std::cout << "Map lookup time: " << map_time.count() << " mseconds" << std::endl;
    std::cout << "Unordered Map lookup time: " << unordered_map_time.count() << " mseconds" << std::endl;

    return 0;
}
