#include <iostream>
#include <vector>
#include <chrono>
#include <random>

int main() {
    // Create a 2D vector and resize it to 100 rows and 100 columns
    std::vector<std::vector<double>> matrix(250, std::vector<double>(250));

    // Fill the matrix with some values (for demonstration)
    // std::random_device rd;
    // std::mt19937 gen(rd());
    // std::uniform_int_distribution<> dis(1, 1000);

    // for (int i = 0; i < 100; ++i) {
    //     for (int j = 0; j < 100; ++j) {
    //         matrix.at(i).at(j) = dis(gen);
    //     }
    // }

    // Perform a lookup at a random index and measure the time to print it
    // std::uniform_int_distribution<> row_dis(0, 999999999);
    // std::uniform_int_distribution<> col_dis(0, 999999999);
    // int row = row_dis(gen);
    // int col = col_dis(gen);
    long int row = 97;
    long int col = 82;

    auto start = std::chrono::high_resolution_clock::now();
    std::cout << "Value at index (" << row << ", " << col << "): " << matrix.at(row).at(col) << std::endl;
    auto end = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "Time taken to print the value: " << duration.count() << " microseconds" << std::endl;

    return 0;
}
