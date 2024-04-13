#include <iostream>
#include <deque>

int main() {
    std::deque<int> deq;

    // Get the number of elements in the deque
    size_t numElements = deq.size();

    // Output the number of elements
    std::cout << "Number of elements in the deque: " << numElements << std::endl;
    deq.push_back(3);
    deq.push_back(4);
    deq.push_back(2);
    std::cout << "Number of elements in the deque: " << deq[deq.size() - 1] << std::endl;
    deq.pop_back();
    numElements = deq.size();
    std::cout << "Number of elements in the deque: " << numElements << std::endl;
    return 0;
}