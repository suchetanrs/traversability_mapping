#include <iostream>
#include <thread>
#include <vector>

class MyClass {
public:
    void longRunningFunction(int k) {
        // Simulate a long-running task
        for (long unsigned int i = 0; i < 50000000; ++i) {
            std::cout << "Long running task iteration " << k << ", " << i << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // Simulate work
        }
    }
};

int main() {
    MyClass obj;
    std::vector<std::thread> threads;

    for (int j = 0; j < 3000; j++) {
        // Launching longRunningFunction in a new thread
        // std::thread t(&MyClass::longRunningFunction, &obj, j);
        threads.emplace_back(&MyClass::longRunningFunction, &obj, j);
        // t.detach();
    }

    // Do other tasks in the main thread
    for (int i = 0; i < 300000; ++i) {
        std::cout << "Main thread iteration " << i << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(200)); // Simulate work
    }

    for (std::thread& t : threads) {
        t.join();
    }
    // Wait for the thread to finish execution
    // t.join();

    return 0;
}
