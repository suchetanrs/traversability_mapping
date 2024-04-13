#include <iostream>
#include <unordered_map>
#include <vector>
#include <algorithm>

// Function to move an entire set to another set
void moveSet(std::unordered_map<int, std::unordered_map<int, int>>& sets, int sourceSet, int destinationSet) {
    // Check if the source set exists
    auto sourceIt = sets.find(sourceSet);
    if (sourceIt != sets.end()) {
        // Move the entire vector from source set to destination set
        sets[destinationSet] = std::move(sourceIt->second);
        
        // Erase the source set
        sets.erase(sourceSet);
        
        std::cout << "Set " << sourceSet << " moved to set " << destinationSet << std::endl;
    } else {
        std::cout << "Source set " << sourceSet << " not found." << std::endl;
    }
}

// Function to move an element from one set to another
void moveElement(std::unordered_map<int, std::unordered_map<int, int>> &sets, const int &sourceSet, const int &destinationSet, int element, int elementKey)
{
    // Check if the source set exists and the element is present in it
    auto sourceIt = sets.find(sourceSet);
    if (sourceIt != sets.end())
    {
        auto &sourceMap = sourceIt->second;
        auto elementIt = sourceMap.find(elementKey);
        if (elementIt != sourceMap.end())
        {
            // Remove the element from the source set
            sourceMap.erase(elementIt);

            // Insert the element into the destination set
            sets[destinationSet][elementKey] = element;

            std::cout << "Element " << element << " moved from " << sourceSet << " to " << destinationSet << std::endl;
        }
        else
        {
            std::cout << "Element " << element << " not found in " << sourceSet << std::endl;
        }
    }
    else
    {
        std::cout << "Source set " << sourceSet << " not found." << std::endl;
    }
}

// Function to update an element in sets
void updateElement(std::unordered_map<int, std::unordered_map<int, int>> &sets, int key, int element, int elementKey)
{
    bool found = false;
    for (auto &entry : sets)
    {
        if (entry.first != key)
        {
            auto &currentMap = entry.second;
            auto elementIt = currentMap.find(elementKey);
            if (elementIt != currentMap.end())
            {
                moveElement(sets, entry.first, key, element, elementKey);
                std::cout << "Element " << element << " moved to set " << key << std::endl;
                found = true;
                break;
            }
        }
        else
        {
            auto &currentMap = entry.second;
            auto elementIt = currentMap.find(elementKey);
            if (elementIt != currentMap.end())
            {
                std::cout << "Element " << element << " already exists in set " << key << std::endl;
                found = true;
                break;
            }
        }
    }
    if (!found)
    {
        sets[key][elementKey] = element;
        std::cout << "Element " << element << " added to set " << key << std::endl;
    }
}

int main()
{
    // Define your sets
    // Local map, kf id, value
    std::unordered_map<int, std::unordered_map<int, int>> sets;

    // Populate the sets initially
    updateElement(sets, 1, 1, 101);
    updateElement(sets, 1, 2, 102);
    updateElement(sets, 1, 3, 103);
    updateElement(sets, 2, 4, 104);
    updateElement(sets, 2, 5, 105);
    updateElement(sets, 2, 6, 106);

    // Display sets after moving an element
    for (const auto &entry : sets)
    {
        std::cout << "Values of " << entry.first << " set:";
        for (auto value : entry.second)
        {
            std::cout << " " << value.second;
        }
        std::cout << std::endl;
    }

    std::cout << "=============================" << std::endl;
    updateElement(sets, 3, 69, 1069);
    updateElement(sets, 1, 4, 104);
    updateElement(sets, 1, 5, 105);
    updateElement(sets, 1, 6, 106);
    // moveSet(sets, 1, 4);
    // Display sets after moving an element
    for (const auto &entry : sets)
    {
        std::cout << "Values of " << entry.first << " set:";
        for (auto value : entry.second)
        {
            std::cout << " " << value.second;
        }
        std::cout << std::endl;
    }
    return 0;
}