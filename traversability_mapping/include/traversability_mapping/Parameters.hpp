// keyFrame.hpp
#ifndef PARAMETER_HPP_
#define PARAMETER_HPP_

#include <iostream>
#include <unordered_map>
#include <chrono>
#include <string>
#include <fstream>
#include <thread>
#include <mutex>
#include <random>
#include <ctime>
#include <any>

#include <iomanip>
#include <sstream>
#include <ctime>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

class ParameterHandler
{
  public:
    ParameterHandler();

    static ParameterHandler& getInstance()
    {
        std::lock_guard<std::mutex> lock(instanceMutex_);
        if(parameterHandlerPtr_ == nullptr)
            parameterHandlerPtr_.reset(new ParameterHandler());
        return *parameterHandlerPtr_;
    }

    template <typename T>
    T getValue(std::string parameterKey)
    {
        if (parameter_map_.find(parameterKey) != parameter_map_.end())
        {
            // std::cout << "\e[0;106m" << "Got request for: " << parameterKey << "\e[m" << std::endl;
            // std::cout << "\e[0;106m" << "Returning value " << std::any_cast<T>(parameter_map_[parameterKey]) << " for parameter " << parameterKey
            //           << "\e[m" << std::endl;
            return std::any_cast<T>(parameter_map_[parameterKey]);
        }
        else
        {
            // TODO : Handle this runtime error.
            throw std::runtime_error("Parameter " + parameterKey + " is not found in the map");
        }
    }

  private:
    ParameterHandler(const ParameterHandler&) = delete;
    ParameterHandler& operator=(const ParameterHandler&) = delete;
    static std::unique_ptr<ParameterHandler> parameterHandlerPtr_;
    static std::mutex instanceMutex_;
    std::unordered_map<std::string, std::any> parameter_map_;
};

inline ParameterHandler& parameterInstance = ParameterHandler::getInstance();

#endif