/* Traversability Mapping - A global and local traversability mapping algorithm.
 * Copyright (C) 2024 Suchetan Saravanan and Damien Vivet
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library.  If not, see
 * <https://www.gnu.org/licenses/>.
 */
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

#include <iomanip>
#include <sstream>
#include <ctime>
#include <stdexcept>
#include <yaml-cpp/yaml.h>
#include <boost/any.hpp>

#include "ament_index_cpp/get_package_share_directory.hpp"

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
            // std::cout << "\e[0;106m" << "Returning value " << boost::any_cast<T>(parameter_map_[parameterKey]) << " for parameter " << parameterKey
            //           << "\e[m" << std::endl;
            return boost::any_cast<T>(parameter_map_[parameterKey]);
        }
        else
        {
            // TODO : Handle this runtime error.
            throw std::runtime_error("Parameter " + parameterKey + " is not found in the map");
        }
    }

    template <typename T>
    void setValue(const std::string& parameterKey, const T& value) {
        parameter_map_[parameterKey] = value;
    }

  private:
    ParameterHandler(const ParameterHandler&) = delete;
    ParameterHandler& operator=(const ParameterHandler&) = delete;
    static std::unique_ptr<ParameterHandler> parameterHandlerPtr_;
    static std::mutex instanceMutex_;
    std::unordered_map<std::string, boost::any> parameter_map_;
};

inline ParameterHandler& parameterInstance = ParameterHandler::getInstance();

#endif