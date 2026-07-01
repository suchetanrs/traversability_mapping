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

#ifndef TRAVERSABILITY_PARAMETERS_HPP_
#define TRAVERSABILITY_PARAMETERS_HPP_

#include <iostream>
#include <memory>
#include <unordered_map>
#include <string>
#include <vector>
#include <typeinfo>
#include <mutex>
#include <stdexcept>
#include <yaml-cpp/yaml.h>
#include <boost/any.hpp>

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace traversability_mapping
{
class ParameterHandler
{
  public:
    ParameterHandler(std::string yaml_file_path = "");

    static ParameterHandler& getInstance(std::string yaml_file_path = "")
    {
        std::lock_guard<std::mutex> lock(instanceMutex_);
        if(parameterHandlerPtr_ == nullptr)
            parameterHandlerPtr_.reset(new ParameterHandler(yaml_file_path));
        return *parameterHandlerPtr_;
    }

    template <typename T>
    T getValue(const std::string& parameterKey) const
    {
        std::lock_guard<std::mutex> lock(mapMutex_);
        auto it = parameter_map_.find(parameterKey);
        if (it != parameter_map_.end())
        {
            // std::cout << "\e[0;106m" << "Got request for: " << parameterKey << "\e[m" << std::endl;
            // std::cout << "\e[0;106m" << "Returning value " << boost::any_cast<T>(it->second) << " for parameter " << parameterKey
            //           << "\e[m" << std::endl;
            return boost::any_cast<T>(it->second);
        }
        // TODO : Handle this runtime error.
        throw std::runtime_error("Parameter " + parameterKey + " is not found in the map");
    }

    template <typename T>
    void setValue(const std::string& parameterKey, const T& value) {
        std::lock_guard<std::mutex> lock(mapMutex_);
        parameter_map_[parameterKey] = value;
    }

    /// Introspection hooks. The adapter
    /// dispatches on typeOf() and reuses getValue<T>/setValue<T>.
    std::vector<std::string> keys() const;
    const std::type_info& typeOf(const std::string& parameterKey) const;

  private:
    ParameterHandler(const ParameterHandler&) = delete;
    ParameterHandler& operator=(const ParameterHandler&) = delete;
    static std::unique_ptr<ParameterHandler> parameterHandlerPtr_;
    static std::mutex instanceMutex_;
    mutable std::mutex mapMutex_;
    std::unordered_map<std::string, boost::any> parameter_map_;
};
} // namespace traversability_mapping

#define parameterInstance (::traversability_mapping::ParameterHandler::getInstance())

#endif