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
/**
 * @brief Thread-safe, YAML-backed key/value parameter store used as a process singleton.
 *
 * Values are held type-erased in a boost::any map, so the ROS-free core carries no ROS
 * parameter dependency; an adapter can mirror these to ROS params via the introspection
 * hooks (keys() / typeOf()) and getValue<T>() / setValue<T>().
 */
class ParameterHandler
{
  public:
    /// @brief Construct from a YAML file; an empty path loads the package default.
    /// @param yaml_file_path path to the YAML params file ("" = package default).
    ParameterHandler(std::string yaml_file_path = "");

    /// @brief Lazily-constructed process singleton.
    /// @param yaml_file_path source path; only the FIRST call's value takes effect.
    /// @return the singleton instance.
    static ParameterHandler& getInstance(std::string yaml_file_path = "")
    {
        std::lock_guard<std::mutex> lock(instanceMutex_);
        if(parameterHandlerPtr_ == nullptr)
            parameterHandlerPtr_.reset(new ParameterHandler(yaml_file_path));
        return *parameterHandlerPtr_;
    }

    /// @brief Typed read of a parameter.
    /// @tparam T stored value type (must match what was loaded / set).
    /// @param parameterKey key to read. @return the stored value.
    /// @throws std::runtime_error if the key is absent.
    /// @throws boost::bad_any_cast if @p T does not match the stored type.
    template <typename T>
    T getValue(const std::string& parameterKey) const
    {
        std::lock_guard<std::mutex> lock(mapMutex_);
        auto it = parameter_map_.find(parameterKey);
        if (it != parameter_map_.end())
        {
            return boost::any_cast<T>(it->second);
        }
        throw std::runtime_error("Parameter " + parameterKey + " is not found in the map");
    }

    /// @brief Insert or overwrite a parameter.
    /// @tparam T value type. @param parameterKey key. @param value value to store.
    template <typename T>
    void setValue(const std::string& parameterKey, const T& value) {
        std::lock_guard<std::mutex> lock(mapMutex_);
        parameter_map_[parameterKey] = value;
    }

    /// @brief Set an EXISTING parameter to a new value, preserving its stored type.
    /// @param parameterKey key to update (must already exist).
    /// @param value new value; its held type must match the stored type.
    /// @throws std::runtime_error if the key is absent or the type does not match.
    void setParameter(const std::string& parameterKey, const boost::any& value);

    /// @brief Introspection hook for the ROS bridge. @return every currently-stored key.
    std::vector<std::string> keys() const;
    /// @brief Stored type of a parameter; the bridge dispatches on this.
    /// @param parameterKey key to query. @return the stored type.
    /// @throws std::runtime_error if the key is absent.
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

/// @brief Shorthand for the ParameterHandler singleton instance.
#define parameterInstance (::traversability_mapping::ParameterHandler::getInstance())

#endif
