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

#pragma once

// ROS <-> core parameter bridge.

#include <typeinfo>

#include <rclcpp/rclcpp.hpp>

#include "traversability_mapping/Parameters.hpp"

namespace tmap_ros
{

/// Call AFTER ParameterHandler::getInstance(...) has loaded the YAML.
inline rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
bridgeCoreParameters(rclcpp::Node *node)
{
    auto &ph = traversability_mapping::ParameterHandler::getInstance();

    // 1) Seed ROS params from the core (single source of truth -> ROS).
    for (const auto &key : ph.keys())
    {
        const std::type_info &t = ph.typeOf(key);
        if (node->has_parameter(key))
            continue;  // never clobber a param the node already owns
        if (t == typeid(double))
            node->declare_parameter<double>(key, ph.getValue<double>(key));
        else if (t == typeid(int))
            node->declare_parameter<int>(key, ph.getValue<int>(key));
        else if (t == typeid(bool))
            node->declare_parameter<bool>(key, ph.getValue<bool>(key));
        else if (t == typeid(std::string))
            node->declare_parameter<std::string>(key, ph.getValue<std::string>(key));
        else
            RCLCPP_WARN(node->get_logger(),
                        "[param_bridge] '%s' has an unbridged type; skipping.", key.c_str());
    }

    auto cb = [&ph](const std::vector<rclcpp::Parameter> &params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto &p : params)
        {
            const std::string &name = p.get_name();
            const std::type_info *t = nullptr;
            try { t = &ph.typeOf(name); }
            catch (const std::exception &) { continue; }  // not a core param

            using PT = rclcpp::ParameterType;
            const auto pt = p.get_type();
            if (*t == typeid(double) && pt == PT::PARAMETER_DOUBLE)
                ph.setValue<double>(name, p.as_double());
            else if (*t == typeid(int) && pt == PT::PARAMETER_INTEGER)
                ph.setValue<int>(name, static_cast<int>(p.as_int()));
            else if (*t == typeid(bool) && pt == PT::PARAMETER_BOOL)
                ph.setValue<bool>(name, p.as_bool());
            else if (*t == typeid(std::string) && pt == PT::PARAMETER_STRING)
                ph.setValue<std::string>(name, p.as_string());
            else
            {
                result.successful = false;
                result.reason = "Type mismatch for core parameter '" + name + "'";
            }
        }
        return result;
    };
    return node->add_on_set_parameters_callback(cb);
}

}  // namespace tmap_ros
