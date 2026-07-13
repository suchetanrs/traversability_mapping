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

#include <traversability_mapping/Parameters.hpp>

namespace traversability_mapping
{

std::unique_ptr<ParameterHandler> ParameterHandler::parameterHandlerPtr_ = nullptr;
std::mutex ParameterHandler::instanceMutex_;

ParameterHandler::ParameterHandler(std::string yaml_file_path)
{
    // Load YAML file and retrieve parameters
    std::string yaml_path;
    if (yaml_file_path == "")
    {
        std::cerr << "\033[1;33m[ParameterHandler] No YAML file path provided. Falling back to defaults.\033[0m" << std::endl;
        const std::string yaml_base_path = ament_index_cpp::get_package_share_directory("traversability_mapping");
        yaml_path = yaml_base_path + "/params/traversabilityParams.yaml";
    }
    else
    {
        yaml_path = yaml_file_path;
    }
    YAML::Node loaded_node = YAML::LoadFile(yaml_path);

    // The key string is the single source of
    // truth shared by the YAML, this map, every getValue, and the ROS param.

    // --- Global grid geometry ---
    parameter_map_["grid/resolution_local_map"] = loaded_node["grid/resolution_local_map"].as<double>();
    parameter_map_["grid/grid_center_x"] = loaded_node["grid/grid_center_x"].as<double>();
    parameter_map_["grid/grid_center_y"] = loaded_node["grid/grid_center_y"].as<double>();
    parameter_map_["grid/half_size_traversability"] = loaded_node["grid/half_size_traversability"].as<double>();
    parameter_map_["grid/extend_length_every_resize_by"] = loaded_node["grid/extend_length_every_resize_by"].as<double>();

    // --- Traversability (per-cell hazard metric) ---
    parameter_map_["traversability/robot_radius"] = loaded_node["traversability/robot_radius"].as<double>();
    parameter_map_["traversability/ground_clearance"] = loaded_node["traversability/ground_clearance"].as<double>();
    parameter_map_["traversability/max_slope"] = loaded_node["traversability/max_slope"].as<double>();
    parameter_map_["traversability/min_vicinity_points"] = loaded_node["traversability/min_vicinity_points"].as<int>();
    parameter_map_["traversability/min_occupied_fraction"] = loaded_node["traversability/min_occupied_fraction"].as<double>();

    // --- Mapping (keyframe retention + worker scheduling) ---
    parameter_map_["mapping/is_kf_optimization_enabled"] = loaded_node["mapping/is_kf_optimization_enabled"].as<bool>();
    parameter_map_["mapping/num_local_keyframes"] = loaded_node["mapping/num_local_keyframes"].as<int>();
    parameter_map_["mapping/global_adjustment_sleep"] = loaded_node["mapping/global_adjustment_sleep"].as<int>();

    // --- Ingestion (cloud filtering + buffering) ---
    parameter_map_["ingestion/robot_height"] = loaded_node["ingestion/robot_height"].as<double>();
    parameter_map_["ingestion/max_range_base_frame"] = loaded_node["ingestion/max_range_base_frame"].as<double>();
    parameter_map_["ingestion/min_range_base_frame"] = loaded_node["ingestion/min_range_base_frame"].as<double>();
    parameter_map_["ingestion/use_pointcloud_buffer"] = loaded_node["ingestion/use_pointcloud_buffer"].as<bool>();
    parameter_map_["ingestion/use_ros_buffer"] = loaded_node["ingestion/use_ros_buffer"].as<bool>();

    // --- Publishing (node) ---
    parameter_map_["node/publish_rate_hz"] = loaded_node["node/publish_rate_hz"].as<double>();
}

std::vector<std::string> ParameterHandler::keys() const
{
    std::lock_guard<std::mutex> lock(mapMutex_);
    std::vector<std::string> out;
    out.reserve(parameter_map_.size());
    for (const auto &kv : parameter_map_)
        out.push_back(kv.first);
    return out;
}

const std::type_info& ParameterHandler::typeOf(const std::string& parameterKey) const
{
    std::lock_guard<std::mutex> lock(mapMutex_);
    auto it = parameter_map_.find(parameterKey);
    if (it == parameter_map_.end())
        throw std::runtime_error("Parameter " + parameterKey + " is not found in the map");
    return it->second.type();
}
} // namespace traversability_mapping