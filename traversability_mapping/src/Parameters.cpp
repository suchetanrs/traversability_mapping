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

    // --- Global grid / fixed lattice ---
    parameter_map_["resolution_local_map"] = loaded_node["resolution_local_map"].as<double>();
    parameter_map_["grid_center_x"] = loaded_node["grid_center_x"].as<double>();
    parameter_map_["grid_center_y"] = loaded_node["grid_center_y"].as<double>();
    parameter_map_["half_size_traversability"] = loaded_node["half_size_traversability"].as<double>();
    parameter_map_["extend_length_every_resize_by"] = loaded_node["extend_length_every_resize_by"].as<double>();

    // --- Traversability ---
    parameter_map_["security_distance"] = loaded_node["security_distance"].as<double>();
    parameter_map_["ground_clearance"] = loaded_node["ground_clearance"].as<double>();
    parameter_map_["max_slope"] = loaded_node["max_slope"].as<double>();
    parameter_map_["robot_height"] = loaded_node["robot_height"].as<double>();
    parameter_map_["max_range_base_frame"] = loaded_node["max_range_base_frame"].as<double>();
    parameter_map_["min_range_base_frame"] = loaded_node["min_range_base_frame"].as<double>();

    // --- Plane-fit gates ---
    parameter_map_["min_vicinity_points"] = loaded_node["min_vicinity_points"].as<int>();
    parameter_map_["min_occupied_fraction"] = loaded_node["min_occupied_fraction"].as<double>();

    // --- Publishing ---
    parameter_map_["publish_rate_hz"] = loaded_node["publish_rate_hz"].as<double>();

    // --- Mapping (cloud retention; used once true re-binning lands) ---
    parameter_map_["is_kf_optimization_enabled"] = loaded_node["is_kf_optimization_enabled"].as<bool>();
}

} // namespace traversability_mapping