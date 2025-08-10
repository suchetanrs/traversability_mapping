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

std::unique_ptr<ParameterHandler> ParameterHandler::parameterHandlerPtr_ = nullptr;
std::mutex ParameterHandler::instanceMutex_;

ParameterHandler::ParameterHandler(std::string yaml_file_path)
{
    // Load YAML file and retrieve parameters
    std::string yaml_path;
    if (yaml_file_path == "")
    {
        std::cerr << "\033[1;31m!!!Error: No YAML file path provided. Using defaults!!!\033[0m" << std::endl;
        const std::string yaml_base_path = ament_index_cpp::get_package_share_directory("traversability_mapping");
        YAML::Node yaml_node;
        yaml_path = yaml_base_path + "/params/traversabilityParams.yaml";
    }
    else
    {
        yaml_path = yaml_file_path;
    }
    YAML::Node loaded_node = YAML::LoadFile(yaml_path);

    // Traversability Params
    parameter_map_["SLAM_System"] = loaded_node["SLAM_System"].as<std::string>();
    
    parameter_map_["half_size_local_map"] = loaded_node["half_size_local_map"].as<double>();
    parameter_map_["grid_center_x"] = loaded_node["grid_center_x"].as<double>();
    parameter_map_["grid_center_y"] = loaded_node["grid_center_y"].as<double>();
    parameter_map_["resolution_local_map"] = loaded_node["resolution_local_map"].as<double>();
    parameter_map_["half_size_traversability"] = loaded_node["half_size_traversability"].as<double>();
    parameter_map_["security_distance"] = loaded_node["security_distance"].as<double>();
    parameter_map_["ground_clearance"] = loaded_node["ground_clearance"].as<double>();
    parameter_map_["max_slope"] = loaded_node["max_slope"].as<double>();
    parameter_map_["robot_height"] = loaded_node["robot_height"].as<double>();
    parameter_map_["translation_change_threshold"] = loaded_node["translation_change_threshold"].as<double>();
    parameter_map_["rotation_change_threshold"] = loaded_node["rotation_change_threshold"].as<double>();
    parameter_map_["num_local_keyframes"] = loaded_node["num_local_keyframes"].as<double>();
    parameter_map_["is_kf_optimization_enabled"] = loaded_node["is_kf_optimization_enabled"].as<bool>();
    parameter_map_["use_pointcloud_buffer"] = loaded_node["use_pointcloud_buffer"].as<bool>();
    parameter_map_["use_ros_buffer"] = loaded_node["use_ros_buffer"].as<bool>();
    parameter_map_["use_averaging"] = loaded_node["use_averaging"].as<bool>();
    parameter_map_["use_probabilistic_update"] = loaded_node["use_probabilistic_update"].as<bool>();
    parameter_map_["average_persistence"] = loaded_node["average_persistence"].as<double>();
    parameter_map_["use_virtual_boundary"] = loaded_node["use_virtual_boundary"].as<bool>();
    parameter_map_["extend_length_every_resize_by"] = loaded_node["extend_length_every_resize_by"].as<double>();
    parameter_map_["use_pca_to_compute_normals"] = loaded_node["use_pca_to_compute_normals"].as<bool>();
    parameter_map_["use_least_squares_fit_to_compute_normals"] = loaded_node["use_least_squares_fit_to_compute_normals"].as<bool>();
    parameter_map_["save_normals_to_csv"] = loaded_node["save_normals_to_csv"].as<bool>();
    
    parameter_map_["global_adjustment_sleep"] = loaded_node["global_adjustment_sleep"].as<int>();

    if(getValue<bool>("use_averaging") && getValue<bool>("use_probabilistic_update"))
    {
        throw std::runtime_error("Cannot use both probabilistic update and average. Make either one true, not both.");
    }
}