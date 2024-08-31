#include <traversability_mapping/Parameters.hpp>

std::unique_ptr<ParameterHandler> ParameterHandler::parameterHandlerPtr_ = nullptr;
std::mutex ParameterHandler::instanceMutex_;

ParameterHandler::ParameterHandler()
{
    // Load YAML file and retrieve parameters
    YAML::Node loaded_node = YAML::LoadFile("/usr/local/params/traversabilityParams.yaml");

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
    parameter_map_["is_kf_optimization_enabled"] = loaded_node["is_kf_optimization_enabled"].as<bool>();
    parameter_map_["use_pointcloud_buffer"] = loaded_node["use_pointcloud_buffer"].as<bool>();

    parameter_map_["T_SLAMFrameToLidarFrame/translation/x"] = loaded_node["T_SLAMFrameToLidarFrame"]["translation"]["x"].as<float>();
    parameter_map_["T_SLAMFrameToLidarFrame/translation/y"] = loaded_node["T_SLAMFrameToLidarFrame"]["translation"]["y"].as<float>();
    parameter_map_["T_SLAMFrameToLidarFrame/translation/z"] = loaded_node["T_SLAMFrameToLidarFrame"]["translation"]["z"].as<float>();
    parameter_map_["T_SLAMFrameToLidarFrame/quaternion/w"] = loaded_node["T_SLAMFrameToLidarFrame"]["quaternion"]["w"].as<float>();
    parameter_map_["T_SLAMFrameToLidarFrame/quaternion/x"] = loaded_node["T_SLAMFrameToLidarFrame"]["quaternion"]["x"].as<float>();
    parameter_map_["T_SLAMFrameToLidarFrame/quaternion/y"] = loaded_node["T_SLAMFrameToLidarFrame"]["quaternion"]["y"].as<float>();
    parameter_map_["T_SLAMFrameToLidarFrame/quaternion/z"] = loaded_node["T_SLAMFrameToLidarFrame"]["quaternion"]["z"].as<float>();
}