/* Unit tests for the ROS-free ParameterHandler (Parameters.hpp / Parameters.cpp).
 *
 * These exercise YAML loading, the typed get/set accessors, and the introspection
 * hooks (keys / typeOf) that the ROS parameter bridge relies on. They depend only
 * on yaml-cpp + boost (pulled in transitively by traversabilitySystem), so they
 * build and run without ROS.
 */
#include <gtest/gtest.h>
#include <traversability_mapping/Parameters.hpp>

#include <boost/any.hpp>

#include <fstream>
#include <set>
#include <string>
#include <vector>

using traversability_mapping::ParameterHandler;

namespace
{
    // A complete params file (every key the constructor reads) with KNOWN values,
    // so each accessor can be asserted exactly. Mirrors traversabilityParams.yaml.
    const char *kYaml = R"yaml(
grid/resolution_local_map: 0.05
grid/grid_center_x: 1.5
grid/grid_center_y: -2.5
ingestion/robot_height: 3.5
ingestion/max_range_base_frame: 300.0
ingestion/min_range_base_frame: 1.0
ingestion/use_pointcloud_buffer: true
ingestion/use_ros_buffer: false
grid/half_size_traversability: 7.5
grid/extend_length_every_resize_by: 30.0
traversability/robot_radius: 0.4
traversability/ground_clearance: 0.15
traversability/max_slope: 0.4
traversability/min_vicinity_points: 15
traversability/min_occupied_fraction: 0.5
mapping/is_kf_optimization_enabled: true
mapping/num_local_keyframes: 10
mapping/global_adjustment_sleep: 0
node/publish_rate_hz: 1.0
)yaml";

    // Write kYaml to a unique temp file and return its path.
    std::string writeTempYaml()
    {
        const std::string path = ::testing::TempDir() + "/tmap_test_params.yaml";
        std::ofstream out(path);
        out << kYaml;
        out.close();
        return path;
    }

    // The 19 keys the YAML / constructor define.
    const std::set<std::string> kExpectedKeys = {
        "grid/resolution_local_map", "grid/grid_center_x", "grid/grid_center_y",
        "ingestion/robot_height", "ingestion/max_range_base_frame", "ingestion/min_range_base_frame",
        "ingestion/use_pointcloud_buffer", "ingestion/use_ros_buffer",
        "grid/half_size_traversability", "grid/extend_length_every_resize_by",
        "traversability/robot_radius", "traversability/ground_clearance", "traversability/max_slope",
        "traversability/min_vicinity_points", "traversability/min_occupied_fraction",
        "mapping/is_kf_optimization_enabled", "mapping/num_local_keyframes",
        "mapping/global_adjustment_sleep", "node/publish_rate_hz"};
}  // namespace

// --- Loading + typed get ----------------------------------------------------

TEST(Parameters, LoadsTypedValuesFromYaml)
{
    ParameterHandler ph(writeTempYaml());
    EXPECT_DOUBLE_EQ(ph.getValue<double>("grid/resolution_local_map"), 0.05);
    EXPECT_DOUBLE_EQ(ph.getValue<double>("grid/grid_center_x"), 1.5);
    EXPECT_DOUBLE_EQ(ph.getValue<double>("grid/grid_center_y"), -2.5);
    EXPECT_DOUBLE_EQ(ph.getValue<double>("traversability/max_slope"), 0.4);
    EXPECT_DOUBLE_EQ(ph.getValue<double>("node/publish_rate_hz"), 1.0);
    EXPECT_EQ(ph.getValue<int>("traversability/min_vicinity_points"), 15);
    EXPECT_EQ(ph.getValue<int>("mapping/num_local_keyframes"), 10);
    EXPECT_EQ(ph.getValue<int>("mapping/global_adjustment_sleep"), 0);
    EXPECT_TRUE(ph.getValue<bool>("ingestion/use_pointcloud_buffer"));
    EXPECT_FALSE(ph.getValue<bool>("ingestion/use_ros_buffer"));
    EXPECT_TRUE(ph.getValue<bool>("mapping/is_kf_optimization_enabled"));
}

TEST(Parameters, MissingKeyThrows)
{
    ParameterHandler ph(writeTempYaml());
    EXPECT_THROW(ph.getValue<double>("grid/does_not_exist"), std::runtime_error);
}

TEST(Parameters, WrongTypeThrows)
{
    // grid_center_x is stored as double; asking for int must not silently succeed.
    ParameterHandler ph(writeTempYaml());
    EXPECT_THROW(ph.getValue<int>("grid/grid_center_x"), boost::bad_any_cast);
    EXPECT_THROW(ph.getValue<bool>("grid/grid_center_x"), boost::bad_any_cast);
}

// --- setValue ---------------------------------------------------------------

TEST(Parameters, SetValueOverwritesExisting)
{
    ParameterHandler ph(writeTempYaml());
    ph.setValue<double>("traversability/max_slope", 0.99);
    EXPECT_DOUBLE_EQ(ph.getValue<double>("traversability/max_slope"), 0.99);
}

TEST(Parameters, SetValueAddsNewKey)
{
    ParameterHandler ph(writeTempYaml());
    ASSERT_THROW(ph.getValue<int>("mapping/new_runtime_key"), std::runtime_error);
    ph.setValue<int>("mapping/new_runtime_key", 7);
    EXPECT_EQ(ph.getValue<int>("mapping/new_runtime_key"), 7);
    EXPECT_TRUE(ph.typeOf("mapping/new_runtime_key") == typeid(int));
}

// --- Introspection (keys / typeOf) used by the ROS bridge -------------------

TEST(Parameters, KeysReturnsExactLoadedSet)
{
    ParameterHandler ph(writeTempYaml());
    std::vector<std::string> got = ph.keys();
    std::set<std::string> gotSet(got.begin(), got.end());
    EXPECT_EQ(got.size(), kExpectedKeys.size());  // no duplicates
    EXPECT_EQ(gotSet, kExpectedKeys);
}

TEST(Parameters, TypeOfReportsStoredType)
{
    ParameterHandler ph(writeTempYaml());
    EXPECT_TRUE(ph.typeOf("grid/grid_center_x") == typeid(double));
    EXPECT_TRUE(ph.typeOf("traversability/min_vicinity_points") == typeid(int));
    EXPECT_TRUE(ph.typeOf("ingestion/use_pointcloud_buffer") == typeid(bool));
}

TEST(Parameters, TypeOfMissingKeyThrows)
{
    ParameterHandler ph(writeTempYaml());
    EXPECT_THROW(ph.typeOf("grid/does_not_exist"), std::runtime_error);
}

// --- Singleton accessor -----------------------------------------------------

TEST(Parameters, GetInstanceIsSingletonAndLoadsOnce)
{
    const std::string path = writeTempYaml();
    ParameterHandler &a = ParameterHandler::getInstance(path);
    ParameterHandler &b = ParameterHandler::getInstance();  // path ignored after first
    EXPECT_EQ(&a, &b);
    EXPECT_DOUBLE_EQ(a.getValue<double>("grid/grid_center_x"), 1.5);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
