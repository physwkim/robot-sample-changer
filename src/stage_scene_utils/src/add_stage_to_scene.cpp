/*Copyright 2024 Brookhaven National Laboratory
BSD 3 Clause License. See LICENSE.txt for details.*/

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <filesystem>
#include <chrono>
#include <thread>
#include <cstdlib>

using namespace std::chrono_literals;
namespace fs = std::filesystem;

class StageSceneSetup : public rclcpp::Node
{
public:
  StageSceneSetup()
  : Node("stage_scene_setup")
  {
    // Get home directory from environment
    const char* home = std::getenv("HOME");
    std::string default_stl_path = home ? std::string(home) + "/ws/resources/stage.stl" : "/tmp/stage.stl";

    // Declare parameters
    this->declare_parameter<std::string>("stl_path", default_stl_path);
    this->declare_parameter<std::string>("object_name", "stage");
    this->declare_parameter<std::string>("frame_id", "world");
    this->declare_parameter<std::vector<double>>("position", {0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>("rotation", {0.0, 0.0, 0.0});  // [roll, pitch, yaw] in radians
    this->declare_parameter<std::vector<double>>("scale", {1.0, 1.0, 1.0});

    RCLCPP_INFO(this->get_logger(), "Stage Scene Setup node initialized");
  }

  bool add_stage_mesh()
  {
    // Get parameters
    std::string stl_path = this->get_parameter("stl_path").as_string();
    std::string object_name = this->get_parameter("object_name").as_string();
    std::string frame_id = this->get_parameter("frame_id").as_string();
    auto position = this->get_parameter("position").as_double_array();
    auto rotation = this->get_parameter("rotation").as_double_array();  // [roll, pitch, yaw]
    auto scale = this->get_parameter("scale").as_double_array();

    // Validate file exists
    if (!fs::exists(stl_path)) {
      RCLCPP_ERROR(this->get_logger(), "STL file not found: %s", stl_path.c_str());
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "STL file found: %s (%.2f MB)",
                stl_path.c_str(), fs::file_size(stl_path) / 1024.0 / 1024.0);

    // Clear previous objects (optional)
    auto known_objects = planning_scene_interface_.getKnownObjectNames();
    if (std::find(known_objects.begin(), known_objects.end(), object_name) != known_objects.end()) {
      RCLCPP_INFO(this->get_logger(), "Removing existing '%s' object", object_name.c_str());
      planning_scene_interface_.removeCollisionObjects({object_name});
      std::this_thread::sleep_for(500ms);
    }

    RCLCPP_INFO(this->get_logger(), "Loading mesh...");
    RCLCPP_INFO(this->get_logger(), "   Name: %s", object_name.c_str());
    RCLCPP_INFO(this->get_logger(), "   Frame: %s", frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "   Position: [%.3f, %.3f, %.3f]",
                position[0], position[1], position[2]);
    RCLCPP_INFO(this->get_logger(), "   Rotation (RPY): [%.3f, %.3f, %.3f] rad = [%.1f, %.1f, %.1f] deg",
                rotation[0], rotation[1], rotation[2],
                rotation[0] * 180.0 / M_PI, rotation[1] * 180.0 / M_PI, rotation[2] * 180.0 / M_PI);
    RCLCPP_INFO(this->get_logger(), "   Scale: [%.2f, %.2f, %.2f]",
                scale[0], scale[1], scale[2]);

    // Load mesh using geometric_shapes
    std::string stl_resource = "file://" + stl_path;
    Eigen::Vector3d scale_vec(scale[0], scale[1], scale[2]);

    shapes::Mesh* mesh = shapes::createMeshFromResource(stl_resource, scale_vec);
    if (!mesh) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load mesh from: %s", stl_resource.c_str());
      return false;
    }

    // Convert to ROS message
    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(mesh, mesh_msg);
    delete mesh;

    shape_msgs::msg::Mesh mesh_shape = boost::get<shape_msgs::msg::Mesh>(mesh_msg);

    // Create collision object
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.id = object_name;
    collision_object.header.frame_id = frame_id;
    collision_object.header.stamp = this->now();

    // Convert RPY to quaternion
    tf2::Quaternion quat;
    quat.setRPY(rotation[0], rotation[1], rotation[2]);

    geometry_msgs::msg::Pose mesh_pose;
    mesh_pose.position.x = position[0];
    mesh_pose.position.y = position[1];
    mesh_pose.position.z = position[2];
    mesh_pose.orientation = tf2::toMsg(quat);

    collision_object.meshes.push_back(mesh_shape);
    collision_object.mesh_poses.push_back(mesh_pose);
    collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;

    // Apply to planning scene
    RCLCPP_INFO(this->get_logger(), "Adding mesh to planning scene...");
    bool success = planning_scene_interface_.applyCollisionObject(collision_object);

    if (!success) {
      RCLCPP_ERROR(this->get_logger(), "Failed to add mesh to planning scene!");
      return false;
    }

    // Wait for planning scene to update
    std::this_thread::sleep_for(1s);

    // Update ACM to allow collision between stage and base links only
    RCLCPP_INFO(this->get_logger(), "Updating ACM to allow stage collision with base links only...");
    update_acm_for_gripper(object_name);
    std::this_thread::sleep_for(500ms);

    // Verify object was added
    auto updated_objects = planning_scene_interface_.getKnownObjectNames();
    if (std::find(updated_objects.begin(), updated_objects.end(), object_name) != updated_objects.end()) {
      RCLCPP_INFO(this->get_logger(), "Mesh '%s' successfully added!", object_name.c_str());
      return true;
    } else {
      RCLCPP_WARN(this->get_logger(), "Mesh added but not found in scene (may need more time)");
      return true;
    }
  }

  void list_objects()
  {
    auto objects = planning_scene_interface_.getKnownObjectNames();
    if (objects.empty()) {
      RCLCPP_INFO(this->get_logger(), "No objects in planning scene");
    } else {
      std::string names;
      for (const auto& obj : objects)
        names += obj + " ";
      RCLCPP_INFO(this->get_logger(), "Objects in scene: [%s]", names.c_str());
    }
  }

  void update_acm_for_gripper(const std::string& object_name)
  {
    // Create service clients
    auto get_scene_client = this->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    auto apply_scene_client = this->create_client<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene");

    // Wait for services
    if (!get_scene_client->wait_for_service(5s)) {
      RCLCPP_ERROR(this->get_logger(), "GetPlanningScene service not available");
      return;
    }
    if (!apply_scene_client->wait_for_service(5s)) {
      RCLCPP_ERROR(this->get_logger(), "ApplyPlanningScene service not available");
      return;
    }

    // Get current planning scene (with ACM and world objects)
    auto get_request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    get_request->components.components =
      moveit_msgs::msg::PlanningSceneComponents::SCENE_SETTINGS |
      moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX |
      moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_NAMES |
      moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;

    auto get_future = get_scene_client->async_send_request(get_request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), get_future, 5s) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get planning scene");
      return;
    }

    auto current_scene = get_future.get()->scene;
    auto& acm = current_scene.allowed_collision_matrix;

    // Only base links to allow collision with stage (arm/gripper should still check collision)
    std::vector<std::string> allowed_links = {
      "base_link",
      "base_link_inertia",
      "base"
    };

    // Find or add object_name to ACM
    auto it = std::find(acm.entry_names.begin(), acm.entry_names.end(), object_name);
    size_t object_idx;

    if (it == acm.entry_names.end()) {
      // Add new entry
      object_idx = acm.entry_names.size();
      acm.entry_names.push_back(object_name);

      // Resize all existing rows
      for (auto& row : acm.entry_values) {
        row.enabled.push_back(false);
      }

      // Add new row for object
      moveit_msgs::msg::AllowedCollisionEntry new_row;
      new_row.enabled.resize(acm.entry_names.size(), false);
      acm.entry_values.push_back(new_row);
    } else {
      object_idx = std::distance(acm.entry_names.begin(), it);
    }

    // Enable collisions between object and all robot links
    for (const auto& link : allowed_links) {
      auto link_it = std::find(acm.entry_names.begin(), acm.entry_names.end(), link);
      if (link_it != acm.entry_names.end()) {
        size_t link_idx = std::distance(acm.entry_names.begin(), link_it);
        acm.entry_values[object_idx].enabled[link_idx] = true;
        acm.entry_values[link_idx].enabled[object_idx] = true;
        RCLCPP_DEBUG(this->get_logger(), "Allowing collision: %s <-> %s", object_name.c_str(), link.c_str());
      }
    }

    // Apply updated scene
    auto apply_request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
    apply_request->scene = current_scene;
    apply_request->scene.is_diff = false;  // Send full scene

    auto apply_future = apply_scene_client->async_send_request(apply_request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), apply_future, 5s) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to apply planning scene");
      return;
    }

    if (apply_future.get()->success) {
      RCLCPP_INFO(this->get_logger(), "Successfully updated ACM for '%s'", object_name.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to update ACM");
    }
  }

private:
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<StageSceneSetup>();

  // Give MoveIt time to initialize
  RCLCPP_INFO(node->get_logger(), "Waiting for MoveIt to initialize...");
  std::this_thread::sleep_for(2s);

  // Add mesh
  if (!node->add_stage_mesh()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to add stage mesh");
    rclcpp::shutdown();
    return 1;
  }

  // List all objects
  node->list_objects();

  // Keep node alive to ensure planning scene updates propagate
  RCLCPP_INFO(node->get_logger(), "Keeping node alive for 3 seconds...");
  auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < 3s && rclcpp::ok()) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(100ms);
  }

  RCLCPP_INFO(node->get_logger(), "Done!");
  rclcpp::shutdown();
  return 0;
}
