#include <rclcpp/rclcpp.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/allowed_collision_matrix.hpp>
#include <moveit_msgs/msg/allowed_collision_entry.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <geometric_shapes/shape_operations.h>   // shapes::createMeshFromResource, constructMsgFromShape
#include <geometric_shapes/mesh_operations.h>

#include <chrono>
#include <thread>
#include <string>
#include <vector>

using namespace std::chrono_literals;

class StageSceneSetup : public rclcpp::Node
{
public:
  StageSceneSetup()
  : Node("stage_scene_setup")
  {
    RCLCPP_INFO(get_logger(), "Stage Scene Setup node initialized");
  }

  bool add_stage_mesh()
  {
    // 1) Remove previous objects (optional)
    RCLCPP_INFO(get_logger(), "Clearing previous scene objects...");
    auto names = planning_scene_interface_.getKnownObjectNames();
    if (!names.empty())
      planning_scene_interface_.removeCollisionObjects(names);

    std::this_thread::sleep_for(500ms);

    // 2) Build CollisionObject with Mesh
    const std::string object_id = "stage";
    const std::string frame_id  = "base_link";

    // IMPORTANT:
    // createMeshFromResource expects a resource URI.
    // For a plain absolute path, use "file://".
    const std::string stl_resource = "file:///home/stevek/ws/stage.stl";

    // Scale (convert cm to meter: 0.01)
    const Eigen::Vector3d scale(0.01, 0.01, 0.01);

    shapes::Mesh* mesh = shapes::createMeshFromResource(stl_resource, scale);
    if (!mesh)
    {
      RCLCPP_ERROR(get_logger(), "Failed to load mesh from: %s", stl_resource.c_str());
      return false;
    }

    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(mesh, mesh_msg);
    delete mesh;

    shape_msgs::msg::Mesh mesh_shape = boost::get<shape_msgs::msg::Mesh>(mesh_msg);

    geometry_msgs::msg::Pose mesh_pose;
    // Stage position in meters (relative to base_link)
    // NOTE: z=0.0에 두면 로봇 베이스와 겹쳐서 "전체가 collision"처럼 보일 수 있습니다.
    mesh_pose.position.x = -0.15;  // Forward(+) / Backward(-)
    mesh_pose.position.y = 0.4;    // Left(+) / Right(-)
    mesh_pose.position.z = 0.8;    // Up(+). 예: 테이블 상판 높이
    
    // Stage orientation (quaternion)
    // IMPORTANT: quaternion은 degrees를 직접 넣는게 아니라 (x,y,z,w) 단위 quaternion이어야 합니다.
    // 회전이 필요 없으면 w=1.0, 나머지 0.0이 맞습니다.
    mesh_pose.orientation.x = 0.0;
    mesh_pose.orientation.y = 0.0;
    mesh_pose.orientation.z = 0.0;
    mesh_pose.orientation.w = 1.0;

    moveit_msgs::msg::CollisionObject co;
    co.id = object_id;
    co.header.frame_id = frame_id;
    co.meshes.push_back(mesh_shape);
    co.mesh_poses.push_back(mesh_pose);
    co.operation = moveit_msgs::msg::CollisionObject::ADD;

    // 3) Apply to planning scene
    RCLCPP_INFO(get_logger(), "Adding stage mesh to planning scene...");
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(co);
    
    bool success = planning_scene_interface_.applyCollisionObjects(collision_objects);
    
    if (success)
    {
      RCLCPP_INFO(get_logger(), "✅ Stage mesh added successfully!");
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "❌ Failed to add stage mesh");
      return false;
    }

    std::this_thread::sleep_for(1s);
    
    return true;
  }

  void list_objects()
  {
    auto object_names = planning_scene_interface_.getKnownObjectNames();
    RCLCPP_INFO(get_logger(), "Objects in scene:");
    for (const auto& name : object_names)
    {
      RCLCPP_INFO(get_logger(), "  - %s", name.c_str());
    }
  }

  bool update_acm_for_stage()
  {
    RCLCPP_INFO(get_logger(), "Updating ACM to disable stage-base collisions...");
    
    // Create service clients
    auto get_scene_client = create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    auto apply_scene_client = create_client<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene");
    
    if (!get_scene_client->wait_for_service(std::chrono::seconds(3)))
    {
      RCLCPP_ERROR(get_logger(), "Service /get_planning_scene not available");
      return false;
    }
    if (!apply_scene_client->wait_for_service(std::chrono::seconds(3)))
    {
      RCLCPP_ERROR(get_logger(), "Service /apply_planning_scene not available");
      return false;
    }
    
    // Get current ACM
    auto get_request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    get_request->components.components = moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
    
    auto get_future = get_scene_client->async_send_request(get_request);
    if (rclcpp::spin_until_future_complete(shared_from_this(), get_future, std::chrono::seconds(5)) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Failed to get planning scene");
      return false;
    }
    
    auto scene = get_future.get()->scene;
    auto& acm = scene.allowed_collision_matrix;
    
    // Links to disable collision with stage
    std::vector<std::string> base_links = {"base_link", "base_link_inertia", "base", "world"};
    std::string stage_name = "stage";
    
    // Find or add stage to ACM
    auto stage_it = std::find(acm.entry_names.begin(), acm.entry_names.end(), stage_name);
    size_t stage_idx;
    
    if (stage_it == acm.entry_names.end())
    {
      // Add stage to ACM
      RCLCPP_INFO(get_logger(), "Adding stage to ACM...");
      acm.entry_names.push_back(stage_name);
      stage_idx = acm.entry_names.size() - 1;
      
      // Expand existing rows
      for (auto& entry : acm.entry_values)
      {
        entry.enabled.push_back(false);
      }
      
      // Add new row for stage
      moveit_msgs::msg::AllowedCollisionEntry new_entry;
      new_entry.enabled.resize(acm.entry_names.size(), false);
      acm.entry_values.push_back(new_entry);
    }
    else
    {
      stage_idx = std::distance(acm.entry_names.begin(), stage_it);
    }
    
    // Enable collision allowance between stage and base links
    for (const auto& link : base_links)
    {
      auto link_it = std::find(acm.entry_names.begin(), acm.entry_names.end(), link);
      if (link_it != acm.entry_names.end())
      {
        size_t link_idx = std::distance(acm.entry_names.begin(), link_it);
        acm.entry_values[stage_idx].enabled[link_idx] = true;
        acm.entry_values[link_idx].enabled[stage_idx] = true;
        RCLCPP_INFO(get_logger(), "Disabled collision: %s <-> %s", stage_name.c_str(), link.c_str());
      }
    }
    
    // Apply updated ACM
    auto apply_request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
    apply_request->scene.allowed_collision_matrix = acm;
    apply_request->scene.is_diff = true;
    
    auto apply_future = apply_scene_client->async_send_request(apply_request);
    if (rclcpp::spin_until_future_complete(shared_from_this(), apply_future, std::chrono::seconds(5)) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Failed to apply ACM");
      return false;
    }
    
    if (apply_future.get()->success)
    {
      RCLCPP_INFO(get_logger(), "✅ ACM updated successfully!");
      return true;
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "❌ Failed to update ACM");
      return false;
    }
  }

private:
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<StageSceneSetup>();
  
  // Give MoveIt some time to initialize
  RCLCPP_INFO(node->get_logger(), "Waiting for MoveIt to initialize...");
  std::this_thread::sleep_for(2s);
  
  // Add stage mesh
  bool success = node->add_stage_mesh();
  
  if (success)
  {
    // Update ACM to disable stage-base collisions
    bool acm_success = node->update_acm_for_stage();
    
    if (!acm_success)
    {
      RCLCPP_WARN(node->get_logger(), "Stage added but ACM update failed - collision may occur");
    }
    
    // List objects
    node->list_objects();
    
    RCLCPP_INFO(node->get_logger(), "\n"
                "==================================================\n"
                "✅ Stage mesh added to planning scene!\n"
                "==================================================\n"
                "\n"
                "You can now:\n"
                "1. View the stage in RViz (MotionPlanning → Scene Objects)\n"
                "2. Run pick & place tasks\n"
                "3. Adjust object positions in code\n");
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to add stage mesh!");
  }
  
  rclcpp::shutdown();
  return success ? 0 : 1;
}

