// Run a single MTC MoveRelative stage from terminal.
// This is a C++ alternative when MoveItPy is not available / desired.
//
// Example:
//   ros2 run mtc_tutorial move_relative_mtc --ros-args \
//     -p group:=ur_arm -p ik_frame:=tool0 -p direction_frame:=tool0 \
//     -p distance:=0.05 -p dx:=-1.0 -p dy:=0.0 -p dz:=0.0

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <chrono>

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_relative.h>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

namespace mtc = moveit::task_constructor;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_relative_mtc");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  auto node = std::make_shared<rclcpp::Node>("move_relative_mtc", options);

  // Declare motion-related params first (so we can use group name for IK injection if needed)
  const auto group = node->declare_parameter<std::string>("group", "ur_arm");
  const auto ik_frame = node->declare_parameter<std::string>("ik_frame", "robotiq_hande_end");
  const auto direction_frame = node->declare_parameter<std::string>("direction_frame", "robotiq_hande_end");

  // IMPORTANT:
  // URDF(robot_description)와 SRDF(robot_description_semantic)는 반드시 같은 로봇(이름/조인트 포함)이어야 합니다.
  // /robot_description 토픽만 읽으면(로봇팔만) + move_group의 SRDF(그리퍼 포함)처럼 섞여서
  // "Joint ... is not known to the URDF" 에러가 납니다.
  //
  // 따라서 move_group 노드에서 두 파라미터를 "한 쌍"으로 가져와 이 노드에 세팅합니다.
  RCLCPP_INFO(LOGGER, "Waiting for move_group parameter service...");
  auto param_client_node = rclcpp::Node::make_shared("move_relative_mtc_param_client");
  auto params_client = std::make_shared<rclcpp::SyncParametersClient>(param_client_node, "move_group");
  while (!params_client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(LOGGER, "Interrupted while waiting for move_group param service");
      rclcpp::shutdown();
      return 1;
    }
    RCLCPP_INFO(LOGGER, "move_group param service not available, waiting again...");
  }

  auto params = params_client->get_parameters({ "robot_description", "robot_description_semantic", "robot_description_kinematics" });
  if (params.size() != 3) {
    RCLCPP_ERROR(LOGGER, "Failed to fetch robot_description(_semantic/kinematics) from move_group");
    rclcpp::shutdown();
    return 1;
  }
  node->set_parameter(params[0]);
  node->set_parameter(params[1]);
  if (params[2].get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET) {
    node->set_parameter(params[2]);
    RCLCPP_INFO(LOGGER, "Loaded robot_description + robot_description_semantic + robot_description_kinematics from move_group");
  } else {
    RCLCPP_WARN(LOGGER, "move_group has no robot_description_kinematics. Injecting default KDL IK for group '%s'", group.c_str());
    // Minimal IK config for ur_arm (matches ur3e_hande_moveit_config/config/kinematics.yaml)
    // Note: MoveIt reads hierarchical params as dot-separated names.
    node->set_parameter(rclcpp::Parameter("robot_description_kinematics." + group + ".kinematics_solver",
                                          "kdl_kinematics_plugin/KDLKinematicsPlugin"));
    node->set_parameter(rclcpp::Parameter("robot_description_kinematics." + group + ".kinematics_solver_search_resolution", 0.005));
    node->set_parameter(rclcpp::Parameter("robot_description_kinematics." + group + ".kinematics_solver_timeout", 0.005));
    node->set_parameter(rclcpp::Parameter("robot_description_kinematics." + group + ".kinematics_solver_attempts", 3));
    RCLCPP_INFO(LOGGER, "Injected robot_description_kinematics for '%s'", group.c_str());
  }

  // Direction is a unit vector (dx,dy,dz) expressed in direction_frame.
  // Distance is in meters.
  const auto distance = node->declare_parameter<double>("distance", 0.05);
  const auto dx = node->declare_parameter<double>("dx", -1.0);
  const auto dy = node->declare_parameter<double>("dy", 0.0);
  const auto dz = node->declare_parameter<double>("dz", 0.0);

  // Cartesian solver params
  const auto step_size = node->declare_parameter<double>("step_size", 0.005);
  const auto vel_scale = node->declare_parameter<double>("vel_scale", 1.0);
  const auto acc_scale = node->declare_parameter<double>("acc_scale", 1.0);

  try {
    mtc::Task task;
    task.stages()->setName("move_relative_once");
    task.loadRobotModel(node);

    // current state
    task.add(std::make_unique<mtc::stages::CurrentState>("current"));

    auto cartesian = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian->setStepSize(step_size);
    cartesian->setMaxVelocityScalingFactor(vel_scale);
    cartesian->setMaxAccelerationScalingFactor(acc_scale);

    auto stage = std::make_unique<mtc::stages::MoveRelative>("move_relative", cartesian);
    stage->setGroup(group);
    stage->setIKFrame(ik_frame);

    geometry_msgs::msg::Vector3Stamped v;
    v.header.frame_id = direction_frame;
    v.vector.x = dx;
    v.vector.y = dy;
    v.vector.z = dz;
    stage->setDirection(v);
    stage->setMinMaxDistance(distance, distance);
    stage->restrictDirection(mtc::stages::MoveRelative::FORWARD);

    task.add(std::move(stage));

    task.init();
    if (!task.plan(1)) {
      RCLCPP_ERROR(LOGGER, "Planning failed");
      rclcpp::shutdown();
      return 1;
    }

    RCLCPP_INFO(LOGGER, "✅ Planning successful!");
    task.introspection().publishSolution(*task.solutions().front());

    RCLCPP_INFO(LOGGER, "Executing task...");
    auto result = task.execute(*task.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_ERROR(LOGGER, "Task execution failed: %d", result.val);
      rclcpp::shutdown();
      return 1;
    }

    RCLCPP_INFO(LOGGER, "✅ Task execution successful!");
    RCLCPP_INFO(LOGGER, "Done");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(LOGGER, "Error: %s", e.what());
    rclcpp::shutdown();
    return 3;
  }

  rclcpp::shutdown();
  return 0;
}


