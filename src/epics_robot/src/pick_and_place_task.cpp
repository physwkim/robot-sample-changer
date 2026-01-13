// Simple waypoint-based Pick&Place task using MoveIt Task Constructor (MTC)
// - Joint-space waypoints are provided via ROS parameters (paste from /joint_states)
// - Relative motions (lift/retreat/lower) are executed with MoveRelative (Cartesian)
//
// Intended for UR3e + Hand-E:
//   arm_group: ur_arm
//   hand_group: hand (SRDF group states "open"/"close" must exist)

#include <rclcpp/rclcpp.hpp>

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace mtc = moveit::task_constructor;
using mtc::Stage;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_and_place_task");

static std::map<std::string, double> joints_from_list(const std::vector<std::string>& names,
                                                      const std::vector<double>& values,
                                                      const std::string& label)
{
  if (names.empty())
    throw std::runtime_error("arm_joint_names is empty");
  if (names.size() != values.size())
    throw std::runtime_error(label + ": arm_joint_names size != values size");

  std::map<std::string, double> joints;
  for (size_t i = 0; i < names.size(); ++i)
    joints[names[i]] = values[i];
  return joints;
}

static std::unique_ptr<mtc::stages::MoveTo> make_joint_move(const std::string& name,
                                                            const mtc::solvers::PlannerInterfacePtr& planner,
                                                            const std::string& arm_group,
                                                            const std::map<std::string, double>& joints)
{
  auto stage = std::make_unique<mtc::stages::MoveTo>(name, planner);
  stage->setGroup(arm_group);
  stage->setGoal(joints);
  stage->restrictDirection(mtc::stages::MoveTo::FORWARD);
  return stage;
}

static std::unique_ptr<mtc::stages::MoveTo> make_hand_named(const std::string& name,
                                                            const mtc::solvers::PlannerInterfacePtr& planner,
                                                            const std::string& hand_group,
                                                            const std::string& named_target)
{
  auto stage = std::make_unique<mtc::stages::MoveTo>(name, planner);
  stage->setGroup(hand_group);
  stage->setGoal(named_target);  // SRDF group_state e.g. "open", "close"
  stage->restrictDirection(mtc::stages::MoveTo::FORWARD);
  return stage;
}

static std::unique_ptr<mtc::stages::MoveRelative> make_cartesian_relative(const std::string& name,
                                                                          const mtc::solvers::PlannerInterfacePtr& planner,
                                                                          const std::string& group,
                                                                          const std::string& ik_frame_link,
                                                                          const std::string& direction_frame,
                                                                          double distance,
                                                                          double dx,
                                                                          double dy,
                                                                          double dz)
{
  auto stage = std::make_unique<mtc::stages::MoveRelative>(name, planner);
  stage->setGroup(group);
  stage->setIKFrame(ik_frame_link);

  geometry_msgs::msg::Vector3Stamped v;
  v.header.frame_id = direction_frame;
  v.vector.x = dx;
  v.vector.y = dy;
  v.vector.z = dz;
  stage->setDirection(v);

  stage->setMinMaxDistance(distance, distance);
  stage->restrictDirection(mtc::stages::MoveRelative::FORWARD);
  return stage;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("pick_and_place_task", options);

  // Parameters (all under node namespace)
  const auto arm_group = node->declare_parameter<std::string>("arm_group", "ur_arm");
  const auto hand_group = node->declare_parameter<std::string>("hand_group", "hand");
  const auto eef = node->declare_parameter<std::string>("eef", "hand");
  const auto ik_frame = node->declare_parameter<std::string>("ik_frame", "flange");
  // NOTE: tool0 프레임이 TF에 없거나 prefix가 붙는 경우가 있어, 기본은 gripper tip으로 둡니다.
  const auto tcp_frame = node->declare_parameter<std::string>("tcp_frame", "robotiq_hande_end");
  const auto world_frame = node->declare_parameter<std::string>("world_frame", "base_link");

  const auto arm_joint_names =
      node->declare_parameter<std::vector<std::string>>("arm_joint_names",
        { "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint" });

  // Joint-space waypoints (paste from /joint_states)
  const auto home = node->declare_parameter<std::vector<double>>("home", std::vector<double>{});
  const auto sample_approach = node->declare_parameter<std::vector<double>>("sample_approach", std::vector<double>{});
  const auto sample_pregrasp = node->declare_parameter<std::vector<double>>("sample_pregrasp", std::vector<double>{});
  const auto sample_grasp = node->declare_parameter<std::vector<double>>("sample_grasp", std::vector<double>{});
  const auto load_approach = node->declare_parameter<std::vector<double>>("load_approach", std::vector<double>{});
  const auto load_place = node->declare_parameter<std::vector<double>>("load_place", std::vector<double>{});
  const auto return_approach = node->declare_parameter<std::vector<double>>("return_approach", std::vector<double>{});

  // Relative motions (meters)
  const auto lift_z = node->declare_parameter<double>("lift_z", 0.05);
  const auto retreat_x = node->declare_parameter<double>("retreat_x", 0.05);   // along -X in tcp frame
  const auto lower_z = node->declare_parameter<double>("lower_z", 0.05);
  const auto retreat2_x = node->declare_parameter<double>("retreat2_x", 0.05);

  // Named hand targets in SRDF
  const auto hand_open = node->declare_parameter<std::string>("hand_open", "open");
  const auto hand_close = node->declare_parameter<std::string>("hand_close", "close");

  try {
    auto home_j = joints_from_list(arm_joint_names, home, "home");
    auto sample_approach_j = joints_from_list(arm_joint_names, sample_approach, "sample_approach");
    auto sample_pregrasp_j = joints_from_list(arm_joint_names, sample_pregrasp, "sample_pregrasp");
    auto sample_grasp_j = joints_from_list(arm_joint_names, sample_grasp, "sample_grasp");
    auto load_approach_j = joints_from_list(arm_joint_names, load_approach, "load_approach");
    auto load_place_j = joints_from_list(arm_joint_names, load_place, "load_place");
    auto return_approach_j = joints_from_list(arm_joint_names, return_approach, "return_approach");

    mtc::Task task;
    task.stages()->setName("pick_and_place_waypoints");
    task.loadRobotModel(node);

    task.setProperty("group", arm_group);
    task.setProperty("eef", eef);
    task.setProperty("ik_frame", ik_frame);

    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node);
    auto hand_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

    auto cartesian = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian->setMaxVelocityScalingFactor(1.0);
    cartesian->setMaxAccelerationScalingFactor(1.0);
    cartesian->setStepSize(0.005);

    // Start
    task.add(std::make_unique<mtc::stages::CurrentState>("current"));

    // Ensure open
    task.add(make_hand_named("hand open (init)", hand_planner, hand_group, hand_open));

    // Home -> approach -> pregrasp -> grasp
    task.add(make_joint_move("move home", sampling_planner, arm_group, home_j));
    task.add(make_joint_move("move sample approach", sampling_planner, arm_group, sample_approach_j));
    task.add(make_joint_move("move sample pregrasp", sampling_planner, arm_group, sample_pregrasp_j));
    task.add(make_joint_move("move sample grasp", sampling_planner, arm_group, sample_grasp_j));

    // Close gripper
    task.add(make_hand_named("hand close", hand_planner, hand_group, hand_close));

    // Lift up in world frame
    task.add(make_cartesian_relative("lift", cartesian, arm_group, tcp_frame, world_frame, lift_z, 0.0, 0.0, 1.0));

    // Retreat backwards in TCP frame
    task.add(make_cartesian_relative("retreat", cartesian, arm_group, tcp_frame, tcp_frame, retreat_x, -1.0, 0.0, 0.0));

    // Go to load approach, then place
    task.add(make_joint_move("move load approach", sampling_planner, arm_group, load_approach_j));
    task.add(make_joint_move("move load place", sampling_planner, arm_group, load_place_j));

    // Lower in world frame
    task.add(make_cartesian_relative("lower", cartesian, arm_group, tcp_frame, world_frame, lower_z, 0.0, 0.0, -1.0));

    // Open gripper
    task.add(make_hand_named("hand open (release)", hand_planner, hand_group, hand_open));

    // Retreat back again
    task.add(make_cartesian_relative("retreat2", cartesian, arm_group, tcp_frame, tcp_frame, retreat2_x, -1.0, 0.0, 0.0));

    // Return
    task.add(make_joint_move("return to sample approach", sampling_planner, arm_group, return_approach_j));

    // Init/plan/execute
    task.init();
    if (!task.plan(5)) {
      RCLCPP_ERROR(LOGGER, "Task planning failed");
      rclcpp::shutdown();
      return 1;
    }
    task.introspection().publishSolution(*task.solutions().front());

    auto result = task.execute(*task.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_ERROR(LOGGER, "Task execution failed: %d", result.val);
      rclcpp::shutdown();
      return 2;
    }

    RCLCPP_INFO(LOGGER, "✅ Done");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(LOGGER, "Parameter/Task error: %s", e.what());
    rclcpp::shutdown();
    return 3;
  }

  rclcpp::shutdown();
  return 0;
}


