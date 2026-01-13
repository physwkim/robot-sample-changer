#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
UR3e + Hand-E Pick and Place Example using MoveIt Task Constructor
"""

import rclpy
from rclpy.node import Node
from moveit.task_constructor import core, stages
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
import math


class UR3eHandePickPlace(Node):
    def __init__(self):
        super().__init__('ur3e_hande_pickplace')
        
        # Robot parameters for UR3e + Hand-E
        self.arm_group = "ur_arm"
        self.eef_group = "hand"
        self.eef_frame = "robotiq_hande_end"  # TCP frame
        
        # Object parameters
        self.object_name = "box"
        self.object_pose = PoseStamped()
        self.object_pose.header.frame_id = "base_link"
        self.object_pose.pose.position.x = 0.3
        self.object_pose.pose.position.y = 0.0
        self.object_pose.pose.position.z = 0.1
        self.object_pose.pose.orientation.w = 1.0
        
        # Hand-E gripper joint values
        self.gripper_open = 0.025  # 25mm open
        self.gripper_close = 0.0   # fully closed
        
        # Planning scene interface
        self.psi = PlanningSceneInterface(synchronous=True)
        
        self.get_logger().info('UR3e Hand-E Pick and Place node initialized')
    
    def setup_planning_scene(self):
        """Add stage mesh and object to planning scene"""
        # Clear previous objects
        self.psi.remove_world_object()
        
        import time
        time.sleep(0.5)
        
        # Add stage mesh
        stage_pose = PoseStamped()
        stage_pose.header.frame_id = "base_link"
        stage_pose.pose.position.x = 0.0
        stage_pose.pose.position.y = 0.0
        stage_pose.pose.position.z = 0.0
        stage_pose.pose.orientation.w = 1.0
        
        self.get_logger().info('Adding stage mesh...')
        self.psi.add_mesh(
            name="stage",
            pose=stage_pose,
            filename="/home/stevek/ws/stage.stl",
            size=(0.01, 0.01, 0.01)  # Convert cm to meter
        )
        time.sleep(1.0)
        
        # Add box to grasp (on top of stage)
        self.psi.add_box(
            self.object_name, 
            self.object_pose, 
            size=(0.05, 0.05, 0.05)
        )
        
        self.get_logger().info(f'Added stage and {self.object_name} to planning scene')
    
    def create_task(self):
        """Create MTC task for pick and place"""
        task = core.Task()
        task.name = "UR3e Hand-E Pick and Place"
        
        # Set planning scene
        task.loadRobotModel(self)
        
        # Don't reset mock time when running with simulated time
        task.setProperty("trajectory_execution_info", 
                        {"controller_names": ["scaled_joint_trajectory_controller"]})
        
        # ============ Stage 1: Current State ============
        current_state = stages.CurrentState("current state")
        task.add(current_state)
        
        # ============ Stage 2: Open Gripper ============
        open_gripper = stages.MoveTo("open gripper", core.PipelinePlanner(self))
        open_gripper.group = self.eef_group
        open_gripper.setGoal("open")  # Named pose in SRDF
        task.add(open_gripper)
        
        # ============ Stage 3: Move to Pre-Grasp ============
        # This will plan from current state to above the object
        connect_to_pick = stages.Connect(
            "move to pick",
            [(self.arm_group, core.PipelinePlanner(self))]
        )
        connect_to_pick.setTimeout(5.0)
        connect_to_pick.properties.configureInitFrom(core.Stage.PropertyInitializerSource.PARENT)
        task.add(connect_to_pick)
        
        # ============ Pick Container ============
        # Create a serial container for the pick sequence
        pick = core.SerialContainer("pick object")
        task.add(pick)
        
        # Attach the object
        attach_object = stages.ModifyPlanningScene("attach object")
        attach_object.attachObject(self.object_name, self.eef_frame)
        pick.insert(attach_object)
        
        # Approach motion (move down to grasp)
        approach = stages.MoveRelative("approach object", core.CartesianPlanner(self))
        approach.group = self.arm_group
        approach.setIKFrame(self.eef_frame)
        approach.properties.configureInitFrom(core.Stage.PropertyInitializerSource.PARENT, ["group"])
        approach.setMinMaxDistance(0.05, 0.15)
        
        # Approach vector (down in z)
        approach_direction = Vector3Stamped()
        approach_direction.header.frame_id = "base_link"
        approach_direction.vector.z = -1.0
        approach.setDirection(approach_direction)
        pick.insert(approach)
        
        # Close gripper
        close_gripper = stages.MoveTo("close gripper", core.PipelinePlanner(self))
        close_gripper.group = self.eef_group
        close_gripper.setGoal("close")  # Named pose in SRDF
        pick.insert(close_gripper)
        
        # Lift motion
        lift = stages.MoveRelative("lift object", core.CartesianPlanner(self))
        lift.group = self.arm_group
        lift.setIKFrame(self.eef_frame)
        lift.properties.configureInitFrom(core.Stage.PropertyInitializerSource.PARENT, ["group"])
        lift.setMinMaxDistance(0.05, 0.15)
        
        # Lift vector (up in z)
        lift_direction = Vector3Stamped()
        lift_direction.header.frame_id = "base_link"
        lift_direction.vector.z = 1.0
        lift.setDirection(lift_direction)
        pick.insert(lift)
        
        # ============ Stage 4: Move to Place Location ============
        connect_to_place = stages.Connect(
            "move to place",
            [(self.arm_group, core.PipelinePlanner(self))]
        )
        connect_to_place.setTimeout(5.0)
        connect_to_place.properties.configureInitFrom(core.Stage.PropertyInitializerSource.PARENT)
        task.add(connect_to_place)
        
        # ============ Place Container ============
        place = core.SerialContainer("place object")
        task.add(place)
        
        # Approach motion (move down to place)
        place_approach = stages.MoveRelative("lower object", core.CartesianPlanner(self))
        place_approach.group = self.arm_group
        place_approach.setIKFrame(self.eef_frame)
        place_approach.properties.configureInitFrom(core.Stage.PropertyInitializerSource.PARENT, ["group"])
        place_approach.setMinMaxDistance(0.05, 0.15)
        
        place_approach_direction = Vector3Stamped()
        place_approach_direction.header.frame_id = "base_link"
        place_approach_direction.vector.z = -1.0
        place_approach.setDirection(place_approach_direction)
        place.insert(place_approach)
        
        # Open gripper to release
        release_gripper = stages.MoveTo("release gripper", core.PipelinePlanner(self))
        release_gripper.group = self.eef_group
        release_gripper.setGoal("open")
        place.insert(release_gripper)
        
        # Detach object
        detach_object = stages.ModifyPlanningScene("detach object")
        detach_object.detachObject(self.object_name, self.eef_frame)
        place.insert(detach_object)
        
        # Retreat motion (move up after placing)
        retreat = stages.MoveRelative("retreat after place", core.CartesianPlanner(self))
        retreat.group = self.arm_group
        retreat.setIKFrame(self.eef_frame)
        retreat.properties.configureInitFrom(core.Stage.PropertyInitializerSource.PARENT, ["group"])
        retreat.setMinMaxDistance(0.05, 0.15)
        
        retreat_direction = Vector3Stamped()
        retreat_direction.header.frame_id = "base_link"
        retreat_direction.vector.z = 1.0
        retreat.setDirection(retreat_direction)
        place.insert(retreat)
        
        # ============ Stage 5: Return Home ============
        return_home = stages.MoveTo("return home", core.PipelinePlanner(self))
        return_home.group = self.arm_group
        return_home.setGoal("moveit_home")  # Named pose from SRDF
        task.add(return_home)
        
        return task
    
    def run(self):
        """Execute pick and place task"""
        self.get_logger().info('Setting up planning scene...')
        self.setup_planning_scene()
        
        self.get_logger().info('Creating MTC task...')
        task = self.create_task()
        
        self.get_logger().info('Planning task...')
        try:
            task.init()
            if not task.plan():
                self.get_logger().error('Planning failed')
                return False
            
            self.get_logger().info('Planning succeeded! Executing...')
            result = task.execute()
            
            if result.val == result.SUCCESS:
                self.get_logger().info('Task executed successfully!')
                return True
            else:
                self.get_logger().error('Task execution failed')
                return False
                
        except Exception as e:
            self.get_logger().error(f'Error during planning/execution: {e}')
            return False


def main(args=None):
    rclpy.init(args=args)
    
    node = UR3eHandePickPlace()
    
    # Give MoveIt some time to initialize
    import time
    time.sleep(2.0)
    
    # Run the pick and place task
    success = node.run()
    
    if success:
        print("\n✅ Pick and place completed successfully!")
    else:
        print("\n❌ Pick and place failed")
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()

