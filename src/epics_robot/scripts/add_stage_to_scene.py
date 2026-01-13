#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Add stage.stl to RViz Planning Scene
"""

import rclpy
from rclpy.node import Node
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped
import time


class StageSceneSetup(Node):
    def __init__(self):
        super().__init__('stage_scene_setup')
        
        # Planning scene interface
        self.psi = PlanningSceneInterface(synchronous=True)
        
        self.get_logger().info('Stage Scene Setup node initialized')
    
    def add_stage_mesh(self):
        """Add stage.stl as mesh to planning scene"""
        
        # Clear previous objects
        self.get_logger().info('Clearing previous scene objects...')
        self.psi.remove_world_object()
        time.sleep(0.5)
        
        # Stage mesh pose (relative to base_link frame)
        stage_pose = PoseStamped()
        stage_pose.header.frame_id = "base_link"
        # Position in meters
        stage_pose.pose.position.x = 0.0  # Forward(+) / Backward(-)
        stage_pose.pose.position.y = 0.0  # Left(+) / Right(-)
        stage_pose.pose.position.z = 0.0  # Up(+) / Down(-)
        # Orientation (quaternion - w=1.0 means no rotation)
        stage_pose.pose.orientation.w = 1.0
        stage_pose.pose.orientation.x = 0.0
        stage_pose.pose.orientation.y = 0.0
        stage_pose.pose.orientation.z = 0.0
        
        # Path to STL file
        stl_path = "/home/stevek/ws/stage.stl"
        
        self.get_logger().info(f'Adding stage mesh from: {stl_path}')
        
        # Add mesh to planning scene
        # Scale factor: 0.01 converts cm to meter
        self.psi.add_mesh(
            name="stage",
            pose=stage_pose,
            filename=stl_path,
            size=(0.01, 0.01, 0.01)  # Convert cm to meter
        )
        
        time.sleep(1.0)
        self.get_logger().info('✅ Stage mesh added to planning scene!')
        
        # Optionally add a box on top of the stage for pick & place
        # box_pose = PoseStamped()
        # box_pose.header.frame_id = "base_link"
        # box_pose.pose.position.x = 0.3
        # box_pose.pose.position.y = 0.0
        # box_pose.pose.position.z = 0.1  # Above the stage
        # box_pose.pose.orientation.w = 1.0
        # 
        # self.get_logger().info('Adding box object...')
        # self.psi.add_box(
        #     name="box",
        #     pose=box_pose,
        #     size=(0.05, 0.05, 0.05)
        # )
        # 
        # time.sleep(0.5)
        # self.get_logger().info('✅ Box object added!')
    
    def list_objects(self):
        """List all objects in planning scene"""
        objects = self.psi.get_known_object_names()
        self.get_logger().info(f'Objects in scene: {objects}')
        return objects


def main(args=None):
    rclpy.init(args=args)
    
    node = StageSceneSetup()
    
    # Give MoveIt some time to initialize
    time.sleep(2.0)
    
    # Add stage mesh
    node.add_stage_mesh()
    
    # List objects
    node.list_objects()
    
    print("\n" + "="*50)
    print("✅ Stage and objects added to planning scene!")
    print("="*50)
    print("\nYou can now:")
    print("1. View the stage in RViz (MotionPlanning → Scene Objects)")
    print("2. Run pick & place tasks")
    print("3. Manually adjust object positions in code\n")
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()

