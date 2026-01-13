#!/usr/bin/env python3
"""
Setup Allowed Collision Matrix (ACM) for stage object
Disables collision between stage and base links
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import AllowedCollisionMatrix, AllowedCollisionEntry
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene
import time


class StageACMSetup(Node):
    def __init__(self):
        super().__init__('stage_acm_setup')
        
        # Service clients
        self.get_scene_client = self.create_client(
            GetPlanningScene, 
            '/get_planning_scene'
        )
        self.apply_scene_client = self.create_client(
            ApplyPlanningScene, 
            '/apply_planning_scene'
        )
        
        self.get_logger().info('Stage ACM Setup node initialized')
    
    def setup_stage_acm(self):
        """Add stage to ACM and disable collision with base links"""
        
        # Wait for services
        self.get_logger().info('Waiting for planning scene services...')
        self.get_scene_client.wait_for_service(timeout_sec=5.0)
        self.apply_scene_client.wait_for_service(timeout_sec=5.0)
        
        # Get current planning scene
        self.get_logger().info('Getting current planning scene...')
        get_req = GetPlanningScene.Request()
        get_req.components.components = get_req.components.ALLOWED_COLLISION_MATRIX
        
        future = self.get_scene_client.call_async(get_req)
        rclpy.spin_until_future_complete(self, future)
        
        if not future.result():
            self.get_logger().error('Failed to get planning scene')
            return False
        
        scene = future.result().scene
        acm = scene.allowed_collision_matrix
        
        # Links to allow collision with stage
        links_to_allow = [
            'base_link',
            'base_link_inertia',
            'base',
            'world'
        ]
        
        stage_name = 'stage'
        
        # Add stage to ACM if not already present
        if stage_name not in acm.entry_names:
            self.get_logger().info(f'Adding {stage_name} to ACM...')
            acm.entry_names.append(stage_name)
            
            # Add new row/column for stage
            for entry in acm.entry_values:
                entry.enabled.append(False)
            
            # Add new entry for stage
            new_entry = AllowedCollisionEntry()
            new_entry.enabled = [False] * len(acm.entry_names)
            acm.entry_values.append(new_entry)
        
        # Find stage index
        try:
            stage_idx = acm.entry_names.index(stage_name)
        except ValueError:
            self.get_logger().error(f'{stage_name} not found in ACM')
            return False
        
        # Enable collision allowance between stage and specified links
        for link in links_to_allow:
            if link in acm.entry_names:
                link_idx = acm.entry_names.index(link)
                acm.entry_values[stage_idx].enabled[link_idx] = True
                acm.entry_values[link_idx].enabled[stage_idx] = True
                self.get_logger().info(f'Disabled collision: {stage_name} <-> {link}')
        
        # Apply updated ACM
        self.get_logger().info('Applying updated ACM...')
        from moveit_msgs.msg import PlanningScene
        apply_req = ApplyPlanningScene.Request()
        apply_req.scene.allowed_collision_matrix = acm
        apply_req.scene.is_diff = True
        
        future = self.apply_scene_client.call_async(apply_req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() and future.result().success:
            self.get_logger().info('✅ ACM updated successfully!')
            return True
        else:
            self.get_logger().error('❌ Failed to apply ACM')
            return False


def main(args=None):
    rclpy.init(args=args)
    
    node = StageACMSetup()
    
    # Wait a bit for planning scene to be ready
    time.sleep(2.0)
    
    success = node.setup_stage_acm()
    
    if success:
        print("\n✅ Stage ACM configured successfully!")
        print("Collision between stage and base links is now disabled.\n")
    else:
        print("\n❌ Failed to configure stage ACM\n")
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()

