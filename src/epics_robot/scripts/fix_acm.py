#!/usr/bin/env python3
"""
Fix ACM (Allowed Collision Matrix) after MoveIt starts.
Run this AFTER MoveIt launches and AFTER adding stage to scene.

Usage:
    ros2 run mtc_tutorial fix_acm.py
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene
from moveit_msgs.msg import AllowedCollisionEntry


class ACMFixer(Node):
    def __init__(self):
        super().__init__('acm_fixer')
        self.get_cli = self.create_client(GetPlanningScene, '/get_planning_scene')
        self.apply_cli = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.get_logger().info('ACM Fixer initialized')

    def fix_acm(self):
        """Add all robot links to ACM and disable collisions"""
        
        self.get_logger().info('⏳ Waiting for planning scene services...')
        if not self.get_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('❌ /get_planning_scene not available')
            return False
        if not self.apply_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('❌ /apply_planning_scene not available')
            return False
        
        # Get current ACM
        self.get_logger().info('📥 Getting current ACM...')
        req = GetPlanningScene.Request()
        req.components.components = 128  # ALLOWED_COLLISION_MATRIX
        
        future = self.get_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if not future.result():
            self.get_logger().error('❌ Failed to get planning scene')
            return False
        
        scene = future.result().scene
        acm = scene.allowed_collision_matrix
        
        self.get_logger().info(f'📋 Current ACM has {len(acm.entry_names)} entries')
        
        # All robot links to ensure are in ACM
        all_links = [
            'base_link', 'base_link_inertia', 'base', 'world',
            'shoulder_link', 'upper_arm_link', 'forearm_link',
            'wrist_1_link', 'wrist_2_link', 'wrist_3_link',
            'flange', 'tool0',
            'robotiq_hande_coupler', 'robotiq_hande_link',
            'robotiq_hande_left_finger', 'robotiq_hande_right_finger',
            'robotiq_hande_end'
        ]
        
        # Add missing links to ACM
        added = []
        for link in all_links:
            if link not in acm.entry_names:
                acm.entry_names.append(link)
                # Expand existing rows
                for entry in acm.entry_values:
                    entry.enabled.append(False)
                # Add new row
                new_entry = AllowedCollisionEntry()
                new_entry.enabled = [False] * len(acm.entry_names)
                acm.entry_values.append(new_entry)
                added.append(link)
        
        if added:
            self.get_logger().info(f'➕ Added {len(added)} links to ACM')
        
        # If stage exists, allow collision with ALL links (especially base and gripper)
        if 'stage' in acm.entry_names:
            stage_idx = acm.entry_names.index('stage')
            stage_allowed_count = 0
            for i in range(len(acm.entry_names)):
                if i != stage_idx:
                    acm.entry_values[stage_idx].enabled[i] = True
                    acm.entry_values[i].enabled[stage_idx] = True
                    stage_allowed_count += 1
            self.get_logger().info(f'🎯 Disabled stage collision with {stage_allowed_count} links (base + arm + gripper)')
        
        # Disable arm self-collisions
        arm_pairs = [
            ('base_link_inertia', 'shoulder_link'),
            ('upper_arm_link', 'forearm_link'),
            ('shoulder_link', 'upper_arm_link'),
            ('wrist_1_link', 'wrist_2_link'),
            ('wrist_2_link', 'wrist_3_link'),
            ('forearm_link', 'wrist_1_link'),
        ]
        
        self.get_logger().info('🔧 Disabling arm self-collisions...')
        for link1, link2 in arm_pairs:
            if link1 in acm.entry_names and link2 in acm.entry_names:
                idx1 = acm.entry_names.index(link1)
                idx2 = acm.entry_names.index(link2)
                acm.entry_values[idx1].enabled[idx2] = True
                acm.entry_values[idx2].enabled[idx1] = True
        
        # Disable gripper (hand) internal collisions
        gripper_links = [
            'robotiq_hande_coupler',
            'robotiq_hande_link',
            'robotiq_hande_left_finger',
            'robotiq_hande_right_finger',
            'robotiq_hande_end',
        ]
        
        self.get_logger().info('🤚 Disabling gripper internal collisions...')
        # Allow collision between all gripper link pairs
        for i, link1 in enumerate(gripper_links):
            for link2 in gripper_links[i+1:]:
                if link1 in acm.entry_names and link2 in acm.entry_names:
                    idx1 = acm.entry_names.index(link1)
                    idx2 = acm.entry_names.index(link2)
                    acm.entry_values[idx1].enabled[idx2] = True
                    acm.entry_values[idx2].enabled[idx1] = True
                    self.get_logger().info(f'  ✅ {link1} ↔ {link2}')
        
        # Disable gripper to wrist/flange collisions
        gripper_to_wrist = [
            ('robotiq_hande_coupler', 'wrist_3_link'),
            ('robotiq_hande_coupler', 'flange'),
            ('robotiq_hande_link', 'wrist_3_link'),
            ('robotiq_hande_link', 'flange'),
            ('robotiq_hande_left_finger', 'wrist_3_link'),
            ('robotiq_hande_right_finger', 'wrist_3_link'),
        ]
        
        self.get_logger().info('🔗 Disabling gripper-to-wrist collisions...')
        for link1, link2 in gripper_to_wrist:
            if link1 in acm.entry_names and link2 in acm.entry_names:
                idx1 = acm.entry_names.index(link1)
                idx2 = acm.entry_names.index(link2)
                acm.entry_values[idx1].enabled[idx2] = True
                acm.entry_values[idx2].enabled[idx1] = True
        
        # Apply updated ACM
        self.get_logger().info('📤 Applying updated ACM...')
        apply_req = ApplyPlanningScene.Request()
        apply_req.scene.allowed_collision_matrix = acm
        apply_req.scene.is_diff = True
        
        apply_future = self.apply_cli.call_async(apply_req)
        rclpy.spin_until_future_complete(self, apply_future, timeout_sec=5.0)
        
        if apply_future.result() and apply_future.result().success:
            self.get_logger().info('')
            self.get_logger().info('='*70)
            self.get_logger().info('✅✅✅ ACM FIXED SUCCESSFULLY! ✅✅✅')
            self.get_logger().info('='*70)
            self.get_logger().info('')
            self.get_logger().info('로봇이 이제 정상 색상으로 표시됩니다!')
            return True
        else:
            self.get_logger().error('❌ Failed to apply ACM')
            return False


def main(args=None):
    rclpy.init(args=args)
    
    node = ACMFixer()
    
    # Give MoveIt time to initialize
    import time
    time.sleep(2.0)
    
    success = node.fix_acm()
    
    rclpy.shutdown()
    return 0 if success else 1


if __name__ == '__main__':
    import sys
    sys.exit(main())

