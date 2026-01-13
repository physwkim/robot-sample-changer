#!/usr/bin/env python3
"""
Automatic Holder Exchange Script

This script monitors EPICS PVs and automatically cycles through holders 1-4
when each sequence completes.

Required: pyepics library (pip install pyepics)

Usage:
    python3 auto_holder_exchange.py [--holders 1,2,3,4] [--loop]
    
Options:
    --holders   Comma-separated list of holder numbers (default: 1,2,3,4)
    --loop      Continuously loop through holders (default: run once through all)
    --delay     Delay in seconds between trigger and wait (default: 0.5)
"""

import argparse
import signal
import sys
import time
from datetime import datetime

try:
    import epics
except ImportError:
    print("Error: pyepics library not found.")
    print("Please install it with: pip install pyepics")
    sys.exit(1)


class HolderExchangeController:
    """Controller for automatic holder exchange sequences."""
    
    # EPICS PV names
    PV_TRIGGER = "Robot:Trigger"
    PV_WAIT = "Robot:Wait"
    PV_HOLDER = "Robot:Holder"
    PV_CURRENT_STEP = "Robot:CurrentStep"
    PV_GRIPPER = "Robot:Gripper"
    PV_STOP = "Robot:Stop"
    
    def __init__(self, holders: list[int], loop: bool = False, delay: float = 0.5):
        """
        Initialize the controller.
        
        Args:
            holders: List of holder numbers to cycle through
            loop: If True, continuously loop through holders
            delay: Delay in seconds between trigger and wait commands
        """
        self.holders = holders
        self.loop = loop
        self.delay = delay
        self.running = True
        self.current_holder_index = 0
        
        # Connect to PVs
        print(f"[{self._timestamp()}] Connecting to EPICS PVs...")
        self.pv_trigger = epics.PV(self.PV_TRIGGER)
        self.pv_wait = epics.PV(self.PV_WAIT)
        self.pv_holder = epics.PV(self.PV_HOLDER)
        self.pv_current_step = epics.PV(self.PV_CURRENT_STEP)
        self.pv_gripper = epics.PV(self.PV_GRIPPER)
        self.pv_stop = epics.PV(self.PV_STOP)
        
        # Wait for connections
        time.sleep(0.5)
        
        # Verify connections
        if not self._verify_connections():
            print(f"[{self._timestamp()}] Error: Failed to connect to some PVs")
            sys.exit(1)
        
        print(f"[{self._timestamp()}] All PVs connected successfully")
        self._print_status()
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
    
    def _timestamp(self) -> str:
        """Return current timestamp string."""
        return datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    
    def _verify_connections(self) -> bool:
        """Verify all PV connections are established."""
        pvs = [
            (self.pv_trigger, self.PV_TRIGGER),
            (self.pv_wait, self.PV_WAIT),
            (self.pv_holder, self.PV_HOLDER),
            (self.pv_current_step, self.PV_CURRENT_STEP),
            (self.pv_gripper, self.PV_GRIPPER),
        ]
        
        all_connected = True
        for pv, name in pvs:
            if pv.connected:
                print(f"  ✓ {name}: connected")
            else:
                print(f"  ✗ {name}: NOT connected")
                all_connected = False
        
        return all_connected
    
    def _print_status(self):
        """Print current status of all PVs."""
        print(f"\n[{self._timestamp()}] Current PV Status:")
        print(f"  Holder:      {self.pv_holder.get()}")
        print(f"  Trigger:     {self.pv_trigger.get()}")
        print(f"  Wait:        {self.pv_wait.get()}")
        print(f"  CurrentStep: {self.pv_current_step.get()}")
        print(f"  Gripper:     {self._gripper_state()}")
        print(f"  Stop:        {self.pv_stop.get()}")
        print()
    
    def _gripper_state(self) -> str:
        """Return human-readable gripper state."""
        val = self.pv_gripper.get()
        if val == 0:
            return "Closed"
        elif val == 1:
            return "Open"
        else:
            return f"Unknown ({val})"
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully."""
        print(f"\n[{self._timestamp()}] Shutdown signal received. Stopping...")
        self.running = False
    
    def wait_for_sequence_complete(self, timeout: float = 300.0) -> bool:
        """
        Wait for the current sequence to complete.
        
        The sequence is complete when CurrentStep goes above 0 (started) 
        and then returns to 0 (completed).
        
        Args:
            timeout: Maximum time to wait in seconds
            
        Returns:
            True if sequence completed, False if timeout or interrupted
        """
        start_time = time.time()
        last_step = -1
        last_gripper = -1
        sequence_started = False  # Track if sequence has actually started
        
        print(f"[{self._timestamp()}] Waiting for sequence to start...")
        
        while self.running:
            current_step = self.pv_current_step.get()
            gripper = self.pv_gripper.get()
            
            # Print step changes
            if current_step != last_step:
                print(f"[{self._timestamp()}] CurrentStep: {current_step}")
                last_step = current_step
            
            # Print gripper changes
            if gripper != last_gripper:
                state = "Open" if gripper == 1 else "Closed"
                print(f"[{self._timestamp()}] Gripper: {state}")
                last_gripper = gripper
            
            # Check if sequence has started (CurrentStep > 0)
            if current_step > 0 and not sequence_started:
                sequence_started = True
                print(f"[{self._timestamp()}] Sequence started!")
            
            # Check if sequence completed (CurrentStep back to 0 AFTER it started)
            if sequence_started and current_step == 0:
                return True
            
            # Check timeout
            if time.time() - start_time > timeout:
                print(f"[{self._timestamp()}] Timeout waiting for sequence completion")
                return False
            
            time.sleep(0.1)
        
        return False
    
    def start_sequence(self, holder_num: int) -> bool:
        """
        Start a sequence for the specified holder.
        
        Args:
            holder_num: Holder number to use
            
        Returns:
            True if sequence started successfully
        """
        print(f"\n{'='*60}")
        print(f"[{self._timestamp()}] Starting sequence for Holder {holder_num}")
        print(f"{'='*60}")
        
        # Set holder number
        print(f"[{self._timestamp()}] Setting Holder to {holder_num}")
        self.pv_holder.put(holder_num)
        time.sleep(0.1)
        
        # Trigger the sequence
        print(f"[{self._timestamp()}] Triggering sequence...")
        self.pv_trigger.put(1)
        time.sleep(self.delay)
        
        # Set wait to continue
        print(f"[{self._timestamp()}] Setting Wait to continue...")
        self.pv_wait.put(1)
        
        return True
    
    def run(self):
        """Run the automatic holder exchange sequence."""
        print(f"\n[{self._timestamp()}] Starting Automatic Holder Exchange")
        print(f"  Holders: {self.holders}")
        print(f"  Loop mode: {'Yes' if self.loop else 'No'}")
        print(f"  Delay: {self.delay}s")
        print()
        
        iteration = 0
        
        while self.running:
            iteration += 1
            print(f"\n{'#'*60}")
            print(f"# Iteration {iteration}")
            print(f"{'#'*60}")
            
            for i, holder_num in enumerate(self.holders):
                if not self.running:
                    break
                
                self.current_holder_index = i
                
                # Start the sequence
                self.start_sequence(holder_num)
                
                # Wait for completion
                if not self.wait_for_sequence_complete():
                    if not self.running:
                        print(f"[{self._timestamp()}] Interrupted during holder {holder_num}")
                    break
                
                print(f"[{self._timestamp()}] Holder {holder_num} sequence completed!")
                
                # Small delay between holders
                if self.running and i < len(self.holders) - 1:
                    time.sleep(1.0)
            
            if not self.loop:
                break
            
            if self.running:
                print(f"\n[{self._timestamp()}] Cycle complete. Starting next cycle in 2 seconds...")
                time.sleep(2.0)
        
        print(f"\n[{self._timestamp()}] Automatic Holder Exchange finished")
        self._print_status()


def parse_holders(holders_str: str) -> list[int]:
    """Parse comma-separated holder numbers."""
    try:
        holders = [int(h.strip()) for h in holders_str.split(',')]
        for h in holders:
            if h < 1 or h > 10:
                raise ValueError(f"Holder number {h} out of range (1-10)")
        return holders
    except ValueError as e:
        print(f"Error parsing holders: {e}")
        sys.exit(1)


def main():
    parser = argparse.ArgumentParser(
        description="Automatic Holder Exchange Controller",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # Run through holders 1-4 once
    python3 auto_holder_exchange.py
    
    # Run through holders 1,2,3 continuously
    python3 auto_holder_exchange.py --holders 1,2,3 --loop
    
    # Run holders 5-8 with custom delay
    python3 auto_holder_exchange.py --holders 5,6,7,8 --delay 1.0
        """
    )
    
    parser.add_argument(
        '--holders',
        type=str,
        default='1,2,3,4',
        help='Comma-separated list of holder numbers (default: 1,2,3,4)'
    )
    
    parser.add_argument(
        '--loop',
        action='store_true',
        help='Continuously loop through holders'
    )
    
    parser.add_argument(
        '--delay',
        type=float,
        default=0.5,
        help='Delay in seconds between trigger and wait (default: 0.5)'
    )
    
    args = parser.parse_args()
    
    holders = parse_holders(args.holders)
    
    controller = HolderExchangeController(
        holders=holders,
        loop=args.loop,
        delay=args.delay
    )
    
    controller.run()


if __name__ == '__main__':
    main()
