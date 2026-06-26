# packet_loss_simulator.py
import random
import time
import tkinter as tk
from tkinter import messagebox

class PacketLossSimulator:
    """
    Packet loss simulator for simulating channel degradation
    """
    
    DIRECTION_TX_ONLY = 1  # Only lose TX packets
    DIRECTION_RX_ONLY = 2  # Only lose RX packets
    DIRECTION_BOTH = 3     # Bidirectional packet loss

    def __init__(self, loss_probability=0.0, direction=DIRECTION_TX_ONLY):
        """
        Initialize packet loss simulator
        
        Args:
            loss_probability: packet loss probability (0.0 - 1.0)
            direction: loss direction, default TX only
        """
        self.loss_probability = max(0.0, min(1.0, loss_probability))
        self.direction = direction
        self.enabled = False
        self.total_tx_packets = 0
        self.total_rx_packets = 0
        self.lost_tx_packets = 0
        self.lost_rx_packets = 0
        
        # Queue loss specific variables
        self.queue_loss_mode = False
        self.last_command_data = None
        self.queue_running = False
        self.queue_ended_time = 0
        self.correction_started_early = False
        self.queue_ended_by_loss = False

        # Task group loss related variables
        self.current_task_group = None
        self.task_groups = []  # Store all task group info
        self.last_task_group_loss = None  # Last lost task group
        
        # New: target packet loss related
        self.target_loss_mode = False  # Whether target loss mode is enabled
        self.target_task_groups = []   # Task group IDs to lose
        self.target_loss_probability = 1.0  # Target loss probability (default 100%)
        
        # New: manual packet loss trigger
        self.manual_loss_triggered = False
        self.manual_loss_task_group = None
        
        # New: packet loss history
        self.loss_history = []  # Record all packet loss events

        # Statistics
        self.stats = {
            'start_time': time.time(),
            'last_reset_time': time.time(),
        }

    def set_loss_probability(self, probability):
        """Set packet loss probability"""
        self.loss_probability = max(0.0, min(1.0, probability))

    def set_direction(self, direction):
        """Set packet loss direction"""
        self.direction = direction

    def enable(self):
        """Enable packet loss simulation"""
        self.enabled = True
        self.reset_stats()

    def disable(self):
        """Disable packet loss simulation"""
        self.enabled = False

    def enable_queue_loss_mode(self):
        """Enable queue task packet loss mode"""
        self.queue_loss_mode = True
        print("[PacketLossSim] Queue task loss mode enabled")

    def disable_queue_loss_mode(self):
        """Disable queue task packet loss mode"""
        self.queue_loss_mode = False
        print("[PacketLossSim] Queue task loss mode disabled")

    def set_last_command(self, command_data):
        """Set the last successfully sent command"""
        self.last_command_data = command_data
        print(f"[PacketLossSim] Set last command: X={command_data.get('x', 0)}, Y={command_data.get('y', 0)}, angular={command_data.get('angular', 0)}")

    def get_last_command(self):
        """Get last successfully sent command"""
        return self.last_command_data

    def analyze_task_groups(self, speed_queue):
        """
        Analyze speed queue and divide into task groups
        
        Args:
            speed_queue: speed command queue
            
        Returns:
            list: task group list, each containing:
                - group_id: task group ID
                - start_index: start index
                - end_index: end index
                - command_count: command count
                - x_speed: X-direction speed (main speed)
                - y_speed: Y-direction speed
                - angular_speed: angular speed
                - duration: total duration
                - description: task description
        """
        if not speed_queue:
            return []
        
        task_groups = []
        current_group = None
        
        for i, command in enumerate(speed_queue):
            # Determine if command belongs to current task group
            if current_group is None:
                # First command, create new task group
                current_group = {
                    'group_id': len(task_groups),
                    'start_index': i,
                    'end_index': i,
                    'command_count': 1,
                    'x_speed': command.get('x', 0),
                    'y_speed': command.get('y', 0),
                    'angular_speed': command.get('angular', 0),
                    'duration': command.get('duration', 0),
                    'commands': [command]
                }
            else:
                # Check if a new task group should start
                # Rule: start new group when speed direction changes significantly (absolute change exceeds threshold)
                x_change = abs(command.get('x', 0) - current_group['x_speed'])
                y_change = abs(command.get('y', 0) - current_group['y_speed'])
                angular_change = abs(command.get('angular', 0) - current_group['angular_speed'])
                
                # Threshold: speed change exceeds 20% or absolute change exceeds 50
                speed_threshold = 50
                is_speed_changed = (x_change > speed_threshold or 
                                  y_change > speed_threshold or 
                                  angular_change > speed_threshold)
                
                if is_speed_changed:
                    # End current task group, start new task group
                    # Complete current task group info
                    current_group['description'] = self._generate_group_description(current_group)
                    task_groups.append(current_group.copy())
                    
                    # Create new task group
                    current_group = {
                        'group_id': len(task_groups),
                        'start_index': i,
                        'end_index': i,
                        'command_count': 1,
                        'x_speed': command.get('x', 0),
                        'y_speed': command.get('y', 0),
                        'angular_speed': command.get('angular', 0),
                        'duration': command.get('duration', 0),
                        'commands': [command]
                    }
                else:
                    # Belongs to current task group
                    current_group['end_index'] = i
                    current_group['command_count'] += 1
                    current_group['duration'] += command.get('duration', 0)
                    current_group['commands'].append(command)
        
        # Add the last task group
        if current_group is not None:
            current_group['description'] = self._generate_group_description(current_group)
            task_groups.append(current_group)
        
        # Update internal state
        self.task_groups = task_groups
        
        print(f"[PacketLossSim] Analysis complete: {len(task_groups)} task groups")
        for group in task_groups:
            print(f"  Task group {group['group_id']}: {group['description']}, "
                  f"commands {group['start_index']}-{group['end_index']}, "
                  f"duration {group['duration']:.1f}s")
        
        return task_groups

    def _generate_group_description(self, task_group):
        """Generate task group description"""
        x = task_group['x_speed']
        y = task_group['y_speed']
        angular = task_group['angular_speed']
        
        descriptions = []
        
        if abs(x) > 10:
            direction = "Forward" if x > 0 else "Backward"
            descriptions.append(f"{direction} {abs(x)}mm/s")
        
        if abs(y) > 10:
            direction = "Left" if y > 0 else "Right"
            descriptions.append(f"{direction} {abs(y)}mm/s")
        
        if abs(angular) > 10:
            direction = "Counter-clockwise" if angular > 0 else "Clockwise"
            descriptions.append(f"{direction} {abs(angular)}mrad/s")
        
        if not descriptions:
            descriptions.append("Idle")
        
        return " + ".join(descriptions)

    def get_task_group_for_command(self, command_index):
        """
        Get task group by command index
        
        Args:
            command_index: command index in queue
            
        Returns:
            dict: task group info, None if not found
        """
        for group in self.task_groups:
            if group['start_index'] <= command_index <= group['end_index']:
                return group
        return None

    def enable_target_loss_mode(self, task_group_ids=None, probability=1.0):
        """
        Enable target packet loss mode
        
        Args:
            task_group_ids: list of task group IDs to lose, e.g. [1, 3, 5]
            probability: probability of target packet loss (0.0-1.0)
        """
        self.target_loss_mode = True
        self.target_task_groups = task_group_ids or []
        self.target_loss_probability = max(0.0, min(1.0, probability))
        print(f"[PacketLossSim] Target packet loss mode enabled: task_groups={task_group_ids}, probability={probability:.1%}")

    def disable_target_loss_mode(self):
        """Disable target packet loss mode"""
        self.target_loss_mode = False
        self.target_task_groups = []
        print("[PacketLossSim] Target packet loss mode disabled")

    def trigger_manual_loss_for_task_group(self, task_group_id):
        """
        Manually trigger packet loss for a specific task group
        
        Args:
            task_group_id: task group ID to lose
        """
        self.manual_loss_triggered = True
        self.manual_loss_task_group = task_group_id
        print(f"[PacketLossSim] Manually triggered loss for task group {task_group_id}")

    def simulate_queue_loss(self, current_command, next_command=None, 
                           current_task_group=None, next_task_group=None,
                           is_task_change=False, gui_callback=None):
        """
        Simulate queue task packet loss (enhanced, supports multiple loss modes)
        
        Args:
            current_command: currently executing command
            next_command: next command to execute (if any)
            current_task_group: current task group info
            next_task_group: next task group info
            is_task_change: whether this is a task group transition point
            gui_callback: GUI callback for displaying dialog
            
        Returns:
            tuple: (is_lost, start_correction_early, reason)
        """
        if not self.enabled or not self.queue_loss_mode or self.loss_probability <= 0.0:
            return False, False, "Normal execution"

        # Only check for packet loss at task group transitions
        if is_task_change and next_task_group is not None:
            next_group_id = next_task_group.get('group_id')
            command_count = next_task_group.get('command_count', 1)
            self.total_tx_packets += command_count
            
            # Check if packet loss should occur (multiple modes)
            should_loss = False
            loss_reason = ""
            
            # 1. Manual trigger mode (highest priority)
            loss_mode = None
            if self.manual_loss_triggered and self.manual_loss_task_group == next_group_id:
                should_loss = True
                loss_reason = f"Manual trigger task group {next_group_id} lost"
                loss_mode = 'manual'
                self.manual_loss_triggered = False  # reset manual trigger
            
            elif self.target_loss_mode and next_group_id in self.target_task_groups:
                if random.random() < self.target_loss_probability:
                    should_loss = True
                    loss_reason = f"Target task group {next_group_id} packet loss (probability: {self.target_loss_probability:.1%})"
                    loss_mode = 'target'
            
            elif not self.target_loss_mode and random.random() < self.loss_probability:
                should_loss = True
                loss_reason = f"Random task group packet loss (probability: {self.loss_probability:.1%})"
                loss_mode = 'random'
            
            if should_loss:
                self.lost_tx_packets += command_count
                self.last_task_group_loss = next_task_group
                
                loss_record = {
                    'timestamp': time.time(),
                    'task_group_id': next_group_id,
                    'task_group_desc': next_task_group.get('description', 'Unknown'),
                    'command_count': command_count,
                    'duration': next_task_group.get('duration', 0),
                    'reason': loss_reason,
                    'mode': loss_mode or 'random'
                }
                self.loss_history.append(loss_record)
                
                print(f"[PacketLossSim] {loss_reason}")
                print(f"  Lost task group: {next_task_group.get('description', 'Unknown')}")
                print(f"  Lost command count: {command_count}")
                print(f"  Total loss duration: {next_task_group.get('duration', 0):.1f}s")
                
                if gui_callback is not None:
                    response = messagebox.askyesno(
                        "Packet Loss Detected - Task Group Loss",
                        f"Queue task packet loss detected!\n\n"
                        f"Reason: {loss_reason}\n"
                        f"Current task group: {current_task_group.get('description', 'Unknown')}\n"
                        f"Lost task group: {next_task_group.get('description', 'Unknown')}\n"
                        f"Lost command count: {command_count} commands\n"
                        f"Total loss duration: {next_task_group.get('duration', 0):.1f}s\n\n"
                        f"Start correction mode immediately (skip remaining queue)?\n\n"
                        f"Yes: Start correction now using current command\n"
                        f"No: Continue current task group"
                    )
                    
                    if response:
                        self.correction_started_early = True
                        self.queue_ended_by_loss = True
                        return True, True, loss_reason + ", user chose to start correction early"
                    else:
                        self.correction_started_early = False
                        return True, False, loss_reason + ", user chose to continue current task group"
                else:
                    self.correction_started_early = False
                    return True, False, loss_reason + " (no GUI callback)"
        
        return False, False, "Normal execution"

    def simulate_task_group_loss(self, current_task_group, next_task_group, gui_callback=None):
        """
        Simulate task group packet loss specifically
        
        Args:
            current_task_group: current task group
            next_task_group: next task group
            gui_callback: GUI callback
            
        Returns:
            tuple: (is_lost, start_correction_early, reason, lost_task_group_info)
        """
        return self.simulate_queue_loss(
            current_command=None,
            next_command=None,
            current_task_group=current_task_group,
            next_task_group=next_task_group,
            is_task_change=True,
            gui_callback=gui_callback
        )

    def simulate_tx_loss(self, data):
        """
        Simulate TX packet loss (backward compatible)
        
        Args:
            data: data to send
            
        Returns:
            tuple: (is_lost, processed_data, reason)
        """
        self.total_tx_packets += 1

        if not self.enabled or self.loss_probability <= 0.0:
            return False, data, "Normal send"

        # Check if TX packet loss should be simulated (legacy mode)
        if self.direction in [self.DIRECTION_TX_ONLY, self.DIRECTION_BOTH] and not self.queue_loss_mode:
            # Randomly decide whether to lose packet
            if random.random() < self.loss_probability:
                self.lost_tx_packets += 1
                return True, None, f"TX packet loss (probability: {self.loss_probability:.1%})"

        return False, data, "Normal send"

    def simulate_rx_loss(self, data):
        """
        Simulate RX packet loss (only affects received data)
        
        Args:
            data: received data
            
        Returns:
            tuple: (is_lost, processed_data, reason)
        """
        self.total_rx_packets += 1

        if not self.enabled or self.loss_probability <= 0.0:
            return False, data, "Normal receive"

        # Check if RX packet loss should be simulated
        if self.direction in [self.DIRECTION_RX_ONLY, self.DIRECTION_BOTH]:
            # Randomly decide whether to lose packet
            if random.random() < self.loss_probability:
                self.lost_rx_packets += 1
                return True, None, f"RX packet loss (probability: {self.loss_probability:.1%})"

        return False, data, "Normal receive"

    def reset_stats(self):
        """Reset statistics"""
        self.total_tx_packets = 0
        self.total_rx_packets = 0
        self.lost_tx_packets = 0
        self.lost_rx_packets = 0
        self.correction_started_early = False
        self.queue_ended_by_loss = False
        self.task_groups = []
        self.last_task_group_loss = None
        self.loss_history = []
        self.target_loss_mode = False
        self.target_task_groups = []
        self.manual_loss_triggered = False
        self.manual_loss_task_group = None
        self.stats['last_reset_time'] = time.time()

    def get_stats(self):
        """Get statistics"""
        # Calculate TX loss rate
        if self.total_tx_packets == 0:
            tx_loss_rate = 0.0
        else:
            tx_loss_rate = self.lost_tx_packets / self.total_tx_packets
        
        # Calculate RX loss rate
        if self.total_rx_packets == 0:
            rx_loss_rate = 0.0
        else:
            rx_loss_rate = self.lost_rx_packets / self.total_rx_packets

        elapsed_time = time.time() - self.stats['last_reset_time']

        # Calculate task group loss statistics
        total_task_groups = len(self.task_groups)
        lost_task_groups = len([h for h in self.loss_history if h['mode'] != 'manual'])
        manual_triggered_losses = len([h for h in self.loss_history if h['mode'] == 'manual'])
        
        # Calculate average loss size (command count)
        avg_loss_size = 0
        if self.loss_history:
            avg_loss_size = sum(h['command_count'] for h in self.loss_history) / len(self.loss_history)

        stats = {
            'enabled': self.enabled,
            'probability': self.loss_probability,
            'direction': self.direction,
            'queue_loss_mode': self.queue_loss_mode,
            'target_loss_mode': self.target_loss_mode,
            'total_tx_packets': self.total_tx_packets,
            'lost_tx_packets': self.lost_tx_packets,
            'tx_loss_rate': tx_loss_rate,
            'total_rx_packets': self.total_rx_packets,
            'lost_rx_packets': self.lost_rx_packets,
            'rx_loss_rate': rx_loss_rate,
            'elapsed_time': elapsed_time,
            'tx_packets_per_second': self.total_tx_packets / elapsed_time if elapsed_time > 0 else 0,
            'rx_packets_per_second': self.total_rx_packets / elapsed_time if elapsed_time > 0 else 0,
            'correction_started_early': self.correction_started_early,
            'queue_ended_by_loss': self.queue_ended_by_loss,
            'total_task_groups': total_task_groups,
            'lost_task_groups': lost_task_groups,
            'manual_triggered_losses': manual_triggered_losses,
            'total_loss_events': len(self.loss_history),
            'avg_loss_size_commands': avg_loss_size,
            'last_lost_task_group': self.last_task_group_loss,
            'target_task_groups': self.target_task_groups,
            'target_loss_probability': self.target_loss_probability,
        }

        return stats

    def get_stats_text(self):
        """Get statistics as text"""
        stats = self.get_stats()
        
        direction_text = {
            self.DIRECTION_TX_ONLY: "TX only",
            self.DIRECTION_RX_ONLY: "RX only",
            self.DIRECTION_BOTH: "Both directions"
        }.get(stats['direction'], "Unknown")

        mode_text = "Queue task loss mode" if stats['queue_loss_mode'] else "Normal loss mode"
        target_mode_text = "Enabled" if stats['target_loss_mode'] else "Disabled"

        text = f"Packet loss simulation: {'Enabled' if stats['enabled'] else 'Disabled'}\n"
        text += f"Loss mode: {mode_text}\n"
        text += f"Loss direction: {direction_text}\n"
        text += f"Base probability: {stats['probability']:.1%}\n"
        text += f"Target loss: {target_mode_text}\n"
        
        if stats['queue_loss_mode']:
            text += f"Task group stats: {stats['total_task_groups']} total, {stats['lost_task_groups']} lost\n"
            text += f"Loss events: {stats['total_loss_events']} total (including {stats['manual_triggered_losses']} manual)\n"
            text += f"Average loss size: {stats['avg_loss_size_commands']:.1f} commands\n"

            if stats['target_loss_mode']:
                text += f"Target task groups: {stats['target_task_groups']}\n"
                text += f"Target probability: {stats['target_loss_probability']:.1%}\n"

            if stats['last_lost_task_group']:
                lost_group = stats['last_lost_task_group']
                text += f"Last lost task group: {lost_group.get('description', 'Unknown')}\n"
                text += f"Lost command count: {lost_group.get('command_count', 0)}\n"
        
        text += f"TX stats:\n"
        text += f"  Total packets: {stats['total_tx_packets']}\n"
        text += f"  Lost packets: {stats['lost_tx_packets']}\n"
        text += f"  Loss rate: {stats['tx_loss_rate']:.1%}\n"
        text += f"RX stats:\n"
        text += f"  Total packets: {stats['total_rx_packets']}\n"
        text += f"  Lost packets: {stats['lost_rx_packets']}\n"
        text += f"  Loss rate: {stats['rx_loss_rate']:.1%}\n"
        text += f"Queue status:\n"
        text += f"  Correction started early: {'Yes' if stats['correction_started_early'] else 'No'}\n"
        text += f"  Queue ended by loss: {'Yes' if stats['queue_ended_by_loss'] else 'No'}"

        return text
    
    def get_task_group_info(self):
        """Get task group information"""
        if not self.task_groups:
            return "Task groups not analyzed"
        
        info = f"Task group analysis results ({len(self.task_groups)} groups):\n"
        for group in self.task_groups:
            info += f"  Task group {group['group_id']}: {group['description']}\n"
            info += f"    Command range: {group['start_index']}-{group['end_index']}\n"
            info += f"    Command count: {group['command_count']}\n"
            info += f"    Total duration: {group['duration']:.1f}s\n"
            
            if self.target_loss_mode and group['group_id'] in self.target_task_groups:
                info += f"    ⚠ Marked as loss target ({self.target_loss_probability:.0%})\n"
        
        return info
    
    def get_loss_history_text(self, max_entries=10):
        """Get packet loss history text"""
        if not self.loss_history:
            return "No packet loss history"
        
        history_text = f"Packet loss history (latest {min(max_entries, len(self.loss_history))} entries):\n"
        
        recent_history = sorted(self.loss_history, key=lambda x: x['timestamp'], reverse=True)[:max_entries]
        
        for i, record in enumerate(recent_history):
            timestamp = time.strftime("%H:%M:%S", time.localtime(record['timestamp']))
            history_text += f"\n{i+1}. [{timestamp}] {record['mode']}\n"
            history_text += f"   Task group {record['task_group_id']}: {record['task_group_desc']}\n"
            history_text += f"   Lost commands: {record['command_count']}, duration: {record['duration']:.1f}s\n"
            history_text += f"   Reason: {record['reason']}\n"
        
        return history_text
    
    def clear_loss_history(self):
        """Clear packet loss history"""
        self.loss_history = []
        print("[PacketLossSim] Loss history cleared")
    
    def get_latest_loss_record(self):
        """Get latest loss record"""
        if not self.loss_history:
            return None
        
        # Sort by timestamp, return latest record
        return sorted(self.loss_history, key=lambda x: x['timestamp'], reverse=True)[0]