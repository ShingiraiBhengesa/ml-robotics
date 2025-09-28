#!/usr/bin/env python3
"""
Natural Language Interface Node for Pick and Place System
Converts natural language commands to structured task commands.

Example commands:
- "pick up red ball" -> {'action': 'pick', 'color': 'red', 'object': 'ball'}
- "grab the blue bottle" -> {'action': 'pick', 'color': 'blue', 'object': 'bottle'}
- "pick up yellow block" -> {'action': 'pick', 'color': 'yellow', 'object': 'block'}
"""

import rclpy
import re
from rclpy.node import Node
from std_msgs.msg import String
import json

class NaturalLanguageInterface(Node):
    def __init__(self):
        super().__init__('natural_language_interface')
        
        # Publisher for structured task commands
        self.task_pub = self.create_publisher(String, '/task_cmd', 10)
        
        # Status publisher for user feedback
        self.status_pub = self.create_publisher(String, '/nl_status', 10)
        
        self.get_logger().info("Natural Language Interface ready!")
        self.get_logger().info("Type commands like: 'pick up red ball', 'grab blue bottle', etc.")
        
        # Define supported colors and objects
        self.colors = ['red', 'blue', 'green', 'yellow', 'orange', 'purple', 'white', 'black', 'pink', 'brown']
        self.objects = ['ball', 'bottle', 'cup', 'block', 'box', 'toy', 'can', 'container', 'object', 'item']
        self.actions = ['pick', 'grab', 'get', 'take', 'pickup', 'grasp']
        
        # Start command input loop
        self.start_command_loop()
    
    def parse_command(self, command_text):
        """Parse natural language command into structured format."""
        command_text = command_text.lower().strip()
        
        # Remove common words
        command_text = re.sub(r'\b(the|a|an|up)\b', '', command_text)
        command_text = re.sub(r'\s+', ' ', command_text).strip()
        
        parsed_command = {}
        
        # Extract action
        action_found = None
        for action in self.actions:
            if action in command_text:
                action_found = 'pick'  # Normalize all actions to 'pick'
                break
        
        if not action_found:
            return None
        
        parsed_command['action'] = action_found
        
        # Extract color
        color_found = None
        for color in self.colors:
            if color in command_text:
                color_found = color
                break
        
        if color_found:
            parsed_command['color'] = color_found
        
        # Extract object
        object_found = None
        for obj in self.objects:
            if obj in command_text:
                object_found = obj
                break
        
        if object_found:
            parsed_command['object'] = object_found
        else:
            # If no specific object found, default to 'object'
            parsed_command['object'] = 'object'
        
        return parsed_command if len(parsed_command) > 1 else None
    
    def publish_task_command(self, parsed_command):
        """Publish the parsed command as a task command."""
        try:
            # Convert to the format expected by task_manager
            task_msg = String()
            task_msg.data = str(parsed_command)
            self.task_pub.publish(task_msg)
            
            # Publish status
            status_msg = String()
            status_msg.data = f"✅ Command parsed: {parsed_command}"
            self.status_pub.publish(status_msg)
            
            self.get_logger().info(f"Published task command: {parsed_command}")
            
        except Exception as e:
            error_msg = f"❌ Error publishing command: {str(e)}"
            status_msg = String()
            status_msg.data = error_msg
            self.status_pub.publish(status_msg)
            self.get_logger().error(error_msg)
    
    def start_command_loop(self):
        """Start the interactive command loop."""
        import threading
        
        def input_loop():
            print("\n" + "="*50)
            print("🤖 NATURAL LANGUAGE PICK & PLACE INTERFACE")
            print("="*50)
            print("Commands you can try:")
            print("  • 'pick up red ball'")
            print("  • 'grab blue bottle'") 
            print("  • 'get yellow block'")
            print("  • 'take the green cup'")
            print("  • 'pickup orange toy'")
            print("  • Type 'quit' to exit")
            print("="*50)
            
            while rclpy.ok():
                try:
                    command = input("\n🎯 Enter command: ").strip()
                    
                    if not command:
                        continue
                        
                    if command.lower() in ['quit', 'exit', 'q']:
                        print("👋 Goodbye!")
                        break
                    
                    # Parse the command
                    parsed = self.parse_command(command)
                    
                    if parsed:
                        print(f"✅ Parsed: {parsed}")
                        self.publish_task_command(parsed)
                    else:
                        print("❌ Could not parse command. Try: 'pick up [color] [object]'")
                        print("   Supported colors:", ', '.join(self.colors[:5]) + "...")
                        print("   Supported objects:", ', '.join(self.objects[:5]) + "...")
                        
                except KeyboardInterrupt:
                    print("\n👋 Goodbye!")
                    break
                except EOFError:
                    print("\n👋 Goodbye!")
                    break
                except Exception as e:
                    print(f"❌ Error: {e}")
        
        # Start input loop in separate thread
        input_thread = threading.Thread(target=input_loop, daemon=True)
        input_thread.start()

def main():
    rclpy.init()
    
    try:
        node = NaturalLanguageInterface()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n👋 Natural Language Interface shutting down...")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
