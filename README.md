# Lynxmotion Pick & Place System

A complete ROS2-based robotic pick-and-place system integrating:
- **ZED-M stereo camera** for 3D vision
- **GroundingDINO VLM** for intelligent object detection  
- **5-DOF Lynxmotion robotic arm** with custom inverse kinematics
- **Arduino-based hardware control**

## System Architecture

```
ZED Camera → VLM Detection → Depth Sampling → Target Selection → IK Solver → Arduino Bridge → Robot Arm
     ↓              ↓              ↓              ↓              ↓              ↓
  RGB/Depth    Detections    3D Poses      Selected Target   Joint Commands   Servo Commands
```

## Quick Start

### 1. Install Dependencies

```bash
# Install ROS2 dependencies
sudo apt update
sudo apt install ros-humble-robot-state-publisher ros-humble-joint-state-publisher \
                 ros-humble-xacro ros-humble-tf2-ros ros-humble-cv-bridge

# Install Python dependencies
pip install -r requirements.txt
```

### 2. Install ZED SDK and ROS2 Wrapper

Follow the [ZED ROS2 installation guide](https://github.com/stereolabs/zed-ros2-wrapper) to install:
- ZED SDK
- ZED ROS2 wrapper packages

### 3. Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select lynxmotion_pick_place
source install/setup.bash
```

### 4. Launch the Complete System

```bash
# Launch all components
ros2 launch lynxmotion_pick_place bringup.launch.py

# In a new terminal, send a task command
ros2 topic pub /task_cmd std_msgs/String "data: \"{'color': 'red', 'object': 'ball'}\""
```

## Hardware Setup

### Arduino Configuration
- Connect Lynxmotion arm to Arduino
- Upload servo control firmware
- Update the port in launch file: `port: '/dev/ttyUSB0'`

### ZED Camera
- Mount ZED-M camera above the robot workspace
- Adjust camera position in `urdf/lynxmotion_arm.urdf.xacro` if needed
- Calibrate camera if required

## Configuration

### Robot Parameters
Edit `config/arm_config.yaml`:
- Link lengths (base_height, shoulder_length, etc.)
- Joint limits for safety

### Camera Settings  
Edit `config/zedm.yaml`:
- Resolution and quality settings
- Depth sensing parameters

## Testing

### Run Integration Test
```bash
# Start the system
ros2 launch lynxmotion_pick_place bringup.launch.py

# In another terminal, run the test
ros2 run lynxmotion_pick_place pipeline_integration_test
```

### Manual Testing
```bash
# Monitor topics
ros2 topic echo /detections
ros2 topic echo /selected_target  
ros2 topic echo /arm/joint_cmd

# Send test commands
ros2 topic pub /task_cmd std_msgs/String "data: \"{'color': 'blue', 'object': 'cup'}\""
```

## Architecture Details

### Node Communication
- `vlm_detector_node`: Processes RGB images, publishes object detections
- `depth_sampler_node`: Converts 2D detections to 3D poses using depth
- `target_selector_node`: Selects best target, transforms to robot coordinates  
- `task_manager_node`: Orchestrates tasks and validates targets
- `ik_node`: Computes inverse kinematics for target poses
- `arduino_bridge_node`: Sends joint commands to hardware

### Topics
- `/zed/left/image_rect_color` - RGB camera feed
- `/zed/depth/depth_registered` - Depth images  
- `/detections` - Object detection results
- `/target_candidates` - 3D target poses
- `/selected_target` - Final target selection
- `/arm/joint_cmd` - Joint angle commands
- `/task_cmd` - High-level task commands

## Safety Features
- Joint limit checking in IK solver
- Workspace boundary validation
- Dry-run mode for testing without hardware
- Emergency stop capabilities

## Troubleshooting

### Common Issues
1. **VLM Model Download**: First run downloads GroundingDINO model (~300MB)
2. **ZED Camera**: Ensure ZED SDK installed and camera connected
3. **Arduino**: Check serial port permissions: `sudo chmod 666 /dev/ttyUSB0`
4. **TF Transforms**: Verify robot description is published

### Debug Commands
```bash
# Check node status
ros2 node list
ros2 topic list

# View transforms
ros2 run tf2_tools view_frames

# Monitor system performance
ros2 topic hz /detections
```

## License
MIT License - See LICENSE file for details.
