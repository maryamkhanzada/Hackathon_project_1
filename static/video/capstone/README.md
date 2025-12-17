# Capstone Project - Video Assets

This directory contains demonstration videos for the complete autonomous humanoid assistant capstone project.

## Video List

### 1. Complete System Demo (`full_system_demo.mp4`)
**Description**: End-to-end demonstration of the autonomous humanoid assistant performing multiple tasks.
- Voice command: "Go to the kitchen and bring me a cup"
- Shows complete pipeline: Whisper → GPT-4 → Isaac ROS → Nav2 → Manipulation
- Includes task planning visualization, object detection, navigation, and grasping
- **Duration**: 3-4 minutes
- **Resolution**: 1920x1080

### 2. Multi-Modal Interaction Demo (`multimodal_demo.mp4`)
**Description**: Demonstration of voice + vision + gesture interaction.
- User points and says: "Pick up that object"
- System fuses modalities to identify correct target
- Shows visual grounding with CLIP
- **Duration**: 1-2 minutes
- **Resolution**: 1920x1080

### 3. Task Planning Breakdown (`planning_visualization.mp4`)
**Description**: Visualization of GPT-4 task decomposition.
- Complex command: "Clean the table in the kitchen"
- Shows step-by-step plan generation
- Displays action sequence with parameters
- **Duration**: 1 minute
- **Resolution**: 1920x1080

### 4. Navigation Test Suite (`navigation_tests.mp4`)
**Description**: Robot navigating through various environments.
- Static obstacles
- Dynamic obstacles (moving objects)
- Narrow passages
- Success and failure cases
- **Duration**: 2-3 minutes
- **Resolution**: 1920x1080

### 5. Manipulation Task Demo (`manipulation_demo.mp4`)
**Description**: Object grasping and placement tasks.
- Visual object detection with YOLO
- Grasp pose computation
- MoveIt2 motion planning
- Successful grasps and edge cases
- **Duration**: 2 minutes
- **Resolution**: 1920x1080

### 6. Isaac Sim Training (`isaac_sim_training.mp4`)
**Description**: Robot training in Isaac Sim environment.
- Scene setup with humanoid robot
- Reinforcement learning training (time-lapse)
- Policy deployment and testing
- Sim-to-real transfer preparation
- **Duration**: 2-3 minutes
- **Resolution**: 1920x1080

### 7. Error Recovery Demo (`error_recovery_demo.mp4`)
**Description**: System handling failures gracefully.
- Navigation blocked → replanning
- Object not found → requesting clarification
- Manipulation failure → retry with adjusted pose
- Shows robustness and recovery behaviors
- **Duration**: 2 minutes
- **Resolution**: 1920x1080

### 8. Edge Deployment (`jetson_deployment.mp4`)
**Description**: System running on Jetson AGX Orin.
- Real-time inference on edge device
- Performance metrics overlay (FPS, latency, power)
- Full autonomous operation without cloud
- **Duration**: 2 minutes
- **Resolution**: 1920x1080

### 9. Multi-Robot Coordination (`fleet_coordination.mp4`)
**Description**: Multiple humanoid robots collaborating on tasks.
- Fleet manager allocating tasks
- Collision avoidance between robots
- Coordinated object transport
- **Duration**: 2-3 minutes
- **Resolution**: 1920x1080

### 10. Human-Robot Interaction (`hri_demo.mp4`)
**Description**: Natural interaction scenarios.
- Voice commands with contextual understanding
- Gesture-based control
- Continuous learning from feedback
- Explainable decision-making (robot explains its actions)
- **Duration**: 2-3 minutes
- **Resolution**: 1920x1080

## Recording Guidelines

### Equipment
- **Camera**: 1080p minimum, 4K preferred for Isaac Sim recordings
- **Screen Capture**: OBS Studio or SimpleScreenRecorder for Isaac Sim + RViz
- **Microphone**: Clear audio for voice commands and narration
- **Lighting**: Well-lit environment for real robot footage

### Content Requirements
1. **Split-screen layout**: Isaac Sim on left, RViz/terminal on right
2. **Overlay text**: Display current task, robot state, and key metrics
3. **Audio narration**: Explain what's happening at each step
4. **Real-time indicators**: Show timestamps, FPS, processing latency
5. **Multiple angles**: Combine 3rd-person, camera POV, and system views

### Recording Settings
- **Container**: MP4 (H.264 codec)
- **Resolution**: 1920x1080 (1080p) or 3840x2160 (4K)
- **Frame rate**: 30 fps (60 fps for smooth motion)
- **Bitrate**: 8-15 Mbps for 1080p, 25-40 Mbps for 4K
- **Audio**: AAC, 192-256 kbps, stereo

### Editing Recommendations
1. **Intro**: 5-second title card with project name
2. **Chapters**: Add chapter markers for multi-part demos
3. **Speed up**: Use 1.5-2x speed for slow processes (navigation, training)
4. **Captions**: Add subtitles for voice commands and key events
5. **Outro**: Show final results, metrics, and credits

### File Naming Convention
- Use descriptive snake_case names
- Include video type: `capstone_feature_description.mp4`
- Version number if multiple takes: `full_system_demo_v2.mp4`
- Keep filenames under 50 characters

## Screen Recording Commands

### OBS Studio Setup
```bash
# Install OBS
sudo apt install obs-studio

# Recommended settings:
# - Output: Recording
# - Format: MP4
# - Encoder: NVIDIA NVENC H.264 (if available) or x264
# - Rate Control: CBR
# - Bitrate: 10000 Kbps
# - Preset: High Quality
```

### Command-Line Recording
```bash
# Simple screen recording with ffmpeg
ffmpeg -video_size 1920x1080 -framerate 30 -f x11grab -i :0.0 \
  -c:v libx264 -preset medium -crf 23 \
  output.mp4

# With audio from microphone
ffmpeg -video_size 1920x1080 -framerate 30 -f x11grab -i :0.0 \
  -f alsa -i default \
  -c:v libx264 -preset medium -crf 23 \
  -c:a aac -b:a 192k \
  output.mp4
```

### ROS Bag to Video
```bash
# Convert ROS bag camera topic to video
ros2 bag play recording.bag

# In another terminal
ros2 run image_view video_recorder \
  image:=/camera/image_raw \
  _filename:=camera_recording.avi
```

## Placeholder Instructions

Until videos are recorded, this directory serves as a placeholder. The Docusaurus site will render correctly without actual video files, using alt text and descriptions.

## Copyright and Attribution

All videos should be:
- **Original content** created for this project
- **Properly licensed** with appropriate open-source license (MIT, Apache 2.0)
- **Attributed** with credits to contributors, NVIDIA Isaac Sim, ROS 2, etc.
- **Reproducible**: Include setup instructions so others can recreate demonstrations

## Submission Guidelines

For students submitting capstone projects:

1. **Required**: At least one complete system demo video (Video #1)
2. **Optional**: Additional videos showing specific features or extensions
3. **Format**: Upload to YouTube or institutional video hosting
4. **Links**: Provide video URLs in your technical report
5. **Transcripts**: Include video transcripts for accessibility

---

**Note**: High-quality demonstration videos are crucial for showcasing your capstone project to employers, graduate schools, or research publications. Invest time in professional recording and editing.
