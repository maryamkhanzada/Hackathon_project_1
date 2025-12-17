# Module 3 Videos - NVIDIA Isaac

This directory contains video demonstrations for Module 3: NVIDIA Isaac Platform.

## Required Videos

### Isaac Sim
- `isaac_sim_walkthrough.mp4` - GUI walkthrough and basic navigation
- `urdf_import_demo.mp4` - Importing robot URDF into Isaac Sim
- `photorealistic_rendering.mp4` - Comparison of rendering modes (path tracing vs. ray traced)
- `sensor_visualization.mp4` - Camera, LiDAR, depth sensors in action
- `synthetic_data_generation.mp4` - Replicator API generating labeled datasets
- `ros2_bridge_demo.mp4` - ROS 2 topics publishing from Isaac Sim

### Isaac ROS
- `isaac_ros_installation.mp4` - Docker setup and first launch
- `visual_slam_demo.mp4` - cuVSLAM tracking in real-time
- `stereo_depth_comparison.mp4` - CPU vs. GPU stereo processing side-by-side
- `detectnet_inference.mp4` - Object detection with TensorRT
- `perception_pipeline.mp4` - Complete VSLAM + depth + detection running together

### Isaac Gym
- `parallel_training.mp4` - 4096 environments training simultaneously
- `humanoid_learning_progression.mp4` - Time-lapse of policy learning to walk
- `domain_randomization.mp4` - Varying physics parameters during training
- `trained_policy_deployment.mp4` - Sim-trained policy on real humanoid

### Navigation (Nav2 + Isaac ROS)
- `nav2_integration.mp4` - Isaac ROS sensors feeding Nav2 costmap
- `autonomous_navigation.mp4` - Robot navigating to goal using vision
- `dynamic_obstacle_avoidance.mp4` - Nav2 replanning around moving obstacles

### Sim-to-Real
- `sim_vs_real_comparison.mp4` - Split screen of simulated and real robot performing same task
- `progressive_deployment.mp4` - Staged deployment from sim to real
- `domain_rand_effectiveness.mp4` - Policy with/without randomization on real robot

## Video Specifications

- **Format**: MP4 (H.264 codec, AAC audio)
- **Resolution**: 1920x1080 (1080p preferred) or 1280x720 (720p acceptable)
- **Frame Rate**: 30 fps (60 fps for fast motion)
- **Duration**: 30-120 seconds per video
- **Bitrate**: 5-10 Mbps for smooth quality
- **Captions**: Include on-screen text annotations for key points

## Recording Tools

**Screen Capture**:
- OBS Studio (cross-platform, free)
- SimpleScreenRecorder (Linux)
- NVIDIA ShadowPlay (for GPU-accelerated recording)

**Video Editing**:
- DaVinci Resolve (free, professional)
- Kdenlive (open-source)
- OpenShot (beginner-friendly)

**Annotations**:
- Add arrows, bounding boxes, text overlays to highlight features
- Use consistent color scheme (green for success, red for errors, blue for info)

## Placeholders

For development, use:
- Placeholder videos with title cards
- Screen recordings at lower resolution
- Link to external hosting (YouTube unlisted) if files too large

## Recording Checklist

When creating videos:
- [ ] Clean environment (no personal files visible)
- [ ] Smooth camera movements (no jerky panning)
- [ ] Clear audio if narration included
- [ ] Demonstrate both success and failure cases
- [ ] Show performance metrics (FPS, latency) where relevant
- [ ] Include timestamps or progress indicators
- [ ] Test playback on different devices
