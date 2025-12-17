# Module 3 Images and Diagrams - NVIDIA Isaac

This directory contains diagrams, screenshots, and illustrations for Module 3: NVIDIA Isaac Platform.

## Required Diagrams

### Platform Overview
- `isaac_ecosystem_architecture.svg` - High-level diagram showing Isaac Sim, Isaac ROS, Isaac Gym, Omniverse
- `isaac_vs_gazebo_comparison.png` - Feature comparison table
- `isaac_workflow.svg` - Development workflow: sim → train → deploy
- `gpu_acceleration_benefits.png` - Bar chart comparing CPU vs. GPU performance

### Isaac Sim
- `isaac_sim_architecture.svg` - Layered architecture (Kit → USD → PhysX/RTX → GPU)
- `usd_composition.svg` - How USD layers combine (references, variants, overrides)
- `omniverse_nucleus.svg` - Collaboration workflow with Nucleus server
- `ros2_bridge_diagram.svg` - Message flow between Isaac Sim and ROS 2
- `replicator_pipeline.svg` - Synthetic data generation flowchart
- `camera_intrinsics.svg` - FOV, focal length, principal point illustrated
- `isaac_sim_gui_annotated.png` - Screenshot with UI elements labeled

### Isaac ROS
- `isaac_ros_architecture.svg` - GEM (GPU Engine Module) concept diagram
- `zero_copy_pipeline.svg` - Data flow staying on GPU (camera → processing → output)
- `visual_slam_pipeline.svg` - cuVSLAM components (feature extraction, tracking, mapping)
- `stereo_matching_sgm.svg` - Semi-Global Matching algorithm visualization
- `tensorrt_optimization.svg` - ONNX → TensorRT conversion flow
- `jetson_deployment.svg` - Isaac ROS packages on Jetson AGX Orin
- `perception_performance_comparison.png` - Graph: FPS comparison CPU vs. GPU

### Isaac Gym
- `parallel_environments.svg` - 4096 environments in parallel on single GPU
- `gpu_physics_advantage.svg` - Traditional (CPU) vs. Isaac Gym (GPU) training loop
- `rl_training_speedup.png` - Bar chart: training time comparison
- `actor_critic_architecture.svg` - PPO policy and value network diagram
- `domain_randomization_visual.svg` - Examples of randomized parameters (mass, friction, lighting)
- `asymmetric_actor_critic.svg` - Privileged information for critic during training

### Navigation (Nav2)
- `nav2_architecture.svg` - Nav2 stack components (costmap, planner, controller, BT)
- `costmap_layers.svg` - Layer composition (static, obstacle, inflation, voxel)
- `global_local_planner.svg` - Two-level planning (global path, local trajectory)
- `behavior_tree_example.svg` - Sample BT for navigation with recovery
- `isaac_ros_nav2_integration.svg` - How Isaac ROS sensors feed Nav2

### Sim-to-Real
- `sim_to_real_gap_sources.svg` - Four categories: physics, sensors, actuators, environment
- `domain_randomization_params.png` - Table of randomizable parameters
- `progressive_deployment_stages.svg` - 6-stage sim-to-real pipeline
- `system_identification_process.svg` - Measure → Update Sim → Validate loop
- `residual_rl.svg` - Classical controller + learned residual

## Diagram Specifications

### Vector Graphics (SVG)
- **Tool**: Draw.io (free, web-based), Inkscape, or Illustrator
- **Style**: Clean lines, consistent colors, readable fonts
- **Colors**: Use Module 3 palette:
  - NVIDIA Green: `#76B900`
  - GPU Blue: `#0080FF`
  - Warning Orange: `#FFA500`
  - Neutral Gray: `#555555`
- **Fonts**: Sans-serif (Arial, Roboto), minimum 12pt
- **Export**: SVG with text as paths (for portability)

### Raster Images (PNG)
- **Resolution**: 1920px width for full-width diagrams, 1200px for inline
- **DPI**: 144 (2x for retina displays)
- **Compression**: PNG-8 for simple graphics, PNG-24 for photographs
- **Transparency**: Use alpha channel where applicable

### Screenshots
- **Source**: Isaac Sim, RViz2, Jetson terminals, NVIDIA tools
- **Resolution**: Native (don't upscale)
- **Annotations**: Red boxes for highlights, green checkmarks for success, arrows for flow
- **Crop**: Remove unnecessary toolbars and menus
- **Quality**: Lossless PNG format

## Color Palette

Consistent across all diagrams:

```
Primary:   #76B900  (NVIDIA Green)
Secondary: #0080FF  (Isaac Blue)
Accent:    #FFA500  (Warning Orange)
Success:   #00C853  (Green)
Error:     #D32F2F  (Red)
Info:      #2196F3  (Blue)
Neutral:   #757575  (Gray)
Dark:      #212121  (Text)
Light:     #F5F5F5  (Background)
```

## Icon Usage

Use consistent icons for:
- 🖥️ GPU: NVIDIA logo or generic GPU
- 📷 Camera: Camera icon
- 🤖 Robot: Humanoid silhouette
- 🗺️ Map: Grid icon
- ⚡ Speed: Lightning bolt
- 🎯 Target: Bullseye
- ✅ Success: Green checkmark
- ❌ Failure: Red X

## Diagram Templates

### Architecture Diagram Template
```
┌─────────────────────────────────────────┐
│           Component Name                │
├─────────────────────────────────────────┤
│  - Feature 1                            │
│  - Feature 2                            │
│  - Feature 3                            │
└────────────┬────────────────────────────┘
             │
             ↓ (labeled arrow)
┌─────────────────────────────────────────┐
│         Next Component                  │
└─────────────────────────────────────────┘
```

### Comparison Table Template
| Feature | Option A | Option B | Option C |
|---------|----------|----------|----------|
| Speed | ⭐⭐⭐ | ⭐⭐ | ⭐ |
| Accuracy | ⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐ |
| Ease of Use | ⭐⭐ | ⭐⭐⭐ | ⭐ |

## Placeholders

For development:
- Use https://via.placeholder.com/1200x600.png?text=Diagram+Name
- Or create simple SVG placeholders with just text labels
- Annotate with TODO comments listing missing details

## Quality Checklist

Before finalizing diagrams:
- [ ] All text is legible at 100% zoom
- [ ] Color contrast sufficient (WCAG AA standard)
- [ ] Arrows clearly indicate direction/flow
- [ ] Consistent styling across all diagrams
- [ ] No copyrighted content (NVIDIA logo usage approved for educational purposes)
- [ ] SVG files validated (no broken paths)
- [ ] PNG files optimized (use pngcrush or similar)
- [ ] Filenames descriptive and kebab-case
