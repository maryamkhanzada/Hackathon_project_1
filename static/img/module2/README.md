# Module 2 Images and Diagrams

This directory contains diagrams, screenshots, and illustrations for Module 2: Digital Twin.

## Required Diagrams

### Physics Simulation Fundamentals
- `rigid_body_forces.svg` - Newton-Euler equations illustrated on robot link
- `integration_methods.svg` - Comparison of Euler, Semi-Implicit, RK4
- `collision_broadphase.svg` - BVH tree structure visualization
- `collision_narrowphase.svg` - GJK algorithm steps
- `collision_shapes.png` - Comparison table: sphere, box, cylinder, capsule, mesh
- `contact_forces.svg` - Normal force, friction, and contact point diagram
- `physics_engines_comparison.png` - Feature matrix table (ODE, Bullet, DART, MuJoCo)

### Gazebo & Unity Setup
- `gazebo_architecture.svg` - Gazebo component architecture (server, client, plugins)
- `gazebo_gui_annotated.png` - Screenshot with UI elements labeled
- `unity_editor_layout.png` - Unity Editor with Robotics Hub panels
- `ros_unity_bridge.svg` - Communication flow diagram
- `gazebo_vs_unity.png` - Comparison table

### Simulating Sensors
- `camera_parameters.svg` - FOV, focal length, image plane geometry
- `camera_noise_comparison.png` - Clean vs. noisy image comparison
- `depth_camera_principle.svg` - How depth cameras work (stereo/structured light)
- `lidar_2d_scan_pattern.svg` - 2D LiDAR sweep visualization
- `lidar_3d_pattern.svg` - 3D LiDAR multi-beam pattern
- `imu_components.svg` - Accelerometer, gyroscope, magnetometer illustrated
- `imu_noise_plot.png` - Graph showing noise characteristics over time
- `sensor_noise_types.svg` - Gaussian, uniform, outlier visualizations
- `force_torque_sensor.svg` - FT sensor placement on joint

### Collisions & Dynamics
- `visual_vs_collision_mesh.png` - Side-by-side comparison
- `capsule_advantages.svg` - Why capsules work well for limbs
- `inertia_tensor.svg` - Moment of inertia visualization
- `friction_coefficients.png` - Table with material pairs
- `contact_stiffness_effects.png` - Three panels: too soft, optimal, too stiff
- `joint_types.svg` - Revolute, prismatic, fixed joints illustrated
- `damping_friction_graph.png` - Effect on joint motion over time

### Simulated Humanoid Lab
- `humanoid_urdf_structure.svg` - Link-joint tree diagram
- `sensor_placement.png` - Humanoid with sensor locations labeled
- `gazebo_world_setup.png` - Screenshot of test environment
- `rviz_visualization.png` - RViz showing robot, TF, sensors
- `controller_architecture.svg` - Joint controller block diagram
- `validation_results.png` - Comparison graphs (sim vs. theory)

## Diagram Specifications

### Vector Graphics (SVG)
- **Tool**: Inkscape, Draw.io, Illustrator
- **Style**: Clean, professional, consistent color palette
- **Text**: Readable fonts (Arial, Roboto, 12pt minimum)
- **Export**: SVG with embedded fonts

### Raster Images (PNG)
- **Resolution**: 1920x1080 for screenshots, 1200px width for diagrams
- **Format**: PNG with transparency where applicable
- **Compression**: Optimized (use pngquant or similar)

### Screenshots
- **Tools**: Gazebo, Unity, RViz, terminal
- **Annotation**: Use arrows, labels, highlights (red boxes/circles)
- **Crop**: Remove unnecessary UI elements
- **Quality**: High DPI (2x for retina displays)

## Color Palette

Consistent colors across all diagrams:

- **Primary**: `#1976D2` (Blue) - Main elements, headers
- **Secondary**: `#388E3C` (Green) - Success, positive
- **Accent**: `#F57C00` (Orange) - Highlights, warnings
- **Error**: `#D32F2F` (Red) - Errors, dangers
- **Neutral**: `#757575` (Gray) - Text, backgrounds
- **Code**: `#263238` (Dark Blue-Gray) - Code blocks

## Placeholders

For development, simple placeholder boxes with text labels are acceptable. Use tools like:
- https://via.placeholder.com/1200x600.png?text=Diagram+Name
- Or create simple SVG placeholders

## Image Sources

- **Original Work**: Create all diagrams from scratch
- **Stock Photos**: Unsplash, Pexels (CC0 license)
- **Robot Models**: From URDF examples, open-source projects
- **Screenshots**: From actual Gazebo/Unity simulations
