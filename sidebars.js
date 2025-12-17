/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */

// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  // Main tutorial sidebar for the book
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      link: {
        type: 'doc',
        id: 'intro/index',
      },
      items: [],
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2 Basics',
      link: {
        type: 'doc',
        id: 'module1-ros2/index',
      },
      items: [
        'module1-ros2/ros2-architecture',
        'module1-ros2/rclpy-integration',
        'module1-ros2/urdf-humanoids',
        'module1-ros2/ros2-exercises',
        'module1-ros2/ros2-package-project',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin',
      link: {
        type: 'doc',
        id: 'module2-digital-twin/index',
      },
      items: [
        'module2-digital-twin/physics-simulation',
        'module2-digital-twin/gazebo-unity-setup',
        'module2-digital-twin/simulating-sensors',
        'module2-digital-twin/collisions-dynamics',
        'module2-digital-twin/simulated-humanoid-lab',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac',
      link: {
        type: 'doc',
        id: 'module3-nvidia-isaac/index',
      },
      items: [
        'module3-nvidia-isaac/isaac-platform-overview',
        'module3-nvidia-isaac/isaac-sim-photorealistic',
        'module3-nvidia-isaac/isaac-ros-perception',
        'module3-nvidia-isaac/nav2-path-planning',
        'module3-nvidia-isaac/rl-robot-control',
        'module3-nvidia-isaac/ai-perception-pipeline-lab',
        'module3-nvidia-isaac/sim-to-real-transfer',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action',
      link: {
        type: 'doc',
        id: 'module4-vla/index',
      },
      items: [
        'module4-vla/llm-robotics-planning',
        'module4-vla/whisper-voice-action',
        'module4-vla/multi-modal-interaction',
        'module4-vla/voice-guided-lab',
      ],
    },
    {
      type: 'category',
      label: 'Capstone Project',
      link: {
        type: 'doc',
        id: 'capstone-project/index',
      },
      items: [
        'capstone-project/implementation-guide',
        'capstone-project/integration-ros2-isaac-vla',
        'capstone-project/testing-troubleshooting',
        'capstone-project/optional-extensions',
      ],
    },
    {
      type: 'category',
      label: 'Appendices',
      link: {
        type: 'doc',
        id: 'appendices/index',
      },
      items: [
        'appendices/hardware-software',
        'appendices/glossary',
        'appendices/recommended-reading',
        'appendices/lab-setup-tips',
      ],
    },
  ],
};

module.exports = sidebars;
