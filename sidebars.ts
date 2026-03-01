import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: ROS 2 Fundamentals',
      collapsed: false,
      items: [
        {
          type: 'category',
          label: 'Week 1-2: Physical AI Introduction',
          items: [
            'module-1-ros2/week-1-2-physical-ai-intro/foundations',
            'module-1-ros2/week-1-2-physical-ai-intro/embodied-intelligence',
            'module-1-ros2/week-1-2-physical-ai-intro/sensor-systems',
          ],
        },
        {
          type: 'category',
          label: 'Week 3-5: ROS 2 Fundamentals',
          items: [
            'module-1-ros2/week-3-5-ros2-fundamentals/ros2-architecture',
            'module-1-ros2/week-3-5-ros2-fundamentals/nodes-topics-services',
            'module-1-ros2/week-3-5-ros2-fundamentals/launch-files',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin Simulation',
      collapsed: false,
      items: [
        {
          type: 'category',
          label: 'Week 6-7: Simulation Environments',
          items: [
            'module-2-digital-twin/week-6-7-simulation/gazebo-setup',
            'module-2-digital-twin/week-6-7-simulation/unity-integration',
            'module-2-digital-twin/week-6-7-simulation/sensor-simulation',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac Platform',
      collapsed: false,
      items: [
        {
          type: 'category',
          label: 'Week 8-10: Isaac Ecosystem',
          items: [
            'module-3-nvidia-isaac/week-8-10-isaac-platform/isaac-sdk',
            'module-3-nvidia-isaac/week-8-10-isaac-platform/isaac-sim',
            'module-3-nvidia-isaac/week-8-10-isaac-platform/isaac-ros',
            'module-3-nvidia-isaac/week-8-10-isaac-platform/nav2',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action',
      collapsed: false,
      items: [
        {
          type: 'category',
          label: 'Week 11-13: Conversational Robotics',
          items: [
            'module-4-vla/week-11-13-conversational-robotics/humanoid-kinematics',
            'module-4-vla/week-11-13-conversational-robotics/bipedal-locomotion',
            'module-4-vla/week-11-13-conversational-robotics/conversational-robotics',
          ],
        },
      ],
    },
  ],
};

export default sidebars;
