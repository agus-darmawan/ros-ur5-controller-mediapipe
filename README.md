# UR5 Controller ROS2 Humble Mediapie

## Table of Contents

- [Introduction](#introduction)
- [Prerequisites](#prerequisites)
- [Package Description](#package-description)
- [Installation](#installation)
- [Usage](#usage)

## Introduction

This repository contains code for controlling a UR5 Robot using Mediapie like picture below.

<p align="center">
    <img src="docs/detection.png" alt="Software Architecture" />
</p>

The software architecture consists of three main nodes (Perception Node, Controller Node, UR Interface) and an RViz GUI for visualization and interaction.

## Prerequisites

Before you begin, ensure you have met the following requirements:

- Ubuntu 20.04
- ROS1 Noetic
- 1 Network connection with a ESP of the robot

## Package Description

The package is structured as follows:

- **Perception Node**: Handles camera data and processes using mediapie for the controller node.
- **Controller Node**: Manages the logic for controlling the UR5 robot based on input from the perception.
- **UR Interface**: Service interface for the controller node.

## Installation

Follow these steps to install and set up the package:

1. **Clone the repository**:

   ```bash
   git clone git@github.com:agus-darmawan/ros-ur5-controller-mediapipe.git
   cd ros-ur5-controller-mediapipe
   ```

2. **Build the package**:

   ```bash
   catkin_make
   ```

3. **Source the setup script**:
   ```bash
   source devel/setup.bash
   ```

## Usage

To run the system, follow these steps:

1. **Run the Perception node**:

   ```bash
   rosrun ur_perception hand_tracker_node.py 'source'
   ```

   note : source can be path of video file of source of the camera

2. **Run the Controller node**:
   ```bash
   rosrun ur_controller ur_controller_node.py 'API'
   ```
   note : API is API of the send comand
