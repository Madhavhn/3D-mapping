# 3D-mapping
# Drone Simulation and Mapping System

This repository contains configuration files for a drone simulation and mapping system. It includes various ROS (Robot Operating System) launch files for running a drone simulation, generating Octomap maps, and processing point clouds from depth and RGB cameras.

## Directory Structure

- `drone_simulation.launch`: Configures the drone simulation environment, including sensors and state estimation.
- `map_server.launch`: Configures the Octomap server for generating 3D maps from point cloud data.
- `depth_rgb_point_cloud.launch`: Configures the processing of depth and RGB camera images to generate point clouds.

## Requirements

- ROS (Robot Operating System) [version]
- [Additional dependencies]

## Launch Files

### 1. `drone_simulation.launch`

This file sets up the drone simulation environment. It includes nodes for:

- Unity ROS integration.
- State estimate corruption (if enabled).
- TF static transforms for sensors and cameras.
- Command and state publishing.

#### Arguments

- `corrupt_state_estimate`: Enable state estimate corruption.
- `master`: Primary host for Unity integration.
- `servant`: Secondary host for command publishing.
- `name`: Namespace for the drone.
- `state_receive_port`: Port for state estimation.
- `command_send_port`: Port for command publishing.

### 2. `map_server.launch`

This file sets up the Octomap server to create 3D maps from point cloud data. It includes parameters for:

- Map resolution.
- Filtering ground points.
- Color map application.
- Point cloud and sensor configurations.

#### Arguments

- `cloud_in`: The input point cloud topic.
- `resolution`: The resolution of the Octomap.
- `frame_id`: The frame ID for the map.
- `height_map`: Whether to generate a height map.
- `colored_map`: Whether to apply a color map.
- `latch`: Whether to latch the map.
- `filter_ground`: Whether to filter ground points.
- `ground_filter_distance`: Distance for ground filtering.
- `ground_filter_plane_distance`: Plane distance for ground filtering.
- `occupancy_max_z`: Maximum Z value for occupancy.
- `occupancy_min_z`: Minimum Z value for occupancy.
- `pointcloud_max_x`: Maximum X value for point cloud.
- `pointcloud_min_x`: Minimum X value for point cloud.
- `pointcloud_max_y`: Maximum Y value for point cloud.
- `pointcloud_min_y`: Minimum Y value for point cloud.
- `pointcloud_max_z`: Maximum Z value for point cloud.
- `pointcloud_min_z`: Minimum Z value for point cloud.
- `sensor_max_range`: Maximum range of the sensor.
- `filter_speckles`: Whether to filter speckles.

### 3. `depth_rgb_point_cloud.launch`

This file processes depth and RGB camera images to generate point clouds. It includes nodes for:

- Registering depth images.
- Generating point clouds from depth and RGB images.

#### Arguments

- `depth_camera_info`: Topic for depth camera info.
- `depth_image_rect`: Topic for rectified depth images.
- `depth_registered_camera_info`: Topic for registered depth camera info.
- `depth_registered_image_rect`: Topic for registered rectified depth images.
- `rgb_camera_info`: Topic for RGB camera info.
- `rgb_image_rect`: Topic for rectified RGB images.
- `point_cloud`: Topic for the output point cloud.

## Usage

1. **Launch Drone Simulation:**

   ```bash
   roslaunch [package_name] drone_simulation.launch

2. **Start Octomap Server:**

   ```bash
   roslaunch [package_name] map_server.launch

3. **Process Depth and RGB Images:**

   ```bash
   roslaunch [package_name] depth_rgb_point_cloud.launch

