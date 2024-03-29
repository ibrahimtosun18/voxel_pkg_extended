# Voxel Grid Filter and Moving Least Squares technique

The Voxel Grid Filter is a ROS package that performs downsampling of point clouds using the Voxel Grid algorithm. This filter can be used to reduce the number of points in a point cloud (downsampling) based on a specified voxel size. 

Moving Least Squares technique on the other hand is used for up sampling pcd files. This technique can be used to increase point numbers on output pcd file (upsampling) based on a specified parameters. 

Please note that; Using MLS as upsampling requires alot of processing power, so in order to avoid lagging or freezing on your system, set parameter values compatible according to your prject and system capacity. Getting output may take some time.

## Package Overview

The Voxel Grid Filter consists of the following files:

- `voxel_filter_node.cpp`: The main executable file that initializes the ROS node and creates an instance of the VoxelFilter class.
- `voxel_grid_filter.h`: The header file for the VoxelFilter class, which contains the class definition and function prototypes.
- `voxel_filter.cpp`: The implementation file for the VoxelFilter class, which contains the implementation of the class functions.

The Moving Least Squares technique in this package consists of the following files:

- `mls_upsampler_node.cpp`: The main executable file that initializes the ROS node and creates an instance of the MLSUpsampler class.
- `mls_upsampler.h`: The header file for the MLSUpsampler class, which contains the class definition and function prototypes.
- `Mls_upsampler.cpp`: The implementation file for the MLSUpsampler class, which contains the implementation of the class functions.


## Installation

To use the Voxel Grid Filter package, follow these steps:

1. Clone the repository into your own workspace:
```
cd ~/your_workspace/src
git clone git@github.com:ibrahimtosun18/voxel_pkg.git
```

2. Build the package:
```
cd ~/your_worksapce
catkin_make
```
## Usage
1. Launch the voxel filter node:
```
roslaunch voxel_pkg start_voxel_grid_filter.launch
```

2. Launch Moving Least Squares (MLS) up sampler:
```
roslaunch voxel_pkg start_upsampler.launch
```



## Monitor the output:
1. Voxel Filter:

![terminal2](https://github.com/ibrahimtosun18/Voxel_Grid_Filter/assets/95874081/fae2ffcc-bb85-4158-ab02-f4f2eea5e193)


2. MLS tqchnique:

![terminal_mls](https://github.com/ibrahimtosun18/voxel_pkg/assets/95874081/9911890a-bcfb-4a8b-8272-c83447b41092)


You can use the `rostopic echo` command to monitor the filtered point cloud output for both Voxel Filter and MLS:

1.For Voxel Filter:
```
rostopic echo /voxel_down_node/filtered_cloud
```
2.For MLS:
```
rostopic echo /mls_upsampler_node/upsampled_cloud 
```
This will display the filtered point cloud data on the console.

## Visualize the point clouds:

The voxel filter node also provides visualization of the input and filtered point clouds using the PCLVisualizer. The visualizer windows will appear when the node is running.

1. How it should look like (VOXEL)

![voxel](https://github.com/ibrahimtosun18/Voxel_Grid_Filter/assets/95874081/a8f4df1b-1804-41a1-a6d5-837944c179ff)

2.Another pcd file example:

![down_sample1](https://github.com/ibrahimtosun18/voxel_pkg/assets/95874081/e8f6a8e4-a5ce-4458-a295-b1b4c672b9a7)



2. How it should look like (MLS)

![upsampled1](https://github.com/ibrahimtosun18/voxel_pkg/assets/95874081/11fcd2b6-9ecf-4e49-bdcf-0efe6425d03d)

## Launch File Parameters

1. The voxel filter node can be customized using the following parameters in the launch file (`start_voxel_grid_filter.launch`):

- `input_pcd_file` (default: "data/map.pcd"): The path to the input PCD file.
- `output_pcd_file` (default: "data/new_pcd.pcd"): The path to save the filtered PCD file.
- `leaf_size` (default: 0.1): The voxel size for downsampling or upsampling. Adjust this parameter to control the density of the output point cloud.

2. The mls_upsampler_node can be customized using the following parameters in the launch file (`start_upsampler.launch`):

## Subscribed Topics

The voxel filter node subscribes to the following topic:

- `/input_cloud`: The input point cloud topic. Publish the point cloud data you want to filter on this topic.

The MLS node subscribes to the following topic:

- `/input_cloud`: The input point cloud topic. Publish the point cloud data you want to filter on this topic.


## Published Topics

The voxel filter node publishes the following topic:

- `/filtered_cloud`: The filtered point cloud topic. This topic contains the filtered point cloud data after applying the voxel grid filter.

The MLS node publishes the following topic:
- `/upsampled_cloud`: The filtered point cloud topic. This topic contains the filtered point cloud data after applying the MLS technique.

## Upsampler parameters:
`input_pcd_file`: The path to the input PCD file to be upsampled.

`output_pcd_file`: The path where the upsampled PCD file should be saved.

`use_sim_time`: Set to true if you want the node to use simulation time (from a ROS bag file or simulator) instead of real time.

`m_upsampling_radius`: The radius of the sphere that will be used for the upsampling. Points will be added within this sphere to create the upsampled point cloud.

`m_step_size`: The step size for the movement of the sphere in the upsampling process.

`m_search_radius`: The radius of the sphere within which to search for the nearest neighbors of a point.

`input_cloud_topic`: The ROS topic on which the input point cloud is published.

`upsampled_cloud_topic`: The ROS topic on which the upsampled point cloud will be published.

`input_cloud_viewer`: The title of the viewer window for the input point cloud.

`upsampled_cloud_viewer`: The title of the viewer window for the upsampled point cloud.

`point_size`: The size of the points in the visualization.

`coordinate_system_size`: The size of the coordinate system in the visualization.

`background_color_r`, `background_color_g`, `background_color_b`: The RGB values (0-255) for the background color of the visualization.

## Downsampler parameters:

`input_pcd_file`: The path to the input PCD file to be downsampled.

`output_pcd_file`: The path where the downsampled PCD file should be saved.

`m_leaf_size`: The voxel grid leaf size. This is the size of the side of the voxels (3D pixels) used in the downsampling process. A smaller leaf size will result in a denser downsampled point cloud, but `will take longer to compute.

`publish_topic`: The ROS topic on which the downsampled point cloud will be published.

`point_size`: The size of the points in the visualization.

`coordinate_system_size`: The size of the coordinate system in the visualization.

`background_color_r`, `background_color_g`, `background_color_b`: The RGB values (0-255) for the background color of the visualization.

## Contact

For any issues or inquiries, please contact <ibrahim.tsn18@gmail.com>
