# Voxel Grid Filter and Moving Least Squares technique

The Voxel Grid Filter is a ROS package that performs downsampling of point clouds using the Voxel Grid algorithm. This filter can be used to reduce the number of points in a point cloud (downsampling) based on a specified voxel size. 

The Moving Least Squares (MLS) technique, on the other hand, is used for upsampling PCD files. This technique can be used to increase point numbers on output pcd file (upsampling) based on a specified parameters. 

Please note that; Using MLS as upsampling requires a lot of processing power, so in order to avoid lagging or freezing on your system, set parameter values compatible according to your project and system capacity. Getting output may take some time.

## Package Overview

The Voxel Grid Filter consists of the following files:

- `voxel_filter_node.cpp`: The main executable file that initializes the ROS node and creates an instance of the VoxelFilter class.
- `voxel_grid_filter.h`: The header file for the VoxelFilter class, which contains the class definition and function prototypes.
- `voxel_filter.cpp`: The implementation file for the VoxelFilter class, which contains the implementation of the class functions.

The Moving Least Squares technique in this package consists of the following files:

- `mls_upsampler_node.cpp`: The main executable file that initializes the ROS node and creates an instance of the MLSUpsampler class.
- `mls_upsampler.h`: The header file for the MLSUpsampler class, which contains the class definition and function prototypes.
- `mls_upsampler.cpp`: The implementation file for the MLSUpsampler class, which contains the implementation of the class functions.

## Installation

To use the Voxel Grid Filter package, follow these steps:

1. Clone the repository into your own workspace:
```
cd ~/your_workspace/src
git clone git@github.com:ibrahimtosun18/voxel_pkg.git
```

2. Build the package:
```
cd ~/your_workspace
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
![termialBinaryAscii3](https://github.com/ibrahimtosun18/voxel_pkg_extended/assets/95874081/9249382b-ac26-427a-a774-715489d7df0c)

2. MLS technique:

 ![terminalUpsampled](https://github.com/ibrahimtosun18/voxel_pkg_extended/assets/95874081/a763053e-832e-42e5-a955-7101368a50d2)


You can use the `rostopic echo` command to monitor the filtered point cloud output for both Voxel Filter and MLS:

1. For Voxel Filter:
```
rostopic echo /voxel_down_node/filtered_cloud
```

2. For MLS:
```
rostopic echo /mls_upsampler_node/upsampled_cloud
```

This will display the filtered point cloud data on the console.

## Visualize the point clouds:

The voxel filter node also provides visualization of the input and filtered point clouds using the PCLVisualizer. The visualizer windows will appear when the node is running.

1. How it should look like (VOXEL)

![down_sample1](https://github.com/ibrahimtosun18/voxel_pkg/assets/95874081/e8f6a8e4-a5ce-4458-a295-b1b4c672b9a7)

3. How it should look like (MLS)

![upsampled1](https://github.com/ibrahimtosun18/voxel_pkg/assets/95874081/11fcd2b6-9ecf-4e49-bdcf-0efe6425d03d)

## Statistical Outlier Removal
The code also includes the Statistical Outlier Removal (SOR) method, which is utilized to eliminate redundant points from the PCD file, thereby reducing its overall size. This process enhances performance by ensuring that only significant points are retained. Initially, the original PCD file is filtered using the SOR method to remove any outliers and redundant points, effectively decreasing its size. Subsequently, the output from the SOR filtering is processed through the voxel grid filter to achieve an optimized PCD file.

## How SOR method looks on visualizer
Here you can clearly see the removal of outlier points from original PCD file
![terminalSOR](https://github.com/ibrahimtosun18/voxel_pkg_extended/assets/95874081/0c7f7919-0793-4f9f-8cf7-511f2c7604d0)


## Launch File Parameters

1. The voxel filter node can be customized using the following parameters in the launch file (`start_voxel_grid_filter.launch`):

- `input_pcd_file` (default: "data/map.pcd"): The path to the input PCD file.
- `output_pcd_file` (default: "data/new_pcd.pcd"): The path to save the filtered PCD file.
- `m_leaf_size` (default: 0.1): The voxel size for downsampling. Adjust this parameter to control the density of the output point cloud.
- `use_sor_filter` (default: true): Whether to use Statistical Outlier Removal (SOR) filter.
- `sor_mean_k` (default: 50): The number of nearest neighbors to use for mean distance estimation in SOR.
- `sor_std_dev_mul_thresh` (default: 1.0): The standard deviation multiplier threshold for SOR.

2. The mls_upsampler_node can be customized using the following parameters in the launch file (`start_upsampler.launch`):

- `input_pcd_file` (default: "data/map.pcd"): The path to the input PCD file.
- `output_pcd_file` (default: "data/new_pcd.pcd"): The path to save the filtered PCD file.
- `leaf_size` (default: 0.1): The voxel size for downsampling or upsampling. Adjust this parameter to control the density of the output point cloud.

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

## Contact

For any issues or inquiries, please contact <ibrahim.tsn18@gmail.com>

## License

This project is licensed under the [MIT License](LICENSE).













