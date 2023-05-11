#include "voxel_pkg/voxel_down_sampling/voxel_grid_filter.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "voxel_filter_node");
    ros::NodeHandle nh("~");

    voxel_grid::VoxelFilterDown vf(nh);
    vf.processPointCloud(); // Call the processPointCloud() function

    return 0;
}
