#include "voxel_pkg/mls_upsampler/mls_upsampler.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mls_upsampling_node");
    ros::NodeHandle nh("~");

    mls_upsampling::MLSUpsampler mu(nh);
    mu.processPointCloud(); // Call the processPointCloud() function

    return 0;
}
