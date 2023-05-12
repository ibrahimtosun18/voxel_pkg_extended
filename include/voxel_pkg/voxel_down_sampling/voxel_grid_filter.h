#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <chrono>
#include <thread>

namespace voxel_grid
{
    class VoxelFilterDown
    {

    public:

        //constructor
        VoxelFilterDown(ros::NodeHandle &nh);

        //destructor
        ~VoxelFilterDown();

        // This function is used to process the point cloud. 
        void processPointCloud();

    private:

        //Read parameters function
        bool readParameters();
        
        ros::Publisher point_cloud_publisher;

        ros::NodeHandle nh_;

        std::string m_input_pcd_file;
        std::string m_output_pcd_file;
        
        double m_leaf_size;
    };

}