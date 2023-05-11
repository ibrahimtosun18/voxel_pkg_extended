#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <chrono>
#include <thread>

namespace mls_upsampling
{
    class MLSUpsampler
    {
    public:
        // Constructor
        MLSUpsampler(ros::NodeHandle &nh);

        // Destructor
        ~MLSUpsampler();
        
        // This function is used to process the point cloud. It upsamples it and visualize it then publish it
        void processPointCloud();

    private:
        
        ros::NodeHandle nh_;

        ros::Publisher point_publisher;

        std::string m_input_pcd_file;
        std::string m_output_pcd_file;
        
        double m_search_radius;
        // double m_upsampling_radius;
        // double m_step_size;


    };
}
