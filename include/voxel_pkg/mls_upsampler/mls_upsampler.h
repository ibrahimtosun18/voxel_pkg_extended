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
        
        // This function is used to process the point cloud. 
        void processPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& upsampled_cloud);
        void visualizePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& upsampled_cloud);

    private:
        
        //Read parameters function
        bool readParameters();

        ros::NodeHandle nh_;

        ros::Publisher point_cloud_publisher;

        std::string m_input_pcd_file;
        std::string m_output_pcd_file;
        
        double m_search_radius;
        double m_upsampling_radius;
        double m_step_size;
        double background_color_r;
        double background_color_g;
        double background_color_b;

        std::string m_input_cloud_viewer;
        std::string m_upsampled_cloud_viewer;
        


    };
}
