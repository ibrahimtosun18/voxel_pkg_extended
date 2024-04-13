#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <chrono>
#include <thread>
#include <pcl/filters/statistical_outlier_removal.h>


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
        void processPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
                                                pcl::PointCloud<pcl::PointXYZ>::Ptr& sor_filtered_cloud, 
                                                pcl::PointCloud<pcl::PointXYZ>::Ptr& voxel_filtered_cloud); 

        // This function is used to visualize the point cloud.
        void visualizePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
                                               pcl::PointCloud<pcl::PointXYZ>::Ptr& sor_filtered_cloud,
                                               pcl::PointCloud<pcl::PointXYZ>::Ptr& voxel_filtered_cloud); 

    private:

        //Read parameters function
        bool readParameters();
        
        ros::Publisher point_cloud_publisher_;

        ros::NodeHandle nh_;

        std::string m_input_pcd_file;
        std::string m_output_pcd_file;
        std::string m_publisher_topic_name;

        double m_leaf_size;
        double m_background_color_r;
        double m_background_color_g;
        double m_background_color_b;
        double m_coordinate_system_size;
        double m_point_size;
        double processing_time_ms_;

        // SOR Filter parameters
        bool use_sor_filter_;
        int sor_mean_k_;
        double sor_std_dev_mul_thresh_;
        
        void applySORFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);



        // bool use_sor_filter_; // Whether to use the Statistical Outlier Removal filter
        // int sor_mean_k_; // Number of nearest neighbors to use for mean distance estimation
        // double sor_std_dev_mul_thresh_; // Standard deviation multiplier for determining which points to remove
        

    };

}