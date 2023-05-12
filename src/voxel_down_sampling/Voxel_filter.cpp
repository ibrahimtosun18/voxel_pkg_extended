#include "voxel_pkg/voxel_down_sampling/voxel_grid_filter.h"

namespace voxel_grid
{
    VoxelFilterDown::VoxelFilterDown(ros::NodeHandle &nh) : nh_(nh)
    {
        // Get the parameters from the launch file
        //These lines only used for default values in case of not using the launch file
        nh_.param<std::string>("input_pcd_file", m_input_pcd_file, "data/map.pcd");
        nh_.param<std::string>("output_pcd_file", m_output_pcd_file, "data/new_pcd.pcd");

        //sets the default leaf size of the voxel grid filter but can be changed in the launch file
        nh_.param("leaf_size", m_leaf_size, 0.1);

        // I also added a publisher in case of using the voxel filter as a subscriber from other nodes
        point_publisher = nh_.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1);

        processPointCloud(); // Call the processPointCloud() function
        
        //Giving information about the node
        std::cout << "  =============================================================== " << std::endl;
        std::cout << "  VOXEL FILTER NODE IS STARTED FOR DOWN SAMPLING THE POINT CLOUD" << std::endl;
        std::cout << "  =============================================================== " << std::endl;
        
        std::cout << " " <<std::endl;
        std::cout << "  PARAMETERS:" << std::endl;

        std::cout << "  ************************************************************************" << std::endl;
        std::cout << "  INPUT PCD FILE: " << m_input_pcd_file << std::endl;
        std::cout << "  OUTPUT PCD FILE: " << m_output_pcd_file << std::endl;
        std::cout << "  LEAF SIZE: " << m_leaf_size << std::endl;
        std::cout << "  ************************************************************************" << std::endl;
    }


    // This function loads the input PCD file, applies the voxel grid filter, and saves the filtered PCD file then visualizes the input 
    // and filtered point clouds then publishes the filtered point cloud

    void VoxelFilterDown::processPointCloud()
    {
        // Load the input PCD file
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // check if the input file is empty
        if (pcl::io::loadPCDFile(m_input_pcd_file, *cloud) < 0)
        {
            PCL_ERROR("Failed to load file %s\n", m_input_pcd_file.c_str());
            return;
        }

        // Apply the voxel grid filter
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> vg;

        // Set the leaf size
        vg.setInputCloud(cloud);
        vg.setLeafSize(m_leaf_size, m_leaf_size, m_leaf_size);

        vg.filter(*filtered_cloud);

        // Save the filtered PCD file as binary
        pcl::io::savePCDFileBinary(m_output_pcd_file, *filtered_cloud);

        // Visualize the input and filtered point clouds
        pcl::visualization::PCLVisualizer::Ptr input_viewer(new pcl::visualization::PCLVisualizer("Input Cloud Viewer"));
        pcl::visualization::PCLVisualizer::Ptr filtered_viewer(new pcl::visualization::PCLVisualizer("Filtered Cloud Viewer"));


        // Convert the pcl::PointCloud type to sensor_msgs::PointCloud2 type
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*filtered_cloud, output);
        output.header.frame_id = "filtered_cloud";


        // Set the background color
        input_viewer->setBackgroundColor(0, 0, 0);
        filtered_viewer->setBackgroundColor(0, 0, 0);

        // Add the point clouds to the viewers
        input_viewer->addPointCloud<pcl::PointXYZ>(cloud, "input cloud");
        filtered_viewer->addPointCloud<pcl::PointXYZ>(filtered_cloud, "filtered cloud");

        // Set the size of points
        input_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "input cloud");
        filtered_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "filtered cloud");

        // Set the color of points
        input_viewer->addCoordinateSystem(10.0);
        filtered_viewer->addCoordinateSystem(10.0);

        // Set camera position and orientation
        input_viewer->initCameraParameters();
        filtered_viewer->initCameraParameters();


        while (ros::ok())// this one can be used as well : !input_viewer->wasStopped() && !filtered_viewer->wasStopped()
        {
            // Update the viewers
            input_viewer->spinOnce(100);
            filtered_viewer->spinOnce(100);
            std::this_thread::sleep_for(std::chrono::microseconds(100000));

            // Publish the filtered point cloud
            ros::Time time = ros::Time::now();
            point_publisher.publish(output);
            ros::spinOnce();
        }
        
    } // processPointCloud

} // namespace voxel_grid

