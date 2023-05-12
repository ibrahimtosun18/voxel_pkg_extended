#include "voxel_pkg/voxel_down_sampling/voxel_grid_filter.h"

namespace voxel_grid
{
    VoxelFilterDown::VoxelFilterDown(ros::NodeHandle &nh) : nh_(nh)
    {

        point_cloud_publisher = nh_.advertise<sensor_msgs::PointCloud2>("voxel_downsampled_point_cloud", 1, true);

        // This loop is used to check if the parameters are loaded from the launch file or not. If not, the node will be shut down.
        if (!readParameters())
        {
          ROS_ERROR("Could not launch the node!.");
          ros::requestShutdown();
        }

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

    // Destructor
    VoxelFilterDown::~VoxelFilterDown()
    {
    }

    //This function reads the parameters from the launch file
    bool VoxelFilterDown::readParameters()
    {
      bool success = true;

    if (!nh_.getParam("input_pcd_file", m_input_pcd_file))
    {
        ROS_ERROR_STREAM("Failed to load input pcd file.");
        return false;
    }

    if (!nh_.getParam("output_pcd_file", m_output_pcd_file))
    {
        ROS_ERROR_STREAM("Failed to load output pcd file.");
        return false;
    }

    if (!nh_.getParam("m_leaf_size", m_leaf_size))
    {
        ROS_ERROR_STREAM("Failed to get leaf size.");
        return false;
    }

        return success;
    }

    // This function loads the input PCD file, applies the voxel grid filter, and saves the filtered PCD file as binary

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
            point_cloud_publisher.publish(output);
            ros::spinOnce();
        }
    } // processPointCloud

} // namespace voxel_grid

