#include "voxel_pkg/voxel_down_sampling/voxel_grid_filter.h"

namespace voxel_grid
{
    VoxelFilterDown::VoxelFilterDown(ros::NodeHandle &nh) : nh_(nh)
    {
        // This loop is used to check if the parameters are loaded from the launch file or not. If not, the node will be shut down.
        if (!readParameters())
        {
          ROS_ERROR("Could not launch the node!.");
          ros::requestShutdown();
        }

        point_cloud_publisher = nh_.advertise<sensor_msgs::PointCloud2>(m_publisher_topic_name, 1, true);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        processPointCloud(cloud, filtered_cloud); // Call the processPointCloud() function
        visualizePointCloud(cloud, filtered_cloud); // Call the visualizePointCloud() function
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
        std::cout << "  PUBLISH TOPIC: " << m_publisher_topic_name << std::endl;
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

      if (!nh_.getParam("publish_topic", m_publisher_topic_name))
      {
          ROS_ERROR_STREAM("Failed to get publish topic.");
          return false;
      }

      return success;
    }

    // This function loads the input PCD file,
    // applies the voxel grid filter, and saves the filtered PCD file as binary
    void VoxelFilterDown::processPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_cloud)
    {
        // Load the input PCD file
        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);  // remove this line

        // check if the input file is empty
        if (pcl::io::loadPCDFile(m_input_pcd_file, *cloud) < 0)
        {
            PCL_ERROR("Failed to load file %s\n", m_input_pcd_file.c_str());
            return;
        }

        // Apply the voxel grid filter
        // pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);  // remove this line
        pcl::VoxelGrid<pcl::PointXYZ> vg;

        // Set the leaf size
        vg.setInputCloud(cloud);
        vg.setLeafSize(m_leaf_size, m_leaf_size, m_leaf_size);

        vg.filter(*filtered_cloud);

        // Save the filtered PCD file as binary
        pcl::io::savePCDFileBinary(m_output_pcd_file, *filtered_cloud);
    }


    void VoxelFilterDown::visualizePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_cloud)
    {
        //visualize input point cloud
        pcl::visualization::PCLVisualizer::Ptr input_viewer(new pcl::visualization::PCLVisualizer("Input Cloud Viewer"));

        input_viewer->addPointCloud<pcl::PointXYZ>(cloud, "input cloud");
        input_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "input cloud");
        input_viewer->addCoordinateSystem(m_coordinate_system_size); //10.0
        input_viewer->setBackgroundColor(m_background_color_b, m_background_color_g, m_background_color_r);//0.0, 0.0, 0.0
        input_viewer->initCameraParameters();
        
        // Visualize the filtered point cloud
        pcl::visualization::PCLVisualizer::Ptr filtered_viewer(new pcl::visualization::PCLVisualizer("Filtered Cloud Viewer"));

        filtered_viewer->addPointCloud<pcl::PointXYZ>(filtered_cloud, "filtered cloud");
        filtered_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "filtered cloud");
        filtered_viewer->addCoordinateSystem(m_coordinate_system_size);//10.0
        filtered_viewer->setBackgroundColor(m_background_color_b, m_background_color_g, m_background_color_r);//0.0, 0.0, 0.0
        filtered_viewer->initCameraParameters();

        // Convert the pcl::PointCloud type to sensor_msgs::PointCloud2 type
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*filtered_cloud, output);
        output.header.frame_id = "filtered_cloud";

        while (!filtered_viewer->wasStopped() && !input_viewer->wasStopped())
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
    }

} // namespace voxel_grid
