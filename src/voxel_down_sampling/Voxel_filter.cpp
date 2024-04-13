#include "voxel_pkg/voxel_down_sampling/voxel_grid_filter.h"
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>
#include <sstream>


namespace voxel_grid {
    VoxelFilterDown::VoxelFilterDown(ros::NodeHandle &nh) : nh_(nh) {
        if (!readParameters()) {
          ROS_ERROR("Could not launch the node due to parameter issues.");
          ros::requestShutdown();
          return;
        }

        point_cloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(m_publisher_topic_name, 1, true);

        ROS_INFO("**********************************************************************");
        ROS_INFO("Voxel Grid Filter Node is started for down sampling the point cloud.");
        ROS_INFO("Parameters: Input PCD File: %s, Output PCD File: %s, Leaf Size: %f, Publish Topic: %s",
                 m_input_pcd_file.c_str(), m_output_pcd_file.c_str(), m_leaf_size, m_publisher_topic_name.c_str());
        ROS_INFO("**********************************************************************"
        
        );

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr sor_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        processPointCloud(cloud, sor_filtered_cloud, voxel_filtered_cloud);
        visualizePointCloud(cloud, sor_filtered_cloud, voxel_filtered_cloud);
    }
    VoxelFilterDown::~VoxelFilterDown() {
        ROS_INFO("Voxel Grid Filter Node has been terminated.");
    }

    bool VoxelFilterDown::readParameters() {
      bool success = true;
      success &= nh_.getParam("input_pcd_file", m_input_pcd_file);
      success &= nh_.getParam("output_pcd_file", m_output_pcd_file);
      success &= nh_.getParam("m_leaf_size", m_leaf_size);
      success &= nh_.getParam("publish_topic", m_publisher_topic_name);
      
      //to get SOR parameters
      success &= nh_.getParam("use_sor_filter", use_sor_filter_);
      success &= nh_.getParam("sor_mean_k", sor_mean_k_);
      success &= nh_.getParam("sor_std_dev_mul_thresh", sor_std_dev_mul_thresh_);
      
      return success;
    }

    void VoxelFilterDown::processPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
                                            pcl::PointCloud<pcl::PointXYZ>::Ptr& sor_filtered_cloud, 
                                            pcl::PointCloud<pcl::PointXYZ>::Ptr& voxel_filtered_cloud) {
        auto start = std::chrono::high_resolution_clock::now();

        // Load the point cloud from file
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(m_input_pcd_file, *cloud) < 0) {
            ROS_ERROR("Failed to load file %s", m_input_pcd_file.c_str());
            return;
        }

        // Apply SOR filter
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK(sor_mean_k_);
        sor.setStddevMulThresh(sor_std_dev_mul_thresh_);
        sor.filter(*sor_filtered_cloud);

        // Apply Voxel Grid filter
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(sor_filtered_cloud);
        vg.setLeafSize(m_leaf_size, m_leaf_size, m_leaf_size);
        vg.filter(*voxel_filtered_cloud);

        pcl::io::savePCDFileBinary(m_output_pcd_file, *voxel_filtered_cloud); // Save the final voxel filtered cloud

        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        ROS_INFO("Complete processing took %ld milliseconds.", duration.count());

        // Visualize the point clouds
        visualizePointCloud(cloud, sor_filtered_cloud, voxel_filtered_cloud);
    }



    void VoxelFilterDown::applySORFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) 
    {
        if (!use_sor_filter_) return; // Skip if SOR filter is not enabled

        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK(sor_mean_k_);
        sor.setStddevMulThresh(sor_std_dev_mul_thresh_);
        sor.filter(*cloud);
    }


    void VoxelFilterDown::visualizePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
                                               pcl::PointCloud<pcl::PointXYZ>::Ptr& sor_filtered_cloud,
                                               pcl::PointCloud<pcl::PointXYZ>::Ptr& voxel_filtered_cloud) 
    {
        // Viewer for the original input cloud
        pcl::visualization::PCLVisualizer viewer_original("Original Input Cloud");
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> input_color_handler(cloud, 255, 255, 255); // White
        viewer_original.addPointCloud(cloud, input_color_handler, "original cloud");
        viewer_original.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "original cloud");
        viewer_original.addCoordinateSystem(1.0);
        viewer_original.setBackgroundColor(0, 0, 0); // Black background for contrast
        viewer_original.initCameraParameters();

        // Viewer for the cloud after SOR
        pcl::visualization::PCLVisualizer viewer_sor("Cloud After SOR Filter");
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sor_color_handler(sor_filtered_cloud, 255, 255, 255); // White
        viewer_sor.addPointCloud(sor_filtered_cloud, sor_color_handler, "sor filtered cloud");
        viewer_sor.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sor filtered cloud");
        viewer_sor.addCoordinateSystem(1.0);
        viewer_sor.setBackgroundColor(0, 0, 0);
        viewer_sor.initCameraParameters();

        // Viewer for the cloud after Voxel Grid
        pcl::visualization::PCLVisualizer viewer_voxel("Cloud After Voxel Grid Filter");
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> voxel_color_handler(voxel_filtered_cloud, 255, 255, 255); // White
        viewer_voxel.addPointCloud(voxel_filtered_cloud, voxel_color_handler, "voxel filtered cloud");
        viewer_voxel.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "voxel filtered cloud");
        viewer_voxel.addCoordinateSystem(1.0);
        viewer_voxel.setBackgroundColor(0, 0, 0);
        viewer_voxel.initCameraParameters();

        // Main visualization loops
        while (!viewer_original.wasStopped() || !viewer_sor.wasStopped() || !viewer_voxel.wasStopped()) {
            viewer_original.spinOnce(100);
            viewer_sor.spinOnce(100);
            viewer_voxel.spinOnce(100);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

}
