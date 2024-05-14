#include "voxel_pkg/voxel_down_sampling/voxel_grid_filter.h"
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>
#include <sstream>
#include <chrono>
#include <iostream>
#include <thread>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <sys/stat.h> // POSIX library for file size retrieval
#include <ros/ros.h>

namespace voxel_grid {
    VoxelFilterDown::VoxelFilterDown(ros::NodeHandle &nh) : nh_(nh) {
        if (!readParameters()) {
            ROS_ERROR("Could not launch the node due to parameter issues.");
            ros::requestShutdown();
            return;
        }

        point_cloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(m_publisher_topic_name, 1, true);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr sor_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        processPointCloud(cloud, sor_filtered_cloud, voxel_filtered_cloud);
        outputPerformanceMetrics(); //do not change this functions position. Otherwise it stuck on ros::spin and does not prints anything out. Honestly it took way more time than you imagine for me to solve it.
        visualizePointCloud(cloud, sor_filtered_cloud, voxel_filtered_cloud);
    }

    VoxelFilterDown::~VoxelFilterDown() {
        ROS_INFO("Voxel Grid Filter Node has been terminated.");
    }

    bool VoxelFilterDown::readParameters() {
        bool success = true;
        // Get the input parameters
        success &= nh_.getParam("input_pcd_file", m_input_pcd_file);
        success &= nh_.getParam("output_pcd_file", m_output_pcd_file);
        success &= nh_.getParam("m_leaf_size", m_leaf_size);
        success &= nh_.getParam("publish_topic", m_publisher_topic_name);
      
        // Get SOR parameters
        success &= nh_.getParam("use_sor_filter", use_sor_filter_);
        success &= nh_.getParam("sor_mean_k", sor_mean_k_);
        success &= nh_.getParam("sor_std_dev_mul_thresh", sor_std_dev_mul_thresh_);

        return success;
    }

    void VoxelFilterDown::savePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::string& filename, bool binary) {
        auto start = std::chrono::high_resolution_clock::now();
        pcl::PCDWriter writer;
        writer.write<pcl::PointXYZ>(filename, *cloud, binary);
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = end - start;
        std::cout << " " << std::endl;
        std::cout << "-------------------------------------------------------------------" << std::endl;
        std::cout << "Saving " << filename << " (" << (binary ? "binary" : "ascii") << ") took " << duration.count() << " seconds." << std::endl;
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

        // Save the final voxel filtered cloud in binary and ASCII formats
        savePointCloud(voxel_filtered_cloud, "/home/ibrahim/voxel_ws/src/voxel_pkg/data/filtered_binary.pcd", true);
        savePointCloud(voxel_filtered_cloud, "/home/ibrahim/voxel_ws/src/voxel_pkg/data/filtered_ascii.pcd", false);

        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        processing_time_ms_ = duration.count();

        ROS_INFO("Complete processing took %ld milliseconds.", duration.count());
    }

    void VoxelFilterDown::applySORFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
        if (!use_sor_filter_) return; // Skip if SOR filter is not enabled

        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK(sor_mean_k_);
        sor.setStddevMulThresh(sor_std_dev_mul_thresh_);
        sor.filter(*cloud);
    }

    void VoxelFilterDown::visualizePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
                                               pcl::PointCloud<pcl::PointXYZ>::Ptr& sor_filtered_cloud,
                                               pcl::PointCloud<pcl::PointXYZ>::Ptr& voxel_filtered_cloud) {
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

        // Main visualization loop
        while (!viewer_original.wasStopped() || !viewer_sor.wasStopped() || !viewer_voxel.wasStopped()) {
            viewer_original.spinOnce(100);
            viewer_sor.spinOnce(100);
            viewer_voxel.spinOnce(100);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    void VoxelFilterDown::outputPerformanceMetrics() {

        std::cout<<""<<std::endl;
        //to seperate output performance metrics from the rest of the output
        std::cout << "-------------------------------------------------------------------" << std::endl;

        ROS_INFO("Starting to output performance metrics...");

        pcl::PointCloud<pcl::PointXYZ>::Ptr binary_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr ascii_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        ROS_INFO("Loading binary cloud...");
        // Load the filtered clouds from the saved files
        if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/ibrahim/voxel_ws/src/voxel_pkg/data/filtered_binary.pcd", *binary_cloud) < 0) {
            ROS_ERROR("Failed to load binary file");
            return;
        }
        ROS_INFO("Binary cloud loaded successfully.");

        ROS_INFO("Loading ASCII cloud...");
        if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/ibrahim/voxel_ws/src/voxel_pkg/data/filtered_ascii.pcd", *ascii_cloud) < 0) {
            ROS_ERROR("Failed to load ASCII file");
            return;
        }
        ROS_INFO("ASCII cloud loaded successfully.");

        std::cout<<""<<std::endl;
        std::cout << "PERFORMANCE METRICS:" << std::endl;
        std::cout << "Binary Cloud Point Count: " << binary_cloud->points.size() << std::endl;
        std::cout << "ASCII Cloud Point Count: " << ascii_cloud->points.size() << std::endl;

        // Function to get the file size
        auto getFileSize = [](const std::string& filePath) -> size_t {
            struct stat stat_buf;
            int rc = stat(filePath.c_str(), &stat_buf);
            return rc == 0 ? stat_buf.st_size : -1;
        };

        // Get file sizes for the saved clouds
        size_t binary_file_size = getFileSize("/home/ibrahim/voxel_ws/src/voxel_pkg/data/filtered_binary.pcd");
        size_t ascii_file_size = getFileSize("/home/ibrahim/voxel_ws/src/voxel_pkg/data/filtered_ascii.pcd");

        std::cout << "Binary File Size: " << binary_file_size << " bytes" << std::endl;
        std::cout << "ASCII File Size: " << ascii_file_size << " bytes" << std::endl;

        // Calculate the reduction in points
        double reduction_percentage = 100.0 * (1.0 - static_cast<double>(ascii_cloud->points.size()) / binary_cloud->points.size());
        std::cout << "Reduction in points: " << reduction_percentage << "%" << std::endl;
        
        // Optionally include timing data if captured
        std::cout << "Processing Time (ms): " << processing_time_ms_ << std::endl;

        // Any other metrics...
        auto total_points = binary_cloud->points.size();
        double mean_x = 0, mean_y = 0, mean_z = 0;
        for (const auto& point : binary_cloud->points) {
            mean_x += point.x;
            mean_y += point.y;
            mean_z += point.z;
        }
        mean_x /= total_points;
        mean_y /= total_points;
        mean_z /= total_points;

        std::cout << "Mean X: " << mean_x << std::endl;
        std::cout << "Mean Y: " << mean_y << std::endl;
        std::cout << "Mean Z: " << mean_z << std::endl;

        ROS_INFO("Finished outputting performance metrics.");

        std::cout << "-------------------------------------------------------------------" << std::endl;
    }
}
