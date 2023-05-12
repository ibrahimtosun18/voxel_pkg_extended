#include "voxel_pkg/mls_upsampler/mls_upsampler.h"

namespace mls_upsampling
{
    // Constructor
    MLSUpsampler::MLSUpsampler(ros::NodeHandle &nh) : nh_(nh)
    {
        
        point_cloud_publisher = nh_.advertise<sensor_msgs::PointCloud2>("upsampled_cloud", 1);

        // This loop is used to check if the parameters are loaded from the launch file or not. If not, the node will be shut down.
        if (!readParameters())
        {
          ROS_ERROR("Could not launch the node!.");
          ros::requestShutdown();
        }

        processPointCloud(); // Call the processPointCloud() function

        //Giving information about the node
        std::cout << "  =============================================================== " << std::endl;
        std::cout << "  MLS UPSAMPLER NODE IS STARTED FOR UP SAMPLING THE POINT CLOUD" << std::endl;
        std::cout << "  =============================================================== " << std::endl;
        
        std::cout << " " <<std::endl;
        std::cout << "  PARAMETERS:" << std::endl;

        std::cout << "  ************************************************************************" << std::endl;
        std::cout << "  INPUT PCD FILE: " << m_input_pcd_file << std::endl;
        std::cout << "  OUTPUT PCD FILE: " << m_output_pcd_file << std::endl;
        std::cout << "  SEARCH RADIUS: " << m_search_radius << std::endl;
        std::cout << "  ************************************************************************" << std::endl;
    }
    
    // Destructor
    MLSUpsampler::~MLSUpsampler()
    {
    }

    //This function reads the parameters from the launch file
    bool MLSUpsampler::readParameters()
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

    if (!nh_.getParam("m_search_radius", m_search_radius))
    {
        ROS_ERROR_STREAM("Failed to load search radius.");
        return false;
    }

    if (!nh_.getParam("m_upsampling_radius", m_upsampling_radius))
    {
        ROS_ERROR_STREAM("Failed to load upsampling radius.");
        return false;
    }
    if (!nh_.getParam("m_step_size", m_step_size))
    {
        ROS_ERROR_STREAM("Failed to load step size.");
        return false;
    }

        return success;
    }


    // This function is used to process the point cloud. 
    void MLSUpsampler::processPointCloud()
    {
        // Load the point cloud from the input file
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // check if the input file is empty
        if (pcl::io::loadPCDFile(m_input_pcd_file, *cloud) < 0)
        {
            PCL_ERROR("Failed to load file %s\n", m_input_pcd_file.c_str());
            return;
        }

        // Create the output point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr upsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        // Create the search tree
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

        // Create the object for the upsampling
        pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
        mls.setComputeNormals(true);
        mls.setInputCloud(cloud);
        mls.setPolynomialOrder(2);
        mls.setSearchMethod(tree);
        mls.setSearchRadius(m_search_radius);

        // Upsampling the point cloud
        mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
        mls.setUpsamplingRadius(m_upsampling_radius);
        mls.setUpsamplingStepSize(m_step_size);

    
        // Reconstruct
        mls.process(*upsampled_cloud);

        // Save the output point cloud as binary
        pcl::io::savePCDFileBinary(m_output_pcd_file, *upsampled_cloud);

        // Create the viewers
        pcl::visualization::PCLVisualizer::Ptr input_viewer(new pcl::visualization::PCLVisualizer("Input Cloud Viewer"));
        pcl::visualization::PCLVisualizer::Ptr upsampled_viewer(new pcl::visualization::PCLVisualizer("Upsampled Cloud Viewer"));

        // Convert the pcl::PointCloud type to sensor_msgs::PointCloud2 type
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*upsampled_cloud, output);
        output.header.frame_id = "upsampled_cloud";

        // Set the background color
        input_viewer->setBackgroundColor(0, 0, 0);
        upsampled_viewer->setBackgroundColor(0, 0, 0);

        // Add the point clouds to the viewers
        input_viewer->addPointCloud<pcl::PointXYZ>(cloud, "input cloud");
        upsampled_viewer->addPointCloud<pcl::PointXYZ>(upsampled_cloud, "upsampled cloud");

        // Set the size of points
        input_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "input cloud");
        upsampled_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "upsampled cloud");

        // Set the color of points
        input_viewer->addCoordinateSystem(10.0);
        upsampled_viewer->addCoordinateSystem(10.0);

        // Set camera position and orientation
        input_viewer->initCameraParameters();
        upsampled_viewer->initCameraParameters();

        while (ros::ok())
        {
            // Update the viewers
            input_viewer->spinOnce(100);
            upsampled_viewer->spinOnce(100);
            std::this_thread::sleep_for(std::chrono::microseconds(100000));

            // Publish the upsampled point cloud
            ros::Time time = ros::Time::now();
            point_cloud_publisher.publish(output);
            ros::spinOnce();
        }

    } // processPointCloud

} // namespace mls_upsampling