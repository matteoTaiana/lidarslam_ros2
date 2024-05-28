// This file can be used to compare results between different GICP and NDT implementations (PCL VS OMPPCL).
#include <rclcpp/rclcpp.hpp>

#include <pclomp/ndt_omp.h>
#include <pclomp/ndt_omp_impl.hpp>
#include <pclomp/voxel_grid_covariance_omp.h>
#include <pclomp/voxel_grid_covariance_omp_impl.hpp>
#include <pclomp/gicp_omp.h>
#include <pclomp/gicp_omp_impl.hpp>

#include <nlohmann/json.hpp>
#include <chrono>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <cstdio>
#include <stdio.h>
#include <iostream>
#include <string>
#include <fstream>
#include <cstdint>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
// #include <pcl/visualization/cloud_viewer.h>
// #include <pcl/registration/icp.h>
// #include <pcl/registration/gicp.h>
// #include <pcl/segmentation/extract_clusters.h>
// #include <pcl/filters/extract_indices.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/kdtree/kdtree.h>
// #include <pcl/sample_consensus/method_types.h>
// #include <pcl/sample_consensus/model_types.h>
// #include <pcl/segmentation/sac_segmentation.h>
// #include <pcl/segmentation/extract_clusters.h>
// #include <pcl/filters/statistical_outlier_removal.h>
// #include <pcl/features/normal_3d_omp.h>
#include <Eigen/Dense>
using namespace std::chrono_literals;
using namespace std;

Eigen::Matrix3f camera_matrix;


int main(int argc, char * argv[])
{   
    pcl::console::setVerbosityLevel(pcl::console::L_VERBOSE);

    // GICP parameters.
    pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;   
    gicp.setMaxCorrespondenceDistance(0.20);
    gicp.setMaximumIterations(100);
    gicp.setTransformationEpsilon(1e-8);

    float vg_size_for_map_ = 0.1;



    // Read PCDs.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);  // Input.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>); // Intput.
    pcl::PointCloud<pcl::PointXYZ> input_cloud_after_registration;                     // Output.

    char filename0[200];
    char filename1[200];
    
    sprintf(filename0, "/home/lab0/ws_lab0/data/lidar_slam/2024_04_24/pcds_to_be_aligned/%s", argv[1]);
    sprintf(filename1, "/home/lab0/ws_lab0/data/lidar_slam/2024_04_24/pcds_to_be_aligned/%s", argv[2]);
    cout << filename0 << endl;
    cout << filename1 << endl;
       
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (filename0, *cloud_in) == -1) // Load the PCD.
    {
        PCL_ERROR ("Couldn't read first pcd.\n");
        return (-1);
    }
    std::cout << "Loaded PCD0 with " << cloud_in->width * cloud_in->height << " points." << std::endl;

    if (pcl::io::loadPCDFile<pcl::PointXYZ> (filename1, *cloud_out) == -1)  // Load the PCD.
    {
        PCL_ERROR ("Couldn't read second pcd.\n");
        return (-1);
    }
    std::cout << "Loaded PCD1 with " << cloud_in->width * cloud_in->height << " points." << std::endl;

    // Voxelize first PCD.
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid0;
    voxel_grid0.setLeafSize(vg_size_for_map_, vg_size_for_map_, vg_size_for_map_);  // It might be possible to use the same voxel grid for both.
    voxel_grid0.setInputCloud(cloud_in);
    voxel_grid0.filter(*filtered_cloud_in);
    std::cout << "n_points for PCD0 after voxel filtering = " << filtered_cloud_in->width * filtered_cloud_in->height << std::endl;

    // Voxelize second PCD.
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_out(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid1;
    voxel_grid1.setLeafSize(vg_size_for_map_, vg_size_for_map_, vg_size_for_map_);
    voxel_grid1.setInputCloud(cloud_out);
    voxel_grid1.filter(*filtered_cloud_out);
    std::cout << "n_points for PCD1 after voxel filtering = " << filtered_cloud_out->width * filtered_cloud_out->height << std::endl;



    // Perform registration.
    gicp.setInputSource(filtered_cloud_in);  
    gicp.setInputTarget(filtered_cloud_out); 
    gicp.align(input_cloud_after_registration);

    
    // Print registration information.
    cout << "Has converged:" << gicp.hasConverged() << endl;
    cout << "Fitness = " << gicp.getFitnessScore() << endl;
    cout << "Convergence criterion:" << *gicp.getConvergeCriteria() << endl;
    cout << "**********************************" << endl;
    cout << "Estimated transformation: " << endl;
    cout << gicp.getFinalTransformation() << endl;
  


    return 0;
}