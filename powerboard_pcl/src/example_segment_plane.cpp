#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>

#include <pcl/filters/voxel_grid.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/extract_indices.h>


ros::Publisher pub;

//Segment the plane of the point cloud. Segment returned through cloud_out arg.
//Args: 
//  PCLPointCloud2 &cloud_in : input point cloud
//  PCLPointCloud2 &cloud_seg : segmented point cloud, passed by reference

void cloud_segment_plane (pcl::PCLPointCloud2 &cloud_in, pcl::PCLPointCloud2 &cloud_seg)
{
  // Declare variables
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_T(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_seg_T(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::fromROSMsg (*cloud_filtered, *cloud_filtered_templ);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

  //convert PCLPointCloud2 to PCLPointCloud<T>
  pcl::fromPCLPointCloud2(cloud_in,*cloud_in_T); 

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;

  // Optional
  seg.setOptimizeCoefficients (true);

  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.01);


   // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  int i = 0, nr_points = (int) cloud_in_T->points.size ();
  // While 30% of the original cloud is still there
  // Segment the largest planar component from the remaining cloud
  // while (cloud_in_T->points.size () > 0.3 * nr_points)
  // {
    seg.setInputCloud (cloud_in_T);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      ROS_INFO("%s", "Could not estimate a planar model for the given dataset.");
      //std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      // break;
    }

    // Extract the inliers
    extract.setInputCloud (cloud_in_T);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_seg_T);

    //convert PCLPointCloud<T> to PCLPointCloud2
    pcl::toPCLPointCloud2(*cloud_seg_T, cloud_seg); 
}


//Downsample point cloud. Value returned through cloud_filtered arg.
//Args: 
//  PCLPointCloud2ContPtr cloudPtr : pointer to input cloud
//  PCLPointCloud2 &cloud_filtered : downsampled point cloud, passed by reference
//  float leaf_size : size of leaf for downsampling
void cloud_downsample (pcl::PCLPointCloud2ConstPtr cloudPtr, pcl::PCLPointCloud2 &cloud_filtered, float leaf_size)
{

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (leaf_size, leaf_size, leaf_size);
  sor.filter (cloud_filtered);
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{

  //Container for filtered data
  pcl::PCLPointCloud2* cloud_filtered = new pcl::PCLPointCloud2;

  //Container for segmented data
  pcl::PCLPointCloud2* cloud_seg = new pcl::PCLPointCloud2;

  // Container for original data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);  //pointer to input PCLPointCloud2 object
  
  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  //Downsample
  cloud_downsample(cloudPtr, *cloud_filtered, 0.05f);
  //Segment
  cloud_segment_plane(*cloud_filtered, *cloud_seg);
  
  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(*cloud_seg, output);

  // Publish the data
  pub.publish (output);
  ROS_INFO("%s", "Message Received");

}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_seg");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}