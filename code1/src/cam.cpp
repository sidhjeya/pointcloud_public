#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/PCLPointCloud2.h>

using namespace std;

ros::Publisher pub;
ros::Publisher pubpass;
int i=0;

void passth(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	      //int=PCLPointCloud2
             //int* a = new a
             //a pointer to a variable whose datatype is integer(menaing of the above notion)
             pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
             //cloudPtr used in voxel filter
             pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
             
             pcl::PCLPointCloud2 cloud_filtered;

             // Convert to PCL data type
             pcl_conversions::toPCL(*cloud_msg,*cloud);

                     
             //Pass through filtering happens
              pcl::PassThrough<pcl::PCLPointCloud2> pass;
	      pass.setInputCloud (cloudPtr);
	      pass.setFilterFieldName ("z");
	      pass.setFilterLimits (2,3);
	      pass.filter (cloud_filtered);
		     

             // Convert to ROS data type
             sensor_msgs::PointCloud2 outputpass;
             pcl_conversions::moveFromPCL(cloud_filtered, outputpass);

             // Publish the data
             pubpass.publish(outputpass);
             
  /*           for(auto& point : cloudPtr->point)
 {
    point.x
 }
   */
   
 for (sensor_msgs::PointCloud2ConstIterator<float> it(outputpass,  "x"); it != it.end(); ++it) 
  {     
        cout<<"After pass through filter: "<< i << '\n';
        cout <<" x " << it[0] << ", " <<" y " << it[1] << ", "<< "z " << it[2] << '\n';
        
           i++;
        } 







}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{ 		
	
             //int=PCLPointCloud2
             //int* a = new a
             //a pointer to a variable whose datatype is integer(menaing of the above notion)
             pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
             //cloudPtr used in voxel filter
             pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
               
 
             pcl::PCLPointCloud2 cloud_filtered;

             // Convert to PCL data type
             pcl_conversions::toPCL(*cloud_msg,*cloud);

            // Voxel filtering  happens here
             pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
             sor.setInputCloud (cloudPtr);
             sor.setLeafSize (0.1, 0.1, 0.1);
             sor.filter (cloud_filtered);
                          	     

             // Convert to ROS data type
             sensor_msgs::PointCloud2 output;
             pcl_conversions::moveFromPCL(cloud_filtered, output);

             // Publish the data
             pub.publish (output);
             





}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "neg_obs");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/voxel_filter", 1);
  
  // Create a ROS subscriber for the input point cloud after its is converted to voxel
  ros::Subscriber subpass = nh.subscribe ("/voxel_filter", 1, passth);
  
  // Create a ROS publisher for the output after applying pass filter
  pubpass = nh.advertise<sensor_msgs::PointCloud2> ("/after_pass_filter", 1);

  // Spin
  ros::spin ();
}
