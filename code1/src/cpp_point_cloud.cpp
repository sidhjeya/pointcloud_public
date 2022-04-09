         #include <ros/ros.h>
         #include <pcl_conversions/pcl_conversions.h>
         #include <pcl/point_cloud.h>
         #include <pcl/point_types.h>
         #include <pcl/filters/voxel_grid.h>
         #include <sensor_msgs/PointCloud2.h>
         #include <pcl/filters/passthrough.h>
        #include <pcl/filters/extract_indices.h>
      

        using namespace std;
         ros::Publisher pub;

         void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
         {
             // Container for original & filtered data
             pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
             pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
             pcl::PCLPointCloud2 cloud_filtered;
             
             // Convert to PCL data type
             pcl_conversions::toPCL(*cloud_msg, *cloud);
             

             // Perform the actual filtering
            /* 
             pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
             sor.setInputCloud (cloudPtr);
             sor.setLeafSize (0.1, 0.1, 0.1);
             sor.filter (cloud_filtered);
          pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud;
           for (int i=0 ;i<pointCloud->points.size();i++)
            {
              pointCloud->points[i].x;
              pointCloud->points[i].y;
              pointCloud->points[i].z;
             }
             */
             
             
             
	  pcl::PassThrough<pcl::PCLPointCloud2> pass;
	
	  pass.setInputCloud (cloudPtr);
	  pass.setFilterFieldName ("z");
	  pass.setFilterLimits (2, 3);
	  //pass.setFilterLimitsNegative (true);
	  pass.filter (cloud_filtered);

             // Convert to ROS data type
             sensor_msgs::PointCloud2 output;
            
             pcl_conversions::moveFromPCL(cloud_filtered, output);

             // Publish the data
            
             pub.publish (output);
         }

         int main (int argc, char** argv)
         {
             // Initialize the ROS Node "roscpp_pcl_example"
             ros::init (argc, argv, "roscpp_pcl_example");
             ros::NodeHandle nh;

             // Print "Hello" message with node name to the terminal and ROS log file
             ROS_INFO_STREAM("Hello from ROS Node: " << ros::this_node::getName());

  // Create a ROS Subscriber to IMAGE_TOPIC with a queue_size of 1 and a callback function to cloud_cb
             ros::Subscriber sub = nh.subscribe("/camera/depth/points", 1, cloud_cb);

             // Create a ROS publisher to PUBLISH_TOPIC with a queue_size of 1
             pub = nh.advertise<sensor_msgs::PointCloud2>("/pcl/points", 1);

             // Spin
             ros::spin();

             // Success
             return 0;
         }
