#include <ros/ros.h>
#include <ros/console.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

ros::Publisher pub;
geometry_msgs::TransformStamped transformStamped;

void cloud_callback (const PointCloud::ConstPtr &input)
{
    // Create output cloud
    PointCloud::Ptr cloud_transformed (new PointCloud);

    // This takes pcl::PointCloud<PointT> and outputs pcl::PointCloud<PointT> using a geometry_msgs::Transform
    pcl_ros::transformPointCloud(*input, *cloud_transformed, transformStamped.transform);

    // Set output frame id
    cloud_transformed->header.frame_id = "base_footprint";

    // Publish
    pub.publish(cloud_transformed);
}

int main (int argc, char** argv){

    // Initialize ROS
    ROS_INFO("Node started");

    ros::init (argc, argv, "transform_point_cloud");
    ros::NodeHandle nh;

    // Create a subscriber for the input point cloud
    // When subscribing to a pcl::PointCloud<T> topic 
    // with a sensor_msgs::PointCloud2 subscriber or viceversa, 
    // the conversion (deserialization) between the two types sensor_msgs::PointCloud2 
    // and pcl::PointCloud<T> is done on the fly by the subscriber.  
    ros::Subscriber sub = nh.subscribe<PointCloud>("/zedA/cloudXYZ", 5, cloud_callback);

    // Create a templated publisher - The publisher takes care of the conversion (serialization) between sensor_msgs::PointCloud2 and pcl::PointCloud<T> where needed.
    pub = nh.advertise<PointCloud>("/transformed_cloud", 5);

    // tf2 version
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    while (nh.ok()){

        try {
            // target frame, source frame, with latest available transform
            transformStamped = tfBuffer.lookupTransform("base_footprint", "zedA_left_camera_optical_frame", ros::Time(0));
        } catch (tf2::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
        }

        ros::spinOnce();
    }

    return 0;

}