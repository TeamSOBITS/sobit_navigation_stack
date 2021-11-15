#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <cstring>
#include <sobit_navigation/point_cloud_processor.hpp>

class PointcloudSsubscriber {
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        ros::Subscriber sub_points_;
        ros::Publisher pub_cloud_;
        sobit_navigation::PointCloudProcessor pcp_;
        std::string target_frame_;

        void cbPoints(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
            PointCloud::Ptr cloud (new PointCloud());
            pcp_.transformFramePointCloud( "base_footprint", cloud_msg, cloud );
            pcp_.setPassThroughParameters( "x", 0.5, 8.0 );
            pcp_.passThrough( cloud, cloud );
            pcp_.setPassThroughParameters( "y", -4.0, 4.0 );
            pcp_.passThrough( cloud, cloud );
            pcp_.setPassThroughParameters( "z", 0.05, 2.0 );
            pcp_.passThrough( cloud, cloud );
            pcp_.voxelGrid( cloud, cloud );
            pcp_.radiusOutlierRemoval ( cloud, cloud );
            pcl_conversions::toPCL(cloud_msg->header.stamp, cloud->header.stamp);
            //pcl_conversions::toPCL(ros::Time::now(), cloud->header.stamp);
            pub_cloud_.publish(cloud);
            ROS_INFO("cloud points size = %zu\n", cloud->points.size());
        }

    public:
        PointcloudSsubscriber(): nh_(), pnh_("~") {
            std::string topic_name = pnh_.param<std::string>( "topic_name", "/k4a/points2" );
            target_frame_ = pnh_.param<std::string>( "target_frame", "base_footprint" );

            ROS_INFO("topic_name = '%s'", topic_name.c_str());
            pcp_.setVoxelGridParameter( 0.02 );
            pcp_.setRadiusOutlierRemovalParameters ( 0.05, 2, false );

            pub_cloud_ = nh_.advertise<PointCloud>("cloud_noise_removal", 1);
            sub_points_ = nh_.subscribe(topic_name, 5, &PointcloudSsubscriber::cbPoints, this);
        }
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "noise_removal_pointcloud_publisher");

    PointcloudSsubscriber pointcloud_subscriber;
    ros::spin();
}
