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
        PointCloud::Ptr cloud_;

        void cbPoints(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
            pcp_.transformFramePointCloud( target_frame_, cloud_msg, cloud_ );
            pcp_.setPassThroughParameters( "x", 0.4, 8.0 );
            pcp_.passThrough( cloud_, cloud_ );
            pcp_.setPassThroughParameters( "y", -4.0, 4.0 );
            pcp_.passThrough( cloud_, cloud_ );
            pcp_.setPassThroughParameters( "z", 0.05, 1.5 );
            pcp_.passThrough( cloud_, cloud_ );
            pcp_.voxelGrid( cloud_, cloud_ );
            pcp_.radiusOutlierRemoval ( cloud_, cloud_ );
            pcl_conversions::toPCL(cloud_msg->header.stamp, cloud_->header.stamp);
            pub_cloud_.publish(cloud_);
        }

    public:
        PointcloudSsubscriber(): nh_(), pnh_("~") {
            std::string topic_name = pnh_.param<std::string>( "topic_name", "/points2" );
            double voxel_size = pnh_.param<double>( "voxel_size", 0.025 );
            double radius = pnh_.param<double>( "radius", 0.05 );
            int min_pt = pnh_.param<int>( "min_pt", 5 );
            target_frame_ = pnh_.param<std::string>( "target_frame", "base_footprint" );
            pcp_.setVoxelGridParameter( voxel_size );
            pcp_.setRadiusOutlierRemovalParameters ( radius, min_pt, false );
            pub_cloud_ = nh_.advertise<PointCloud>("cloud_noise_removal", 1);
            sub_points_ = nh_.subscribe(topic_name, 5, &PointcloudSsubscriber::cbPoints, this);
            cloud_.reset(new PointCloud());
        }
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "noise_removal_pointcloud_publisher");

    PointcloudSsubscriber pointcloud_subscriber;
    ros::spin();
}
