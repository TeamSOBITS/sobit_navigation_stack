#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <cstring>
// #include <sensor_msgs/PointCloud.h>
// #include <geometry_msgs/Point32.h>
#include <sobit_navigation/point_cloud_processor.hpp>


struct RGB {
    uint r, g, b;
};


class NOISE_COLOR_POINTCLOUD_PUBLISHER {
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        ros::Subscriber sub_points_;
        ros::Publisher pub_cloud_;
        sobit_navigation::PointCloudProcessor pcp_;
        std::string target_frame_;
        PointCloud::Ptr cloud_, cost_cloud_;
        RGB rgb, flor_rgb, flor_rgb_noise;
        bool frag = true;
        void cbPoints(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
            pcp_.transformFramePointCloud( target_frame_, cloud_msg, cloud_ );
            // cost_cloud_.header.stamp = ros::Time::now();
            // cost_cloud_.header.frame_id = "base_footprint";
            // cost_cloud_.points.clear();
            for (long i=0; i<cloud_->points.size(); i++) {
                rgb.r = cloud_->points[i].r;
                rgb.g = cloud_->points[i].g;
                rgb.b = cloud_->points[i].b;
                if ((((rgb.r < (flor_rgb.r - flor_rgb_noise.r)) || ((flor_rgb.r + flor_rgb_noise.r) < rgb.r)) || ((rgb.g < (flor_rgb.g - flor_rgb_noise.g)) || ((flor_rgb.g + flor_rgb_noise.g) < rgb.g)) || ((rgb.b < (flor_rgb.b - flor_rgb_noise.b)) || ((flor_rgb.b + flor_rgb_noise.b) < rgb.b))) && ((rgb.r != 0) || (rgb.g != 0) || (rgb.b != 0)))
                {
                    // geometry_msgs::Point32 pt;
                    PointT pt;
                    pt.x = cloud_->points[i].x;
                    pt.y = cloud_->points[i].y;
                    pt.z = 0.25;
                    cloud_->points[i].z = 0.25;
                    cost_cloud_->points.push_back(pt);
                }
            }
            // pcp_.setPassThroughParameters( "x", 0.2, 4.0 );
            // pcp_.passThrough( cloud_, cloud_ );
            // pcp_.setPassThroughParameters( "y", -4.0, 4.0 );
            // pcp_.passThrough( cloud_, cloud_ );
            // pcp_.setPassThroughParameters( "z", 0.05, 1.5 );
            // pcp_.passThrough( cloud_, cloud_ );
            // pcp_.voxelGrid( cloud_, cloud_ );
            // pcp_.radiusOutlierRemoval ( cloud_, cloud_ );
            // pcl_conversions::toPCL(cloud_msg->header.stamp, cloud_->header.stamp);
            // pub_cloud_.publish(cloud_);
            pub_cloud_.publish(cost_cloud_);
        }

    public:
        NOISE_COLOR_POINTCLOUD_PUBLISHER(): nh_(), pnh_("~") {
            std::string topic_name = pnh_.param<std::string>( "topic_name", "/points2" );
            flor_rgb.r = nh_.param<int>( "noise_color_pointcloud_publisher/rgb/R", 200 );
            flor_rgb.g = nh_.param<int>( "noise_color_pointcloud_publisher/rgb/G", 200 );
            flor_rgb.b = nh_.param<int>( "noise_color_pointcloud_publisher/rgb/B", 200 );
            flor_rgb_noise.r = pnh_.param<int>( "RGB_R_NOISE", 60 );
            flor_rgb_noise.g = pnh_.param<int>( "RGB_G_NOISE", 60 );
            flor_rgb_noise.b = pnh_.param<int>( "RGB_B_NOISE", 60 );
            double voxel_size = pnh_.param<double>( "voxel_size", 0.025 );
            double radius = pnh_.param<double>( "radius", 0.05 );
            int min_pt = pnh_.param<int>( "min_pt", 5 );
            target_frame_ = pnh_.param<std::string>( "target_frame", "base_footprint" );
            pcp_.setVoxelGridParameter( voxel_size );
            pcp_.setRadiusOutlierRemovalParameters ( radius, min_pt, false );
            pub_cloud_ = nh_.advertise<PointCloud>("/cloud_color_point", 1);
            sub_points_ = nh_.subscribe(topic_name, 5, &NOISE_COLOR_POINTCLOUD_PUBLISHER::cbPoints, this);
            cloud_.reset(new PointCloud());
        }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "noise_color_pointcloud_publisher");
    NOISE_COLOR_POINTCLOUD_PUBLISHER noise_color_pointcloud_publisher;
    ros::spin();
}