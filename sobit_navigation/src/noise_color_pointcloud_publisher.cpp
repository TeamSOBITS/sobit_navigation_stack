#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <cstring>
#include <sobit_navigation/point_cloud_processor.hpp>


// struct RGB {
//     uint r, g, b;
// };


class NoiseColorPointCloudPublisher {
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        ros::Subscriber sub_points_;
        ros::Publisher pub_cloud_;
        sobit_navigation::PointCloudProcessor pcp_;
        std::string target_frame_;
        PointCloud::Ptr cloud_, cost_cloud_;
        int flor_r, flor_g, flor_b, color_noise;
        // RGB flor_rgb;
        void cbPoints(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
            pcp_.transformFramePointCloud( target_frame_, cloud_msg, cloud_ );
            cost_cloud_->header.frame_id = target_frame_;
            cost_cloud_->points.clear();
            for (long i=0; i<cloud_->points.size(); i++) {
                if (checkNanInf(cloud_->points[i])) {
                    if (((cloud_->points[i].r < (flor_r - color_noise)) || ((flor_r + color_noise) < cloud_->points[i].r)) || 
                        ((cloud_->points[i].g < (flor_g - color_noise)) || ((flor_g + color_noise) < cloud_->points[i].g)) || 
                        ((cloud_->points[i].b < (flor_b - color_noise)) || ((flor_b + color_noise) < cloud_->points[i].b))) {
                        PointT pt;
                        pt.x = cloud_->points[i].x;
                        pt.y = cloud_->points[i].y;
                        pt.z = 0.25;
                        cost_cloud_->points.push_back(pt);
                    }
                }
            }
            pub_cloud_.publish(cost_cloud_);
        }
        bool checkNanInf(PointT pt) {
            if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) return false;
            else if (std::isinf(pt.x) || std::isinf(pt.y) || std::isinf(pt.z)) return false;
            return true;
        }
    public:
        NoiseColorPointCloudPublisher(): nh_(), pnh_("~") {
            target_frame_ = pnh_.param<std::string>( "target_frame", "base_footprint" );
            std::string topic_name = pnh_.param<std::string>( "topic_name", "/points2" );
            flor_r = pnh_.param<int>( "rgb/R", 200 );
            flor_g = pnh_.param<int>( "rgb/G", 200 );
            flor_b = pnh_.param<int>( "rgb/B", 200 );
            color_noise = pnh_.param<int>( "color_noise", 60 );
            pub_cloud_ = nh_.advertise<PointCloud>("cloud_color_point", 1);
            sub_points_ = nh_.subscribe(topic_name, 5, &NoiseColorPointCloudPublisher::cbPoints, this);
            cloud_.reset(new PointCloud());
            cost_cloud_.reset(new PointCloud());
        }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "noise_color_pointcloud_publisher");
    NoiseColorPointCloudPublisher noise_color_pointcloud_publisher;
    ros::spin();
}