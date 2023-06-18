#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <sobit_navigation/point_cloud_processor.hpp>


struct RGB {
    uint r, g, b;
};


class COLOR_POINT_PUBLISHER {
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        ros::Subscriber sub_points_;
        ros::Publisher pub_cloud_;
        sobit_navigation::PointCloudProcessor pcp_;
        std::string target_frame_;
        PointCloud::Ptr cloud_;
        RGB rgb, flor_rgb, flor_rgb_noise;
        bool frag = true;
        std::vector<uint> r;
        std::vector<uint> g;
        std::vector<uint> b;

        void cbPoints(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
            // pcp_.transformFramePointCloud( "base_footprint", cloud_msg, cloud_ );
            pcp_.transformFramePointCloud( "azure_rgb_camera_link", cloud_msg, cloud_ );
            r.clear();
            g.clear();
            b.clear();
            for (int i=0; i<cloud_msg.size()/32; i++)
            {
                rgb.r = cloud->data[i*32 + 18];
                rgb.g = cloud->data[i*32 + 17];
                rgb.b = cloud->data[i*32 + 16];
                unsigned char bytes[4];
                // bytes[0] = cloud_->;
                // bytes[1] = cloud_;
                // bytes[2] = cloud_;
                // bytes[3] = cloud_;
                // バイト配列をfloatに変換
                float value;
                std::memcpy(&value, bytes, sizeof(float));
                if (((rgb.r != 0) || (rgb.g != 0) || (rgb.b != 0)) && ())
                {
                    r.push_back(rgb.r);
                    g.push_back(rgb.g);
                    b.push_back(rgb.b);
                }
            }
            std::sort(r.begin(), r.end());
            std::sort(g.begin(), g.end());
            std::sort(b.begin(), b.end());
            // pcp_.setPassThroughParameters( "x", 0.4, 8.0 );
            // pcp_.passThrough( cloud_, cloud_ );
            // pcp_.setPassThroughParameters( "y", -4.0, 4.0 );
            // pcp_.passThrough( cloud_, cloud_ );
            // pcp_.setPassThroughParameters( "z", 0.05, 1.5 );
            // pcp_.passThrough( cloud_, cloud_ );
            // pcp_.voxelGrid( cloud_, cloud_ );
            // pcp_.radiusOutlierRemoval ( cloud_, cloud_ );
            // pcp_.colorRemoval ( cloud_, cloud_ );
            // pcl_conversions::toPCL(cloud_msg->header.stamp, cloud_->header.stamp);
            // pub_cloud_.publish(cloud_);
        }

    public:
        COLOR_POINT_PUBLISHER(): nh_(), pnh_("~") {
            std::string topic_name = pnh_.param<std::string>( "topic_name", "/points2" );
            flor_rgb.r = pnh_.param<int>( "RGB_R", 50 );
            flor_rgb.g = pnh_.param<int>( "RGB_G", 50 );
            flor_rgb.b = pnh_.param<int>( "RGB_B", 50 );
            flor_rgb_noise.r = pnh_.param<int>( "RGB_R_NOISE", 5 );
            flor_rgb_noise.g = pnh_.param<int>( "RGB_G_NOISE", 5 );
            flor_rgb_noise.b = pnh_.param<int>( "RGB_B_NOISE", 5 );
            double voxel_size = pnh_.param<double>( "voxel_size", 0.025 );
            double radius = pnh_.param<double>( "radius", 0.05 );
            int min_pt = pnh_.param<int>( "min_pt", 5 );
            target_frame_ = pnh_.param<std::string>( "target_frame", "base_footprint" );

            ROS_INFO("topic_name = '%s'", topic_name.c_str());
            pcp_.setVoxelGridParameter( 0.025 );
            pcp_.setRadiusOutlierRemovalParameters ( radius, min_pt, false );

            pub_cloud_ = nh_.advertise<PointCloud>("/cloud_color_point", 1);
            sub_points_ = nh_.subscribe(topic_name, 5, &COLOR_POINT_PUBLISHER::cbPoints, this);
            cloud_.reset(new PointCloud());
        }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "color_point_publisher");
    // COLOR_POINT_PUBLISHER color_point_publisher;
    // ros::spin();
    return 0;
}