#include <ros/ros.h>
#include <unistd.h>
#include <iostream>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <sobit_navigation/point_cloud_processor.hpp>


struct RGB {
    uint r, g, b;
};


class FLOOR_COLOR_PUBLISHER {
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        ros::Subscriber sub_points_;
        sobit_navigation::PointCloudProcessor pcp_;
        std::string target_frame_;
        PointCloud::Ptr cloud_;
        RGB rgb, floor_rgb;
        bool getter_flag;
        double max_floor_z;
        std::vector<uint> r;
        std::vector<uint> g;
        std::vector<uint> b;
        std::string file_path = ros::package::getPath("sobit_navigation") + "/param/floor_color/rgb_base.yaml";
        YAML::Node yaml_data;
        void cbPoints(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
            pcp_.transformFramePointCloud( target_frame_, cloud_msg, cloud_ );
            r.clear();
            g.clear();
            b.clear();
            for (long i=0; i<cloud_->points.size(); i++) {
                rgb.r = cloud_->points[i].r;
                rgb.g = cloud_->points[i].g;
                rgb.b = cloud_->points[i].b;
                if (((rgb.r != 0) || (rgb.g != 0) || (rgb.b != 0)) && (cloud_->points[i].z <= max_floor_z)) {
                    r.push_back(rgb.r);
                    g.push_back(rgb.g);
                    b.push_back(rgb.b);
                }
            }
            if ((r.size() != 0) && (g.size() != 0) && (b.size() != 0)) {
                std::sort(r.begin(), r.end());
                std::sort(g.begin(), g.end());
                std::sort(b.begin(), b.end());
                floor_rgb.r = r[r.size()/2];
                floor_rgb.g = g[g.size()/2];
                floor_rgb.b = b[b.size()/2];
                getter_flag = true;
            }
        }
        bool save_rgb() {
            getter_flag = false;
            while (ros::ok()) {
                ros::spinOnce();
                if (getter_flag) break;
            }
            ros::spinOnce();
            yaml_data["R"] = floor_rgb.r;
            yaml_data["G"] = floor_rgb.g;
            yaml_data["B"] = floor_rgb.b;
            std::ofstream file(file_path);
            if (file.is_open()) {
                file << yaml_data;
                file.close();
            } else {
                ROS_ERROR_STREAM("Failed to open file: " << file_path);
                return false;
            }
            sleep(1);
            FILE *fopen = popen("rosrun sobit_mapping rgb_checker.py", "r");
            sleep(4);
            FILE *fkill = popen("rosnode kill rgb_checker", "r");
            int check;
            ROS_INFO("This color will be setting base color.");
            ROS_INFO("Is it okay?");
            ROS_INFO("YES->1 NO->2 : ");
            if (scanf("%d",&check)) {
                if (check == 1) return true;
            }
            return false;
        }
    public:
        FLOOR_COLOR_PUBLISHER(): nh_(), pnh_("~") {
            std::string topic_name = pnh_.param<std::string>( "topic_name", "/points2" );
            double voxel_size = pnh_.param<double>( "voxel_size", 0.025 );
            double radius = pnh_.param<double>( "radius", 0.05 );
            max_floor_z = pnh_.param<double>( "max_floor_z", 0.04 );
            int min_pt = pnh_.param<int>( "min_pt", 5 );
            target_frame_ = pnh_.param<std::string>( "target_frame", "base_footprint" );
            pcp_.setVoxelGridParameter( voxel_size );
            pcp_.setRadiusOutlierRemovalParameters ( radius, min_pt, false );
            sub_points_ = nh_.subscribe(topic_name, 5, &FLOOR_COLOR_PUBLISHER::cbPoints, this);
            cloud_.reset(new PointCloud());
            while (ros::ok()) {
                ros::spinOnce();
                bool end_flag = save_rgb();
                if (end_flag) {
                    ROS_INFO("SAVE RGB_BASE COLOR");
                    break;
                } else {
                    ROS_ERROR_STREAM("PLEASE RE-TRY...");
                    ROS_ERROR_STREAM("FOR EXAMPLE \'MOVE ROBOT\', \'ROTATE ROBOT\'");
                }
            }
        }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "floor_color_setting");
    FLOOR_COLOR_PUBLISHER floor_color_publisher;
    ros::spin();
    return 0;
}