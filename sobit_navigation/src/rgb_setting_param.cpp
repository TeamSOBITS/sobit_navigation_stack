#include <ros/ros.h>
// #include <time.h>
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


class FLOR_COLOR_PUBLISHER {
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        ros::Subscriber sub_points_;
        ros::Publisher pub_rgb_;
        sobit_navigation::PointCloudProcessor pcp_;
        std::string target_frame_;
        PointCloud::Ptr cloud_;
        RGB rgb, flor_rgb;
        bool getter_flag;
        std::vector<uint> r;
        std::vector<uint> g;
        std::vector<uint> b;
        std::string file_path = ros::package::getPath("sobit_navigation") + "/param/sobit_pro_multi_sensor/rgb_base.yaml";
        YAML::Node yaml_data;
        void cbPoints(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
            // pcp_.transformFramePointCloud( "base_footprint", cloud_msg, cloud_ );
            pcp_.transformFramePointCloud( "azure_rgb_camera_link", cloud_msg, cloud_ );
            r.clear();
            g.clear();
            b.clear();
            for (int i=0; i<cloud_msg->data.size()/32; i++)
            {
                rgb.r = cloud_msg->data[i*32 + 18];
                rgb.g = cloud_msg->data[i*32 + 17];
                rgb.b = cloud_msg->data[i*32 + 16];
                if (((rgb.r != 0) || (rgb.g != 0) || (rgb.b != 0)) && (cloud_->points[i].z < 0.04))
                {
                    r.push_back(rgb.r);
                    g.push_back(rgb.g);
                    b.push_back(rgb.b);
                }
            }
            std::sort(r.begin(), r.end());
            std::sort(g.begin(), g.end());
            std::sort(b.begin(), b.end());
            flor_rgb.r = r[r.size()/2];
            flor_rgb.g = g[g.size()/2];
            flor_rgb.b = b[b.size()/2];
            // yaml_data["R"] = flor_rgb.r;
            // yaml_data["G"] = flor_rgb.g;
            // yaml_data["B"] = flor_rgb.b;
            // if (file.is_open())
            // {
            //     file << yaml_data;  // YAMLデータをファイルに書き込む
            //     file.close();
            // }
            // else
            // {
            //     ROS_ERROR_STREAM("Failed to open file: " << file_path);
            // }
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
            getter_flag = true;
        }
        bool save_rgb() {
            getter_flag = false;
            while (ros::ok())
            {
                ros::spinOnce();
                if (getter_flag)
                {
                    break;
                }
            }
            ros::spinOnce();
            yaml_data["R"] = flor_rgb.r;
            yaml_data["G"] = flor_rgb.g;
            yaml_data["B"] = flor_rgb.b;
            std::ofstream file(file_path);
            if (file.is_open())
            {
                file << yaml_data;  // YAMLデータをファイルに書き込む
                file.close();
            }
            else
            {
                ROS_ERROR_STREAM("Failed to open file: " << file_path);
                return false;
            }
            sleep(1);
            popen("roslaunch sobit_navigation sobit_pro_color_base_checker.launch", "r");
            sleep(4);
            popen("rosnode kill rgb_checker", "r");
            int check;
            ROS_INFO("This color will be setting base color.");
            ROS_INFO("Is it okay?");
            ROS_INFO("YES->1 NO->2 : ");
            scanf("%d",&check);
            if (check == 1)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
    public:
        FLOR_COLOR_PUBLISHER(): nh_(), pnh_("~") {
            flor_rgb.r = 200;
            flor_rgb.g = 200;
            flor_rgb.b = 200;
            std::string topic_name = pnh_.param<std::string>( "topic_name", "/points2" );
            double voxel_size = pnh_.param<double>( "voxel_size", 0.025 );
            double radius = pnh_.param<double>( "radius", 0.05 );
            int min_pt = pnh_.param<int>( "min_pt", 5 );
            target_frame_ = pnh_.param<std::string>( "target_frame", "base_footprint" );

            ROS_INFO("topic_name = '%s'", topic_name.c_str());
            pcp_.setVoxelGridParameter( 0.025 );
            pcp_.setRadiusOutlierRemovalParameters ( radius, min_pt, false );

            pub_rgb_ = nh_.advertise<PointCloud>("/flor_color", 1);
            sub_points_ = nh_.subscribe(topic_name, 5, &FLOR_COLOR_PUBLISHER::cbPoints, this);
            cloud_.reset(new PointCloud());
            while (ros::ok())
            {
                ros::spinOnce();
                bool end_flag = save_rgb();
                if (end_flag)
                {
                    ROS_INFO("SAVE RGB_BASE COLOR");
                    break;
                }
                else
                {
                    ROS_ERROR_STREAM("PLEASE RE-TRY...");
                    ROS_ERROR_STREAM("FOR EXAMPLE \'MOVE ROBOT\', \'ROTATE ROBOT\'");
                }
            }
        }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "flor_color_publisher");
    FLOR_COLOR_PUBLISHER flor_color_publisher;
    ros::spin();






    // popen("roslaunch sobit_navigation sobit_pro_color_base_checker.launch", "r");
    // sleep(4);
    // popen("rosnode kill rgb_checker", "r");
    // int check;
    // printf("This color will be setting base color\nIs it okay? : ");
    // scanf("%d",&check);
    // printf("\n%d\n",check);
    // ros::spin();





    // ros::NodeHandle nh_;
    // YAML::Node yaml_data;
    // yaml_data["R"] = 200;
    // yaml_data["G"] = 126;
    // yaml_data["B"] = 166;


    // // std::string file_path;
    // std::string file_path = ros::package::getPath("sobit_navigation") + "/param/sobit_pro_multi_sensor/rgb_base.yaml";
    // std::ofstream file(file_path);
    // if (file.is_open()) {
    // file << yaml_data;  // YAMLデータをファイルに書き込む
    // file.close();
    // ROS_INFO_STREAM("YAML file saved: " << file_path);
    // } else {
    // ROS_ERROR_STREAM("Failed to open file: " << file_path);
    // }



    return 0;
}