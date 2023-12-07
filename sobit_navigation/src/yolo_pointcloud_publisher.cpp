#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <cstring>
#include <sobit_navigation/point_cloud_processor.hpp>
#include <sobit_common_msg/ObjectPoseArray.h>
#include <sobit_common_msg/ObjectPose.h>

class YOLO_POINTCLOUD_PUBLISHER {
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        ros::Subscriber sub_points_;
        ros::Subscriber sub_yolo_;
        ros::Publisher pub_cloud_;
        sobit_navigation::PointCloudProcessor pcp_;
        std::string target_frame_;
        PointCloud::Ptr cloud_;
        double cost_range;
        bool frag = true;
        float object_x, object_y;

        void cbPoints(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
            pcp_.transformFramePointCloud( "base_footprint", cloud_msg, cloud_ );
            ROS_INFO("object_x:''%f",object_x);
            ROS_INFO("object_y:''%f",object_y);
            for (int i=0; i<cloud_msg->data.size()/32; i++)
            // for (int i=0; i<cloud_msg->data.size()/cloud_msg.point_step; i++)
            {
                if((object_x-0.10 < cloud_->points[i].x && cloud_->points[i].x < object_x+0.10) && (object_y-0.10 < cloud_->points[i].y && cloud_->points[i].y < object_y+0.10)){
                // if((1.0 < cloud_->points[i].x && cloud_->points[i].x < 2.0) && (-0.15 < cloud_->points[i].y && cloud_->points[i].y < 0.15)){
                    cloud_->points[i].z = 0.4;
                }
            }
            pcp_.setPassThroughParameters( "x", 0.2, 2.5 );
            pcp_.passThrough( cloud_, cloud_ );
            pcp_.setPassThroughParameters( "y", -3.0, 3.0 );
            pcp_.passThrough( cloud_, cloud_ );
            pcp_.setPassThroughParameters( "z", 0.05, 0.7 );
            pcp_.passThrough( cloud_, cloud_ );
            pcp_.voxelGrid( cloud_, cloud_ );
            pcp_.radiusOutlierRemoval ( cloud_, cloud_ );
            pcl_conversions::toPCL(cloud_msg->header.stamp, cloud_->header.stamp);
            pub_cloud_.publish(cloud_);
            
        }

        void cbTf(const sobit_common_msg::ObjectPoseArray& tf_msg){
            if(tf_msg.object_poses.size() > 0){
                sobit_common_msg::ObjectPose object_pose = tf_msg.object_poses[0];
                object_x = object_pose.pose.position.x;
                object_y = object_pose.pose.position.y;
            }
            else{
                object_x = 100;
                object_y = 100;
            }
        }

    public:
            YOLO_POINTCLOUD_PUBLISHER(): nh_(), pnh_("~") {
            std::string topic_name = pnh_.param<std::string>( "topic_name", "/points2" );
            std::string topic2_name = pnh_.param<std::string>( "topic2_name", "/yolov5_ros/object_poses" );
            double voxel_size = pnh_.param<double>( "voxel_size", 0.025 );
            double radius = pnh_.param<double>( "radius", 0.05 );
            double cost_range = pnh_.param<double>( "cost_range", 0.25 );
            int min_pt = pnh_.param<int>( "min_pt", 5 );
            target_frame_ = pnh_.param<std::string>( "target_frame", "base_footprint" );
            ROS_INFO("topic_name = '%s'", topic_name.c_str());
            pcp_.setVoxelGridParameter( 0.025 );
            pcp_.setRadiusOutlierRemovalParameters ( radius, min_pt, false );
            pub_cloud_ = nh_.advertise<PointCloud>("/cloud_yolo_point", 1);
            sub_points_ = nh_.subscribe("/points2", 5, &YOLO_POINTCLOUD_PUBLISHER::cbPoints, this);
            sub_yolo_ = nh_.subscribe(topic2_name, 5, &YOLO_POINTCLOUD_PUBLISHER::cbTf, this);
            cloud_.reset(new PointCloud());
        }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "yolo_pointcloud_publisher");
    YOLO_POINTCLOUD_PUBLISHER yolo_pointcloud_publisher;
    ros::spin();
}