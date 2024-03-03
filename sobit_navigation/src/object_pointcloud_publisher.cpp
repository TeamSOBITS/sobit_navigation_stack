#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <cstring>
#include <sobit_navigation/point_cloud_processor.hpp>

class ObjectPointCloudPublisher {
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        ros::Subscriber sub_points_;
        ros::Publisher pub_cloud_;
        PointCloud::Ptr cloud_;
        void cbPoints(const PointCloud::Ptr &cloud_msg) {
            cloud_->header.frame_id = cloud_msg->header.frame_id;
            cloud_->points.clear();
            for (int i=0; i<cloud_msg->points.size(); i++) {
                PointT pt;
                pt.x = cloud_msg->points[i].x;
                pt.y = cloud_msg->points[i].y;
                pt.z = 0.25;
                cloud_->points.push_back(pt);
            }
            pub_cloud_.publish(cloud_);
        }
    public:
        ObjectPointCloudPublisher(): nh_(), pnh_("~") {
            std::string object_point_cloud_topic_name  = pnh_.param<std::string>( "object_point_cloud_topic_name", "object_poses" );
            pub_cloud_ = nh_.advertise<PointCloud>("cloud_object_point", 1);
            sub_points_ = nh_.subscribe( object_point_cloud_topic_name , 5, &ObjectPointCloudPublisher::cbPoints, this);
            cloud_.reset(new PointCloud());
        }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "object_pointcloud_publisher");
    ObjectPointCloudPublisher object_pointcloud_publisher;
    ros::spin();
}