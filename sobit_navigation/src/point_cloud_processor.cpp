#include <sobit_navigation/point_cloud_processor.hpp>

using namespace sobit_navigation;

PointCloudProcessor::PointCloudProcessor() : tfBuffer_(), tf_listener_(tfBuffer_) {
    setPassThroughParameters( "z", 0.1, 1.0);
    setVoxelGridParameter( 0.01 );
    setClusteringParameters( 0.05, 1, 1000 );
    setSACSegmentationParameter( pcl::SACMODEL_PLANE, pcl::SAC_RANSAC, 0.01, 0.95 );
    setRadiusOutlierRemovalParameters( 0.05, 20, false );
    tree_ .reset ( new pcl::search::KdTree<PointT>() );
}
bool PointCloudProcessor::transformFramePointCloud ( const std::string target_frame, const sensor_msgs::PointCloud2ConstPtr &input_cloud, PointCloud::Ptr output_cloud ) {
    PointCloud cloud_src;
    pcl::fromROSMsg<PointT>( *input_cloud, cloud_src );
    if (target_frame.empty() == false ){
        try {
            while (ros::ok()) {
                if (tfBuffer_.canTransform(target_frame, cloud_src.header.frame_id, ros::Time(0), ros::Duration(1.0))) {
                    break;
                } else {
                    ros::Duration(1.0).sleep();
                }
            }
            pcl_ros::transformPointCloud(target_frame, ros::Time(0), cloud_src, cloud_src.header.frame_id,  *output_cloud, tfBuffer_);
            output_cloud->header.frame_id = target_frame;
        } catch (tf2::TransformException ex) {
            ROS_ERROR("%s", ex.what());
            return false;
        }
    } else ROS_ERROR("Please set the target frame.");
    return true;
}
// passThrough a point cloud :
bool PointCloudProcessor::passThrough ( const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud ) {
    try {
        PointCloud::Ptr tmp ( new PointCloud() );
        pass_.setInputCloud( input_cloud );
        pass_.filter( *tmp );
        *output_cloud = *tmp;
        output_cloud->header.frame_id = input_cloud->header.frame_id;
        return true;
    } catch ( std::exception& ex ) {
        ROS_ERROR("%s", ex.what());
        return false;
    }
}
// Downsample a point cloud :
bool PointCloudProcessor::voxelGrid ( const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud ) {
    try {
        PointCloud::Ptr tmp ( new PointCloud() );
        voxel_.setInputCloud( input_cloud );
        voxel_.filter( *tmp );
        *output_cloud = *tmp;
        output_cloud->header.frame_id = input_cloud->header.frame_id;
        return true;
    } catch ( std::exception& ex ) {
        ROS_ERROR("%s", ex.what());
        return false;
    }
}
// Removing outliers using a RadiusOutlier removal :
bool PointCloudProcessor::radiusOutlierRemoval ( const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud ) {
    try {
        PointCloud::Ptr tmp ( new PointCloud() );
        outrem_.setInputCloud( input_cloud );
        outrem_.filter (*tmp);
        *output_cloud = *tmp;
        output_cloud->header.frame_id = input_cloud->header.frame_id;
        return true;
    } catch ( std::exception& ex ) {
        ROS_ERROR("%s", ex.what());
        return false;
    }
}