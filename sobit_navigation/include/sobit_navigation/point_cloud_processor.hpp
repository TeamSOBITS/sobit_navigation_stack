#ifndef POINT_CLOUD_PROCESSOR_HPP
#define POINT_CLOUD_PROCESSOR_HPP

#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;


namespace sobit_navigation {
    class PointCloudProcessor {
        protected:
            tf::TransformListener tf_listener_;
            pcl::PassThrough<PointT> pass_;
            pcl::VoxelGrid<PointT> voxel_;
            pcl::search::KdTree<PointT>::Ptr tree_;
            pcl::EuclideanClusterExtraction<PointT> ec_;
            pcl::ExtractIndices<PointT> extract_;
            pcl::RadiusOutlierRemoval<PointT> outrem_;
            pcl::SACSegmentation<PointT> seg_;

        public:
            PointCloudProcessor();
            void setPassThroughParameters( const std::string &axis, const float &limit_min, const float &limit_max );
            void setVoxelGridParameter( const float leaf_size );
            void setClusteringParameters ( const float tolerance, const int min_size, const int max_size );
            void setRadiusOutlierRemovalParameters ( const double radius, const int min_pts, const bool keep_organized );
            void setSACSegmentationParameter( const int model,  const int method, const double threshold, const double probability );
            void setSACPlaneParameter( const std::string &axis, const double eps_angle_degree );

            bool transformFramePointCloud ( const std::string target_frame, const sensor_msgs::PointCloud2ConstPtr &input_cloud, PointCloud::Ptr output_cloud );
            bool passThrough ( const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud );
            bool voxelGrid ( const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud );
            bool euclideanClusterExtraction ( const PointCloud::Ptr input_cloud, std::vector<pcl::PointIndices>* output_indices );
            bool extractIndices( const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud, const pcl::PointIndices::Ptr indices, bool negative );
            bool statisticalRemoval ( const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud, const int nr_k, const double stddev_mult );
            bool radiusOutlierRemoval ( const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud );
            bool sacSegmentation( const PointCloud::Ptr input_cloud, pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients );
    };
    
    inline void PointCloudProcessor::setPassThroughParameters( const std::string &axis, const float &limit_min, const float &limit_max ) {
        pass_.setFilterFieldName( axis );
        pass_.setFilterLimits( limit_min, limit_max);
    }
    inline void PointCloudProcessor::setVoxelGridParameter( const float leaf_size ) {
        voxel_.setLeafSize( leaf_size, leaf_size, leaf_size );
    }
    inline void PointCloudProcessor::setClusteringParameters ( const float tolerance, const int min_size, const int max_size ) {
        ec_.setClusterTolerance( tolerance );
        ec_.setMinClusterSize( min_size );
        ec_.setMaxClusterSize( max_size );
        ec_.setSearchMethod( tree_ );
    }
    inline void PointCloudProcessor::setRadiusOutlierRemovalParameters ( const double radius, const int min_pts, const bool keep_organized ) {
        outrem_.setRadiusSearch( radius );
        outrem_.setMinNeighborsInRadius ( min_pts );
        outrem_.setKeepOrganized( keep_organized );
    }
    inline void PointCloudProcessor::setSACSegmentationParameter( const int model,  const int method, const double threshold, const double probability ) {
        seg_.setOptimizeCoefficients (true);
        seg_.setModelType (model);
        seg_.setMethodType (method);      
        seg_.setDistanceThreshold (threshold);
        seg_.setProbability(probability);  
        seg_.setMaxIterations(1000);
    }
    inline void PointCloudProcessor::setSACPlaneParameter( const std::string &axis, const double eps_angle_degree ) {
        Eigen::Vector3f axis_vec;
        if ( axis == "x" ) axis_vec = Eigen::Vector3f(1.0,0.0,0.0); //y axis
        else if ( axis == "y" ) axis_vec = Eigen::Vector3f(0.0,1.0,0.0); //y axis
        else if ( axis == "z" ) axis_vec = Eigen::Vector3f(0.0,0.0,1.0); //y axis
        else return;
        seg_.setAxis(axis_vec);
        seg_.setEpsAngle( eps_angle_degree * (M_PI/180.0f) ); // plane can be within eps_angle_degree degrees of plane
    }
}

#endif