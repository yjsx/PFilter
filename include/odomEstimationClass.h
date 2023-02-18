
#ifndef _ODOM_ESTIMATION_CLASS_H_
#define _ODOM_ESTIMATION_CLASS_H_

//std lib
#include <string>
#include <math.h>
#include <vector>

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/common.h>

//ceres
#include <ceres/ceres.h>
#include <ceres/rotation.h>

//eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

//LOCAL LIB
#include "lidar.h"
#include "lidarOptimization.h"
#include <ros/ros.h>
typedef pcl::PointXYZRGB PointType;
class OdomEstimationClass 
{

    public:
    	OdomEstimationClass();
    	
		void init(lidar::Lidar lidar_param, double map_resolution, int k_new, float theta_p, int theta_max);	
		void initMapWithPoints(const pcl::PointCloud<PointType>::Ptr& edge_in, const pcl::PointCloud<PointType>::Ptr& surf_in);
		void updatePointsToMap(const pcl::PointCloud<PointType>::Ptr& edge_in, const pcl::PointCloud<PointType>::Ptr& surf_in);
		void getMap(pcl::PointCloud<PointType>::Ptr& laserCloudMap);

		Eigen::Isometry3d odom;
		pcl::PointCloud<PointType>::Ptr laserCloudCornerMap;
		pcl::PointCloud<PointType>::Ptr laserCloudSurfMap;
		float map_resolution;
		int k_new_surf;
		float theta_p_surf;
		int theta_max_surf;
		int k_new_edge;
		float theta_p_edge;
		int theta_max_edge;
	private:
		//optimization variable
		double parameters[7] = {0, 0, 0, 1, 0, 0, 0};
		Eigen::Map<Eigen::Quaterniond> q_w_curr = Eigen::Map<Eigen::Quaterniond>(parameters);
		Eigen::Map<Eigen::Vector3d> t_w_curr = Eigen::Map<Eigen::Vector3d>(parameters + 4);

		Eigen::Isometry3d last_odom;

		//kd-tree
		pcl::KdTreeFLANN<PointType>::Ptr kdtreeEdgeMap;
		pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfMap;

		//points downsampling before add to map
		pcl::VoxelGrid<PointType> downSizeFilterEdge;
		pcl::VoxelGrid<PointType> downSizeFilterSurf;

		//local map
		pcl::CropBox<PointType> cropBoxFilter;

		//optimization count 
		int optimization_count;

		//function
		void addEdgeCostFactor(const pcl::PointCloud<PointType>::Ptr& pc_in, const pcl::PointCloud<PointType>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function);
		void addSurfCostFactor(const pcl::PointCloud<PointType>::Ptr& pc_in, const pcl::PointCloud<PointType>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function);
		void addPointsToMap(const pcl::PointCloud<PointType>::Ptr& downsampledEdgeCloud, const pcl::PointCloud<PointType>::Ptr& downsampledSurfCloud);
		void pointAssociateToMap(PointType const *const pi, PointType *const po);
		void downSamplingToMap(const pcl::PointCloud<PointType>::Ptr& edge_pc_in, pcl::PointCloud<PointType>::Ptr& edge_pc_out, const pcl::PointCloud<PointType>::Ptr& surf_pc_in, pcl::PointCloud<PointType>::Ptr& surf_pc_out);
};

#endif // _ODOM_ESTIMATION_CLASS_H_

