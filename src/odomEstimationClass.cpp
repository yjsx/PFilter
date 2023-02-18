
#include "odomEstimationClass.h"

struct cloud_point_index_idx 
{
    unsigned int idx;
    unsigned int cloud_point_index;

    cloud_point_index_idx (unsigned int idx_, unsigned int cloud_point_index_) : idx (idx_), cloud_point_index (cloud_point_index_) {}
    bool operator < (const cloud_point_index_idx &p) const { return (idx < p.idx); }
};

void extractstablepoint(pcl::PointCloud<PointType>::Ptr input, int k_new, float theta_p, int theta_max){
    std::vector<int> index;
    for(int i = 0; i < input->points.size(); i++){
        if(input->points[i].g < input->points[i].r * theta_p && input->points[i].r > k_new && input->points[i].g <theta_max + 1)
            continue; 
        index.push_back(i);
    }
    boost::shared_ptr<std::vector<int>> index_ptr = boost::make_shared<std::vector<int>>(index);
    // Create the filtering object
    pcl::ExtractIndices<PointType> extract;
    // Extract the inliers
    extract.setInputCloud (input);
    extract.setIndices (index_ptr);
    extract.setNegative (false);//如果设为true,可以提取指定index之外的点云
    extract.filter (*input);
}
pcl::PointCloud<PointType>::Ptr rgbds (pcl::PointCloud<PointType>::Ptr input, float dsleaf, int min_points_per_voxel_=0){
    pcl::PointCloud<PointType>::Ptr output(new pcl::PointCloud<PointType>);
    output->height = 1;        
	output->is_dense = true; 

	Eigen::Vector4f min_p, max_p;
    Eigen::Vector4i min_b_, max_b_, div_b_, divb_mul_;
    pcl::getMinMax3D<PointType> (*input, min_p, max_p);

    min_b_[0] = static_cast<int> (floor (min_p[0] / dsleaf));
	max_b_[0] = static_cast<int> (floor (max_p[0] / dsleaf));
	min_b_[1] = static_cast<int> (floor (min_p[1] / dsleaf));
	max_b_[1] = static_cast<int> (floor (max_p[1] / dsleaf));
	min_b_[2] = static_cast<int> (floor (min_p[2] / dsleaf));
	max_b_[2] = static_cast<int> (floor (max_p[2] / dsleaf));

    div_b_ = max_b_ - min_b_ + Eigen::Vector4i::Ones ();
	div_b_[3] = 0;
	divb_mul_ = Eigen::Vector4i (1, div_b_[0], div_b_[0] * div_b_[1], 0);

    std::vector<cloud_point_index_idx> index_vector;
	index_vector.reserve (input->points.size ());

    for (int i = 0; i < input->points.size(); i++){
        int ijk0 = static_cast<int> (floor (input->points[i].x / dsleaf) - static_cast<float> (min_b_[0]));
        int ijk1 = static_cast<int> (floor (input->points[i].y / dsleaf) - static_cast<float> (min_b_[1]));
        int ijk2 = static_cast<int> (floor (input->points[i].z / dsleaf) - static_cast<float> (min_b_[2]));

        // Compute the centroid leaf index
        int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];
        index_vector.push_back (cloud_point_index_idx (static_cast<unsigned int> (idx), i));
    }

	// Second pass: sort the index_vector vector using value representing target cell as index
	// in effect all points belonging to the same output cell will be next to each other
	std::sort (index_vector.begin (), index_vector.end (), std::less<cloud_point_index_idx> ());

	// Third pass: count output cells
	// we need to skip all the same, adjacenent idx values
	unsigned int total = 0;
	unsigned int index = 0;
	// first_and_last_indices_vector[i] represents the index in index_vector of the first point in
	// index_vector belonging to the voxel which corresponds to the i-th output point,
	// and of the first point not belonging to.
	std::vector<std::pair<unsigned int, unsigned int> > first_and_last_indices_vector;
	// Worst case size
	first_and_last_indices_vector.reserve (index_vector.size ());
	while (index < index_vector.size ()) 
	{
		unsigned int i = index + 1;
		while (i < index_vector.size () && index_vector[i].idx == index_vector[index].idx) 
		    ++i;
		if (i - index >= min_points_per_voxel_){
    		++total;
	     	first_and_last_indices_vector.push_back (std::pair<unsigned int, unsigned int> (index, i));
		}
		index = i;
	}

	// Fourth pass: compute centroids, insert them into their final position
	output->points.resize (total);
	index = 0;
	for (unsigned int cp = 0; cp < first_and_last_indices_vector.size (); ++cp){
		// calculate centroid - sum values from all input points, that have the same idx value in index_vector array
		unsigned int first_index = first_and_last_indices_vector[cp].first;
		unsigned int last_index = first_and_last_indices_vector[cp].second;

		
        Eigen::Vector4f centroid (Eigen::Vector4f::Zero ());

        int r_max = -1;
        float g_max = -1; 
        for (unsigned int li = first_index; li < last_index; ++li){
            centroid += input->points[index_vector[li].cloud_point_index].getVector4fMap ();
            if(input->points[index_vector[li].cloud_point_index].r > r_max){
                r_max = input->points[index_vector[li].cloud_point_index].r;
                // centroid = input->points[index_vector[li].cloud_point_index].getVector4fMap ();
            }
            if(input->points[index_vector[li].cloud_point_index].g > g_max){
                g_max = input->points[index_vector[li].cloud_point_index].g;
            }
        }
        centroid /= static_cast<float> (last_index - first_index);
        output->points[index].getVector4fMap () = centroid;
        output->points[index].r = r_max;
        output->points[index].g = g_max;

		++index;
	}
	output->width = static_cast<uint32_t> (output->points.size ());
	return output;
}
void OdomEstimationClass::init(lidar::Lidar lidar_param, double map_resolution_in,  int k_new_para, float theta_p_para, int theta_max_para){
    //init local map
    laserCloudCornerMap = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
    laserCloudSurfMap = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());

    //downsampling size
    downSizeFilterEdge.setLeafSize(map_resolution_in, map_resolution_in, map_resolution_in);
    downSizeFilterSurf.setLeafSize(map_resolution_in * 2, map_resolution_in * 2, map_resolution_in * 2);

    //kd-tree
    kdtreeEdgeMap = pcl::KdTreeFLANN<PointType>::Ptr(new pcl::KdTreeFLANN<PointType>());
    kdtreeSurfMap = pcl::KdTreeFLANN<PointType>::Ptr(new pcl::KdTreeFLANN<PointType>());

    odom = Eigen::Isometry3d::Identity();
    last_odom = Eigen::Isometry3d::Identity();
    optimization_count=2;
    k_new_surf = k_new_para;
    theta_p_surf = theta_p_para;
    theta_max_surf = theta_max_para;
    k_new_edge = k_new_para;
    theta_p_edge = theta_p_para;
    theta_max_edge = theta_max_para;

    map_resolution = map_resolution_in;
}

void OdomEstimationClass::initMapWithPoints(const pcl::PointCloud<PointType>::Ptr& edge_in, const pcl::PointCloud<PointType>::Ptr& surf_in){
    *laserCloudCornerMap += *edge_in;
    *laserCloudSurfMap += *surf_in;
    optimization_count=12;
}


void OdomEstimationClass::updatePointsToMap(const pcl::PointCloud<PointType>::Ptr& edge_in, const pcl::PointCloud<PointType>::Ptr& surf_in){

    if(optimization_count>2)
        optimization_count--;

    Eigen::Isometry3d odom_prediction = odom * (last_odom.inverse() * odom);
    last_odom = odom;
    odom = odom_prediction;

    q_w_curr = Eigen::Quaterniond(odom.rotation());
    t_w_curr = odom.translation();

    pcl::PointCloud<PointType>::Ptr downsampledEdgeCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr downsampledSurfCloud(new pcl::PointCloud<PointType>());
    downSamplingToMap(edge_in,downsampledEdgeCloud,surf_in,downsampledSurfCloud);
    //ROS_WARN("point nyum%d,%d",(int)downsampledEdgeCloud->points.size(), (int)downsampledSurfCloud->points.size());
    if(laserCloudCornerMap->points.size()>10 && laserCloudSurfMap->points.size()>50){
        kdtreeEdgeMap->setInputCloud(laserCloudCornerMap);
        kdtreeSurfMap->setInputCloud(laserCloudSurfMap);

        for (int iterCount = 0; iterCount < optimization_count; iterCount++){
            ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
            ceres::Problem::Options problem_options;
            ceres::Problem problem(problem_options);

            problem.AddParameterBlock(parameters, 7, new PoseSE3Parameterization());
            
            addEdgeCostFactor(downsampledEdgeCloud,laserCloudCornerMap,problem,loss_function);
            addSurfCostFactor(downsampledSurfCloud,laserCloudSurfMap,problem,loss_function);

            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            options.max_num_iterations = 4;
            options.minimizer_progress_to_stdout = false;
            options.check_gradients = false;
            options.gradient_check_relative_precision = 1e-4;
            ceres::Solver::Summary summary;

            ceres::Solve(options, &problem, &summary);

        }
    }else{
        printf("not enough points in map to associate, map error");
    }
    odom = Eigen::Isometry3d::Identity();
    odom.linear() = q_w_curr.toRotationMatrix();
    odom.translation() = t_w_curr;
    addPointsToMap(downsampledEdgeCloud,downsampledSurfCloud);

}

void OdomEstimationClass::pointAssociateToMap(PointType const *const pi, PointType *const po)
{
    Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
    Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
    po->x = point_w.x();
    po->y = point_w.y();
    po->z = point_w.z();
    po->r = pi->r;
    po->g = pi->g;
    po->b = pi->b;
    // po->intensity = pi->intensity;
    //po->intensity = 1.0;
}

void OdomEstimationClass::downSamplingToMap(const pcl::PointCloud<PointType>::Ptr& edge_pc_in, pcl::PointCloud<PointType>::Ptr& edge_pc_out, const pcl::PointCloud<PointType>::Ptr& surf_pc_in, pcl::PointCloud<PointType>::Ptr& surf_pc_out){
    downSizeFilterEdge.setInputCloud(edge_pc_in);
    downSizeFilterEdge.filter(*edge_pc_out);
    downSizeFilterSurf.setInputCloud(surf_pc_in);
    downSizeFilterSurf.filter(*surf_pc_out);    
}

void OdomEstimationClass::addEdgeCostFactor(const pcl::PointCloud<PointType>::Ptr& pc_in, const pcl::PointCloud<PointType>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function){
    int corner_num=0;
    for (int i = 0; i < (int)pc_in->points.size(); i++)
    {
        PointType point_temp;
        pointAssociateToMap(&(pc_in->points[i]), &point_temp);

        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        kdtreeEdgeMap->nearestKSearch(point_temp, 5, pointSearchInd, pointSearchSqDis); 
        if (pointSearchSqDis[4] < 1.0)
        {
            std::vector<Eigen::Vector3d> nearCorners;
            Eigen::Vector3d center(0, 0, 0);
            for (int j = 0; j < 5; j++)
            {
                Eigen::Vector3d tmp(map_in->points[pointSearchInd[j]].x,
                                    map_in->points[pointSearchInd[j]].y,
                                    map_in->points[pointSearchInd[j]].z);
                center = center + tmp;
                nearCorners.push_back(tmp);
            }
            center = center / 5.0;

            Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
            for (int j = 0; j < 5; j++)
            {
                Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
                covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
            }

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

            Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
            Eigen::Vector3d curr_point(pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z);
            if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
            { 
                Eigen::Vector3d point_on_line = center;
                Eigen::Vector3d point_a, point_b;
                point_a = 0.1 * unit_direction + point_on_line;
                point_b = -0.1 * unit_direction + point_on_line;
                float observe = (map_in->points[pointSearchInd[0]].g + 
                                map_in->points[pointSearchInd[1]].g +
                                map_in->points[pointSearchInd[2]].g +
                                map_in->points[pointSearchInd[3]].g +
                                map_in->points[pointSearchInd[4]].g ) / 5.0 + 1;
                float round = (map_in->points[pointSearchInd[0]].r + 
                                map_in->points[pointSearchInd[1]].r +
                                map_in->points[pointSearchInd[2]].r +
                                map_in->points[pointSearchInd[3]].r +
                                map_in->points[pointSearchInd[4]].r ) / 5.0;
                for (int j = 0; j < 5; j++)
                    map_in->points[pointSearchInd[j]].g = std::min(255, map_in->points[pointSearchInd[j]].g + 1);
                
                if(observe / round > 5)
                    observe = 255;
                if(observe < round * theta_p_edge && round > k_new_edge && observe < theta_max_edge){
                    continue;
                }
                pc_in->points[i].r = std::min(255,int(round));
                pc_in->points[i].g = std::min(255,int(observe));
                ceres::CostFunction *cost_function = new EdgeAnalyticCostFunction(curr_point, point_a, point_b);  
                problem.AddResidualBlock(cost_function, loss_function, parameters);
                corner_num++;   
            }                           
        }
    }
    if(corner_num<20){
        printf("not enough correct points");
    }

}

void OdomEstimationClass::addSurfCostFactor(const pcl::PointCloud<PointType>::Ptr& pc_in, const pcl::PointCloud<PointType>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function){
    int surf_num=0;
    for (int i = 0; i < (int)pc_in->points.size(); i++)
    {
        PointType point_temp;
        pointAssociateToMap(&(pc_in->points[i]), &point_temp);
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        kdtreeSurfMap->nearestKSearch(point_temp, 5, pointSearchInd, pointSearchSqDis);

        Eigen::Matrix<double, 5, 3> matA0;
        Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
        if (pointSearchSqDis[4] < 1.0)
        {
            
            for (int j = 0; j < 5; j++)
            {
                matA0(j, 0) = map_in->points[pointSearchInd[j]].x;
                matA0(j, 1) = map_in->points[pointSearchInd[j]].y;
                matA0(j, 2) = map_in->points[pointSearchInd[j]].z;
            }
            // find the norm of plane
            Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
            double negative_OA_dot_norm = 1 / norm.norm();
            norm.normalize();

            bool planeValid = true;
            for (int j = 0; j < 5; j++)
            {
                // if OX * n > 0.2, then plane is not fit well
                if (fabs(norm(0) * map_in->points[pointSearchInd[j]].x +
                         norm(1) * map_in->points[pointSearchInd[j]].y +
                         norm(2) * map_in->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2)
                {
                    planeValid = false;
                    break;
                }
            }
            Eigen::Vector3d curr_point(pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z);
            if (planeValid)
            {
                float observe = (map_in->points[pointSearchInd[0]].g + 
                                map_in->points[pointSearchInd[1]].g +
                                map_in->points[pointSearchInd[2]].g +
                                map_in->points[pointSearchInd[3]].g +
                                map_in->points[pointSearchInd[4]].g ) / 5.0 + 1;
                float round = (map_in->points[pointSearchInd[0]].r + 
                                map_in->points[pointSearchInd[1]].r +
                                map_in->points[pointSearchInd[2]].r +
                                map_in->points[pointSearchInd[3]].r +
                                map_in->points[pointSearchInd[4]].r ) / 5.0;
                for (int j = 0; j < 5; j++){   
                    map_in->points[pointSearchInd[j]].g  = std::min(255, map_in->points[pointSearchInd[j]].g + 1);
                } 
                if(observe / round > 5)
                    observe = 255;
                if(observe < round * theta_p_surf && round > k_new_surf && observe < theta_max_surf){
                    continue;
                }
                pc_in->points[i].r = std::min(255,int(round));
                pc_in->points[i].g = std::min(255,int(observe));
                ceres::CostFunction *cost_function = new SurfNormAnalyticCostFunction(curr_point, norm, negative_OA_dot_norm);    
                problem.AddResidualBlock(cost_function, loss_function, parameters);

                surf_num++;
            }
        }

    }
    if(surf_num<20){
        printf("not enough correct points");
    }

}

void OdomEstimationClass::addPointsToMap(const pcl::PointCloud<PointType>::Ptr& downsampledEdgeCloud, const pcl::PointCloud<PointType>::Ptr& downsampledSurfCloud){

    for (int i = 0; i < (int)downsampledEdgeCloud->points.size(); i++)
    {
        PointType point_temp;
        pointAssociateToMap(&downsampledEdgeCloud->points[i], &point_temp);
        laserCloudCornerMap->push_back(point_temp); 
    }
    
    for (int i = 0; i < (int)downsampledSurfCloud->points.size(); i++)
    {
        PointType point_temp;
        pointAssociateToMap(&downsampledSurfCloud->points[i], &point_temp);
        laserCloudSurfMap->push_back(point_temp);
    }
    
    double x_min = +odom.translation().x()-100;
    double y_min = +odom.translation().y()-100;
    double z_min = +odom.translation().z()-100;
    double x_max = +odom.translation().x()+100;
    double y_max = +odom.translation().y()+100;
    double z_max = +odom.translation().z()+100;
    
    //ROS_INFO("size : %f,%f,%f,%f,%f,%f", x_min, y_min, z_min,x_max, y_max, z_max);
    cropBoxFilter.setMin(Eigen::Vector4f(x_min, y_min, z_min, 1.0));
    cropBoxFilter.setMax(Eigen::Vector4f(x_max, y_max, z_max, 1.0));
    cropBoxFilter.setNegative(false);    

    pcl::PointCloud<PointType>::Ptr tmpCorner(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr tmpSurf(new pcl::PointCloud<PointType>());
    cropBoxFilter.setInputCloud(laserCloudSurfMap);
    cropBoxFilter.filter(*tmpSurf);
    cropBoxFilter.setInputCloud(laserCloudCornerMap);
    cropBoxFilter.filter(*tmpCorner);


    laserCloudSurfMap = rgbds(tmpSurf, map_resolution*2 );
    laserCloudCornerMap = rgbds(tmpCorner, map_resolution);
    // downSizeFilterSurf.setInputCloud(tmpSurf);
    // downSizeFilterSurf.filter(*laserCloudSurfMap);
    // downSizeFilterEdge.setInputCloud(tmpCorner);
    // downSizeFilterEdge.filter(*laserCloudCornerMap);
    extractstablepoint(laserCloudSurfMap, k_new_surf, theta_p_surf, theta_max_surf);
    extractstablepoint(laserCloudCornerMap, k_new_edge, theta_p_edge, theta_max_edge);
    for(int i = 0;i < laserCloudSurfMap->points.size(); i++){
        if(laserCloudSurfMap->points[i].r > 250)
            laserCloudSurfMap->points[i].r = 255;
        else
            laserCloudSurfMap->points[i].r +=2;
    }
        
    for(int i = 0;i < laserCloudCornerMap->points.size(); i++)
        if(laserCloudCornerMap->points[i].r > 250)
            laserCloudCornerMap->points[i].r = 255;
        else
            laserCloudCornerMap->points[i].r +=2;

}

void OdomEstimationClass::getMap(pcl::PointCloud<PointType>::Ptr& laserCloudMap){
    
    *laserCloudMap += *laserCloudSurfMap;
    *laserCloudMap += *laserCloudCornerMap;
}

OdomEstimationClass::OdomEstimationClass(){

}
