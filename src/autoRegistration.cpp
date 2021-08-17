#include "autoRegistration.h"

#include <iomanip>

//for MSE
#include <pcl/common/distances.h>

//for globalRegistration
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include<pcl/registration/default_convergence_criteria.h>

//for icpRegistration
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/registration/gicp.h>
#include <pcl/features/from_meshes.h>

Eigen::Matrix4f globalRegistration(GeoDetection::Cloud& reference, 
	GeoDetection::Cloud& source, const float& radius, const float& subres)
{
	GD_INFO("----------------------------------------------\
		Auto Registration --Global...\n");

	if (!reference.hasNormals()) {
		reference.m_normals = reference.getNormals(1.0);
	}

	if (!source.hasNormals()) {
		source.m_normals = source.getNormals(1.0);
	}

	//Compute ISS keypoints
	pcl::PointCloud<pcl::PointXYZ>::Ptr ref_keypoints = reference.getKeyPoints();
	pcl::PointCloud<pcl::PointXYZ>::Ptr src_keypoints = source.getKeyPoints();

	//Compute fast point feature histograms at keypoints
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr ref_fpfh = reference.getFPFH(ref_keypoints);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr src_fpfh = source.getFPFH(src_keypoints);

	//Compute keypoint correspondences
	pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);

	pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> estimator;
	estimator.setInputTarget(ref_fpfh);
	estimator.setInputSource(src_fpfh);
	estimator.determineCorrespondences(*correspondences);

	GD_TRACE(":: Number of initial fpfh correspondences: {0}", correspondences->size());

	//Random Sample Consensus (RANSAC) -based correspondence rejection. 
	pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> ransac_rejector;
	pcl::CorrespondencesPtr remaining_correspondences(new pcl::Correspondences);

	ransac_rejector.setInputTarget(ref_keypoints);
	ransac_rejector.setInputSource(src_keypoints);
	ransac_rejector.setMaximumIterations(1000000);
	ransac_rejector.setRefineModel(true);
	ransac_rejector.setInlierThreshold(0.5);
	
	ransac_rejector.getRemainingCorrespondences(*correspondences, *remaining_correspondences);
	Eigen::Matrix4f transformation = ransac_rejector.getBestTransformation();

	GD_TRACE(":: Number of inlier fpfh correpondences: {0}", correspondences->size());
	GD_INFO("--> Transformation computed: \n");
	std::cout << std::setprecision(16) << std::fixed << transformation << '\n' << std::endl;

	//Compute the MSE of the global registration
	pcl::transformPointCloud(*src_keypoints, *src_keypoints, transformation);

	double mse = 0;
	for (const auto& corr : *remaining_correspondences)
	{
		float distance = pcl::euclideanDistance(src_keypoints->at(corr.index_query),
			ref_keypoints->at(corr.index_match));

		std::cout << distance << std::endl;
		mse += distance;
	}

	mse /= (double)(remaining_correspondences->size());

	GD_TRACE("--> Mean square error: {0}", mse);
	GD_WARN("NOTE: MSE may be higher due to a subsample input");

	//Apply the transformation to the source GeoDetection object.
	source.applyTransformation(transformation);
	return transformation;
}

Eigen::Matrix4f icpRegistration(GeoDetection::Cloud& reference, 
	GeoDetection::Cloud& source, const float radius)
{
	std::cout << "----------------------------------------------" << std::endl;
	std::cout << "Auto Registration --ICP...\n" << std::endl;

	//Compute the normals if they are not already computed.
	if (!reference.hasNormals()) {
		reference.m_normals = reference.getNormals(radius);
	}

	if (!source.hasNormals()) {
		source.m_normals = source.getNormals(radius);
	}

	pcl::GeneralizedIterativeClosestPoint <pcl::PointXYZ, pcl::PointXYZ> gicp;
	gicp.setMaxCorrespondenceDistance(radius);

	gicp.setInputSource(source.m_cloud);
	gicp.setInputTarget(reference.m_cloud);

	gicp.setCorrespondenceRandomness(10);
	gicp.setMaximumIterations(100);
	gicp.setSearchMethodSource(source.m_kdtree);
	gicp.setSearchMethodTarget(reference.m_kdtree);

	gicp.align(*source.m_cloud);
	
	Eigen::Matrix4f transformation = gicp.getFinalTransformation();
	return transformation;

}