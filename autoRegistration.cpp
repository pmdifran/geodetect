#include "autoRegistration.h"
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include<pcl/registration/default_convergence_criteria.h>

Eigen::Matrix4f globalRegistration(GeoDetection& reference, GeoDetection& source, const float& radius)
{
	//Check that the clouds have normals
	if (!reference.hasNormals())
	{

		reference.computeNormals(1.0); //****************************************************************
	}

	if (!source.hasNormals())
	{
		source.computeNormals(1.0); ////****************************************************************
	}

	//Compute ISS keypoints
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_reference = reference.getKeyPoints();
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_source = source.getKeyPoints();

	//Compute fast point feature histograms at keypoints
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_reference = reference.getFPFH(keypoints_reference);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_source = source.getFPFH(keypoints_source);

	//Compute keypoint correspondences
	pcl::CorrespondencesPtr correspondences (new pcl::Correspondences);
	pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> estimator;
	estimator.setInputTarget(fpfh_reference);
	estimator.setInputSource(fpfh_source);
	estimator.determineCorrespondences(*correspondences);

	std::cout << ":: Number of initial fpfh correspondences: " << correspondences->size() << std::endl;

	//Random Sample Consensus (RANSAC) -based correspondence rejection. 
	pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> rejector;
	rejector.setInputTarget(keypoints_reference);
	rejector.setInputSource(keypoints_source);
	rejector.setMaximumIterations(1000000);
	rejector.setRefineModel(true);
	rejector.setInlierThreshold(0.5);
	rejector.setInputCorrespondences(correspondences);
	
	rejector.getCorrespondences(*correspondences); //computes ransac --> gets best transforation
	Eigen::Matrix4f transformation = rejector.getBestTransformation();
	
	std::cout << ":: Number of inlier fpfh correpondences: " << correspondences->size() << std::endl;
	std::cout << ":: Transformation computed\n" << std::endl;

	double mse = 0;
	for (const auto& correspondence : *correspondences)
		mse += correspondence.distance;
	mse /= (double)(correspondences->size());

	std::cout << "  --> Mean square error: " << mse << std::endl;

	return transformation;
}
