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

namespace geodetection
{
	//Transforms source cloud using a global registration determined with keypoint fast point feature histogram correspondences.
	//Default norml radius = 1.0. Not used if the cloud already has normals.
	Eigen::Matrix4f getGlobalRegistration(geodetection::Cloud& reference,
		geodetection::Cloud& source, float radius /* = 1.0 */)
	{
		GD_TITLE("Auto Registration --Global");
		Timer timer;

		if (!reference.hasCloud()) { GD_ERROR("Reference does not have a cloud"); }
		if (!reference.hasNormals()) { reference.updateNormalsRadiusSearch(radius); };

		if (!source.hasCloud()) { GD_ERROR("Source does not have a cloud"); }
		if (!source.hasNormals()) { source.updateNormalsRadiusSearch(radius); }

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

		GD_TRACE(":: Number of inlier fpfh correpondences: {0}\n", correspondences->size());
		GD_WARN("--> Transformation computed in: {0} ms: \n", timer.getDuration());
		std::cout << std::setprecision(16) << std::fixed <<
			"Transformation: \n" << transformation << '\n' << std::endl;

		//Compute the MSE of the global registration
		pcl::transformPointCloud(*src_keypoints, *src_keypoints, transformation);

		double mse = 0;

#pragma omp parallel for reduction(+: mse)
		for (int64_t i = 0; i < remaining_correspondences->size(); i++)
		{
			pcl::Correspondence& corr = (*remaining_correspondences)[i];
			float distance = pcl::euclideanDistance(src_keypoints->at(corr.index_query),
				ref_keypoints->at(corr.index_match));

			mse += distance;
		}

		mse /= (double)(remaining_correspondences->size());

		GD_WARN("--> Mean square error: {0}", mse);
		GD_TRACE("NOTE: MSE may be higher due to a subsampled input\n");

		//Apply the transformation to the source geodetection object.
		source.applyTransformation(transformation);
		return transformation;
	}

	//Transforms source cloud using a global registration determined with keypoint fast point feature histogram correspondences.
	//Default norml radius = 1.0. Not used if the cloud already has normals.
	//This overload uses RegistrationCloud: which preserves the keypoints and fast point feature histograms,
	//--> so that they are not recalculated in a batched registration pipeline. 
	Eigen::Matrix4f getGlobalRegistration(geodetection::RegistrationCloud& reference,
		geodetection::Cloud& source, float radius /* = 1.0 */)
	{
		GD_TITLE("Auto Registration --Global");
		Timer timer;

		if (!reference.hasCloud()) { GD_ERROR("Reference does not have a cloud"); }
		if (!reference.hasNormals()) { reference.updateNormalsRadiusSearch(radius); };
		if (!reference.hasKeypoints()) { reference.updateKeypoints(); }
		if (!reference.hasFPFH()) { reference.updateFPFH(); }

		if (!source.hasCloud()) { GD_ERROR("Source does not have a cloud"); }
		if (!source.hasNormals()) { source.updateNormalsRadiusSearch(radius); }

		//Get pointers to reference global registration data
		auto ref_keypoints = reference.keypoints();
		auto ref_fpfh = reference.fpfh();

		//Compute source registration data
		pcl::PointCloud<pcl::PointXYZ>::Ptr src_keypoints = source.getKeyPoints();
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

		GD_TRACE(":: Number of inlier fpfh correpondences: {0}\n", correspondences->size());
		GD_WARN("--> Transformation computed in: {0} ms: \n", timer.getDuration());
		std::cout << std::setprecision(16) << std::fixed <<
			"Transformation: \n" << transformation << '\n' << std::endl;

		//Compute the MSE of the global registration
		pcl::transformPointCloud(*src_keypoints, *src_keypoints, transformation);

		double mse = 0;
		for (const auto& corr : *remaining_correspondences)
		{
			float distance = pcl::euclideanDistance(src_keypoints->at(corr.index_query),
				ref_keypoints->at(corr.index_match));

			mse += distance;
		}

		mse /= (double)(remaining_correspondences->size());

		GD_WARN("--> Mean square error: {0}", mse);
		GD_TRACE("NOTE: MSE may be higher due to a subsample input\n");

		//Apply the transformation to the source geodetection object.
		source.applyTransformation(transformation);
		return transformation;
	}

	//Transforms source cloud using a fine generalized ICP registration (i.e. plane-to-plane).
	Eigen::Matrix4f getICPRegistration(geodetection::Cloud& reference,
		geodetection::Cloud& source, float radius /* = 1.0 */)
	{
		GD_TITLE("Auto Registration --ICP");
		Timer timer;

		if (!reference.hasCloud()) { GD_ERROR("Reference does not have a cloud"); }
		if (!reference.hasNormals()) { reference.updateNormalsRadiusSearch(radius); };

		if (!source.hasCloud()) { GD_ERROR("Source does not have a cloud"); }
		if (!source.hasNormals()) {
			source.updateNormalsRadiusSearch(radius);
		}

		pcl::GeneralizedIterativeClosestPoint <pcl::PointXYZ, pcl::PointXYZ> gicp;
		gicp.setMaxCorrespondenceDistance(radius);

		gicp.setInputSource(source.cloud());
		gicp.setInputTarget(reference.cloud());

		gicp.setCorrespondenceRandomness(10);
		gicp.setMaximumIterations(100);
		gicp.setSearchMethodSource(source.kdtree());
		gicp.setSearchMethodTarget(reference.kdtree());

		gicp.align(*source.cloud()); //**The transformation is applied to the cloud here!

		Eigen::Matrix4f transformation = gicp.getFinalTransformation();

		// update the matrix without transforming the data again.
		source.updateTransformation(transformation);
		
		GD_WARN("ICP computed in: {0} ms\n", timer.getDuration());
		std::cout << std::setprecision(16) << std::fixed <<
			"Transformation: \n" << transformation << '\n' << std::endl;

		GD_WARN("--> Mean square error: {0}\n", gicp.getFitnessScore());
		GD_TRACE("NOTE: MSE may be higher due to a subsample input\n");

		return transformation;
	}

}