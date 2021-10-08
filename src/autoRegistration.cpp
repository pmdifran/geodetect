#include "autoRegistration.h"
#include "features.h"

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

#include <pcl/keypoints/iss_3d.h>

double getMeanSquareError(pcl::CorrespondencesPtr remaining_correspondences, pcl::PointCloud<pcl::PointXYZ>::Ptr src_keypoints,
	pcl::PointCloud<pcl::PointXYZ>::Ptr ref_keypoints)
{
	//Compute the MSE of the global registration
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

	return mse;
}

namespace geodetection
{
	//Transforms source cloud using a global registration determined with keypoint fast point feature histogram correspondences.
	//Default norml radius = 1.0. Not used if the cloud already has normals.
	//@TODO: Break this function up into smaller more readable functions.
	Eigen::Matrix4f getGlobalRegistration(geodetection::Cloud& reference,
		geodetection::Cloud& source, float normal_scale, float scale_coefficient)
	{
		GD_TITLE("Auto Registration --Global");
		Timer timer;

		//Compute constants based from scale coefficient
		//Keypoints
		float salient_radius = pow(scale_coefficient, 1.4f) + 4.0f;
		float non_max_radius = pow(scale_coefficient, 1.3f) + 2.0f;
		int min_nbrs = 5;
		//Fast point feature histograms
		float fpfh_scale = pow(salient_radius, 1.1f);
		//RANSAC 
		float inlier_threshold = pow(scale_coefficient, 1.1f);

		//Compute normals
		reference.updateNormalsRadiusSearch(normal_scale);
		source.updateNormalsRadiusSearch(normal_scale);

		//Currently fixing a bug in PCL which allows us to use a subset of points to compute. 
		//Compute subsampled cloud for keypoints
		auto ref_down = reference.cloud();
		auto src_down = source.cloud();

		//Replace with this after.
		//auto ref_down = reference.getVoxelDownSample(scale_coefficient);
		//auto src_down = source.getVoxelDownSample(scale_coefficient);

		//Compute ISS keypoints
		auto ref_keypoints = reference.getISSKeyPoints(salient_radius, non_max_radius, min_nbrs, ref_down, 0.975f, 0.975f);
		auto src_keypoints = source.getISSKeyPoints(salient_radius, non_max_radius, min_nbrs, src_down, 0.975f, 0.975f);

		//Compute fast point feature histograms.
		auto ref_fpfh = reference.getFPFH(ref_keypoints, fpfh_scale);
		auto src_fpfh = source.getFPFH(src_keypoints, fpfh_scale);

		//Compute feature correspondences
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
		ransac_rejector.setRefineModel(false);
		ransac_rejector.setInlierThreshold(inlier_threshold);

		ransac_rejector.getRemainingCorrespondences(*correspondences, *remaining_correspondences);
		Eigen::Matrix4f transformation = ransac_rejector.getBestTransformation();

		GD_TRACE(":: Number of inlier fpfh correpondences: {0}\n", correspondences->size());
		GD_WARN("--> Transformation computed in: {0} ms: \n", timer.getDuration());
		
		//Apply the transformation to the source geodetection object.
		source.applyTransformation(transformation);
		source.printTransformation();

		//Calculate MSE. (Must transform keypoints first)
		pcl::transformPointCloud(*src_keypoints, *src_keypoints, transformation);
		GD_WARN("--> Mean square error: {0}", getMeanSquareError(remaining_correspondences, src_keypoints, ref_keypoints));
		GD_TRACE("NOTE: MSE may be higher due to a subsampled input\n");

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

	void computeAutoRegistration(Cloud source, Cloud reference, float subsample_scale)
	{
		//Subsample for calculating 
	}

}