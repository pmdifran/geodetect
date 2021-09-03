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

namespace GeoDetection
{
	Eigen::Matrix4f getGlobalRegistration(GeoDetection::Cloud& reference,
		GeoDetection::Cloud& source, const float& radius, const float& subres)
	{
		GD_TITLE("Auto Registration --Global");
		auto start = GeoDetection::Time::getStart();

		if (!reference.hasNormals()) {
			reference.setNormals(reference.getNormalsRadiusSearch(1.0));
		}

		if (!source.hasNormals()) {
			source.setNormals(source.getNormalsRadiusSearch(1.0));
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

		GD_TRACE(":: Number of inlier fpfh correpondences: {0}\n", correspondences->size());
		GD_WARN("--> Transformation computed in: {0} ms: \n", GeoDetection::Time::getDuration(start));
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

		//Apply the transformation to the source GeoDetection object.
		source.applyTransformation(transformation);
		return transformation;
	}

	Eigen::Matrix4f getICPRegistration(GeoDetection::Cloud& reference,
		GeoDetection::Cloud& source, const float radius)
	{
		GD_TITLE("Auto Registration --ICP");
		auto start = GeoDetection::Time::getStart();

		//Compute the normals if they are not already computed.
		if (!reference.hasNormals()) {
			reference.setNormals(reference.getNormalsRadiusSearch(radius));
		}

		if (!source.hasNormals()) {
			source.setNormals(source.getNormalsRadiusSearch(radius));
		}

		pcl::GeneralizedIterativeClosestPoint <pcl::PointXYZ, pcl::PointXYZ> gicp;
		gicp.setMaxCorrespondenceDistance(radius);

		gicp.setInputSource(source.cloud());
		gicp.setInputTarget(reference.cloud());

		gicp.setCorrespondenceRandomness(10);
		gicp.setMaximumIterations(100);
		gicp.setSearchMethodSource(source.tree());
		gicp.setSearchMethodTarget(reference.tree());

		gicp.align(*source.cloud()); //**The transformation is applied to the cloud here!

		Eigen::Matrix4f transformation = gicp.getFinalTransformation();

		// update the matrix without transforming the data again.
		source.updateTransformation(transformation);
		
		GD_WARN("ICP computed in: {0} ms\n", GeoDetection::Time::getDuration(start));
		std::cout << std::setprecision(16) << std::fixed <<
			"Transformation: \n" << transformation << '\n' << std::endl;

		GD_WARN("--> Mean square error: {0}\n", gicp.getFitnessScore());
		GD_TRACE("NOTE: MSE may be higher due to a subsample input\n");

		return transformation;
	}
}