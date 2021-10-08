#pragma once
#include "GeoDetection.h"

namespace geodetection
{
	//Registration cloud for storing the features computed in auto registration
	//Typically a reference cloud which is used numerous times for registration in a batch processing
	class RegistrationCloud : public Cloud
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr m_keypoints;
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr m_fpfh;

	public: 
		RegistrationCloud()
			: Cloud(),
			m_keypoints (new pcl::PointCloud<pcl::PointXYZ>),
			m_fpfh (new pcl::PointCloud<pcl::FPFHSignature33>)
		{
			GD_CORE_TRACE("--> Registration members added to empty GeoDetection Cloud");
		}

		inline pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints() { return m_keypoints; }
		inline pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh() { return m_fpfh; }

		inline bool hasKeypoints() { return m_keypoints->size() > 0; }
		inline bool hasFPFH() { return m_fpfh->size() > 0; }

		/**
		* Computes intrinsic shape signature keypoints and stores it in the Registration Cloud as a member.
		* @param[in] salient radius: Spherical neighborhood (i.e. scale) at which we determine the largest point variations within.
		* @paramin[in] non_max_radius: Radius for the application of the non maxima supression algorithm.
		* @param[in] min_neighbors: The minimum number of neighbors that has to be found while applying the non maxima suppression algorithm
		* @param[in] cloud: Point cloud (possible subsampled search_surface) used to compute the ISS keypoints.
		* @param[in] search_surface: Point cloud used to compute the ISS signatures at salient_radius scales.
		* @param[in] tree: Search tree corresponding to search_surface.
		* @return shared pointer to a pcl point cloud of ISS keypoints.
		*/
		void updateISSKeyPoints(float salient_radius, float non_max_radius, int min_neighbors,
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr search_surface,
			pcl::search::KdTree<pcl::PointXYZ>::Ptr search_surface_tree,
			float max_eigenratio21 = 0.975f, float max_eigenratio32 = 0.975f)
		{
			m_keypoints = this->Cloud::getISSKeyPoints(salient_radius, non_max_radius, min_neighbors, cloud, search_surface, search_surface_tree,
				max_eigenratio21, max_eigenratio32);
		}

		/**
		* Computes intrinsic shape signature keypoints.
		* @param[in] salient radius: Spherical neighborhood (i.e. scale) at which we determine the largest point variations within.
		* @paramin[in] non_max_radius: Radius for the application of the non maxima supression algorithm.
		* @param[in] min_neighbors: The minimum number of neighbors that has to be found while applying the non maxima suppression algorithm
		* @param[in] cloud: Point cloud (possible subsampled search_surface) used to compute the ISS keypoints.
		* @param[in] search_surface: Point cloud used to compute the ISS signatures at salient_radius scales.
		* @param[in] tree: Search tree corresponding to search_surface.
		* @return shared pointer to a pcl point cloud of ISS keypoints.
		*/
		pcl::PointCloud<pcl::PointXYZ>::Ptr getISSKeyPoints(float salient_radius, float non_max_radius, int min_neighbors,
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr search_surface,
			pcl::search::KdTree<pcl::PointXYZ>::Ptr search_surface_tree,
			float max_eigenratio21 = 0.975f, float max_eigenratio32 = 0.975f)
		{
			if (!this->hasKeypoints())
			{
				this->updateISSKeyPoints(salient_radius, non_max_radius, min_neighbors, cloud, search_surface, search_surface_tree,
					max_eigenratio21, max_eigenratio32);
			}
			return m_keypoints;
		}

		/**
		* Computes intrinsic shape signature keypoints. Uses member point cloud for search surface, tree, and normals.
		* @param[in] cloud: subsampled cloud at which keypoint calculations are done for.
		* @param[in] salient radius: Spherical neighborhood (i.e. scale) at which we determine the largest point variations within.
		* @paramin[in] non_max_radius: Radius for the application of the non maxima supression algorithm.
		* @param[in] min_neighbors: The minimum number of neighbors that has to be found while applying the non maxima suppression algorithm
		* @return shared pointer to a pcl point cloud of ISS keypoints.
		*/
		pcl::PointCloud<pcl::PointXYZ>::Ptr getISSKeyPoints(float salient_radius, float non_max_radius, int min_neighbors,
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float max_eigenratio21 = 0.975f, float max_eigenratio32 = 0.975f)
		{
			this->getISSKeyPoints(salient_radius, non_max_radius, min_neighbors, cloud, max_eigenratio21, max_eigenratio32);
		}

		/**
		* Computes intrinsic shape signature keypoints. Uses member point cloud for keypoint calculations, search surface, tree, and normals.
		* @param[in] salient radius: Spherical neighborhood (i.e. scale) at which we determine the largest point variations within.
		* @paramin[in] non_max_radius: Radius for the application of the non maxima supression algorithm.
		* @param[in] min_neighbors: The minimum number of neighbors that has to be found while applying the non maxima suppression algorithm
		* @return shared pointer to a pcl point cloud of ISS keypoints.
		*/
		pcl::PointCloud<pcl::PointXYZ>::Ptr getISSKeyPoints(float salient_radius, float non_max_radius, int min_neighbors,
			float max_eigenratio21 = 0.975f, float max_eigenratio32 = 0.975f)
		{
			this->getISSKeyPoints(salient_radius, non_max_radius, min_neighbors, this->cloud(), max_eigenratio21, max_eigenratio32);
		}

		/**
		* Computes fast point feature histograms. and stores it as the member variable.
		* @param keypoints: Shared pointer to a pcl point cloud containing keypoints, at which fpf histograms are calculated for.
		* @param radius: Scale at to compute the histograms.
		* @param search_surface: point cloud to use when computing histograms.
		* @param search_surface_tree: kdtree for the search_surface.
		* @param search_surface_normals: normals of search_surface point cloud (sizes must correspond)
		* @return shared pointer to point cloud with fast point feature histograms.
		*/
		void updateFPFH(const pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints, float radius,
			pcl::PointCloud<pcl::PointXYZ>::Ptr search_surface, pcl::search::KdTree<pcl::PointXYZ>::Ptr search_surface_tree,
			pcl::PointCloud<pcl::Normal>::Ptr search_surface_normals)
		{
			m_fpfh = this->Cloud::getFPFH(keypoints, radius, search_surface, search_surface_tree, search_surface_normals);
		}

		/**
		* Computes fast point feature histograms.
		* @param keypoints: Shared pointer to a pcl point cloud containing keypoints, at which fpf histograms are calculated for.
		* @param radius: Scale at to compute the histograms.
		* @param search_surface: point cloud to use when computing histograms.
		* @param search_surface_tree: kdtree for the search_surface.
		* @param search_surface_normals: normals of search_surface point cloud (sizes must correspond)
		* @return shared pointer to point cloud with fast point feature histograms.
		*/
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr getFPFH(const pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints, float radius,
			pcl::PointCloud<pcl::PointXYZ>::Ptr search_surface, pcl::search::KdTree<pcl::PointXYZ>::Ptr search_surface_tree,
			pcl::PointCloud<pcl::Normal>::Ptr search_surface_normals)
		{
			assert(this->hasKeypoints());
			if (!this->hasFPFH())
			{
				this->updateFPFH(keypoints, radius, search_surface, search_surface_tree, search_surface_normals);
			}
			return m_fpfh;
		}

		/**
		* Computes fast point feature histograms. Uses member variables for cloud, search tree and normals.
		* @param keypoints: Shared pointer to a pcl point cloud containing keypoints, at which fpf histograms are calculated for.
		* @param radius: Scale at to compute the histograms.
		* @return shared pointer to point cloud with fast point feature histograms.
		*/
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr getFPFH(const pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints, float radius)
		{
			return this->getFPFH(keypoints, radius, this->cloud(), this->kdtree(), this->normals());
		}

	};
}