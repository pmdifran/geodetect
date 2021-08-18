//GeoDetection
#include "GeoDetection.h"

#include <omp.h> //for Open MP

//stdlib
#include <chrono>
#include <fstream>
#include <iomanip> //for std::setprecision
#include <algorithm> //for std::remove

//for transformation
#include <pcl/common/impl/transforms.hpp>

//feature estimation
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/keypoints/iss_3d.h>

//filters
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>

namespace GeoDetection
{
	void
		Cloud::removeNaN()
	{
		GD_CORE_TRACE(":: Removing NaN from Cloud.\n");

		m_cloud->is_dense = false;
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(*m_cloud, *m_cloud, indices);
		m_cloud->is_dense = true;
	}

	void
		Cloud::writeRT(const char* fname)
	{
		GD_CORE_TRACE(":: Writing RT file...\n");

		std::ofstream ofs;
		ofs.open(fname, std::ios::out | std::ios::binary | std::ios::trunc);
		ofs << std::fixed << std::setprecision(16) << m_transformation << std::endl;
		ofs.close();
	}

	void
		Cloud::setKdTrees()
	{
		GD_CORE_TRACE(":: Constructing Search Trees");
		auto start = GeoDetection::Time::getStart();

		m_kdtreeFLANN->setInputCloud(m_cloud);
		m_kdtree->setInputCloud(m_cloud);

		GD_CORE_WARN("--> KdTree construction time: {0} ms\n",
			GeoDetection::Time::getDuration(start));
	}

	std::vector<float> Cloud::getResolution(int num_neighbors)
	{
		GD_CORE_TRACE(":: Computing cloud resolution...");
		GD_CORE_TRACE("--> k-nearest neighbors used for distances: {0}", num_neighbors);
		auto start = GeoDetection::Time::getStart();

		std::vector<float> resolution(m_cloud->size());
		unsigned int k = num_neighbors + 1; //The first nearest neighbor is the query point itself --> use for searches.

		double avg_resolution = 0; //cloud-wide average resolution

#pragma omp parallel for reduction(+: avg_resolution)
		for (int64_t i = 0; i < m_cloud->size(); i++)
		{
			double local_resolution = 0.0; //per-point localized resolution determined with number of neighbors (num_neighbors)
			std::vector<int> indices(k);
			std::vector<float> sq_distances(k);

			m_kdtreeFLANN->nearestKSearch(i, k, indices, sq_distances);
			for (int j = 1; j < k; j++)
			{
				local_resolution += sq_distances[j];
			}

			local_resolution /= (double)num_neighbors;
			resolution[i] = local_resolution;
			avg_resolution += local_resolution; //thread-safe with omp reduction
		}

		avg_resolution = sqrt(avg_resolution / (double)m_cloud->size());
		m_resolution = avg_resolution;

		GD_INFO("--> Point cloud resolution: {0} meters", m_resolution);
		GD_CORE_WARN("--> Resolution estimation time: {0} ms\n",
			GeoDetection::Time::getDuration(start));

		return resolution;
	}

	pcl::PointCloud<pcl::Normal>::Ptr
		Cloud::getNormals(float nrad)
	{
		GD_CORE_TRACE(":: Computing point cloud normals...\n:: Normal scale {0}", nrad);
		GD_CORE_WARN(":: # Threads automatically set to the number of cores: {0}",
			omp_get_num_procs());
		auto start = GeoDetection::Time::getStart();

		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		normals->reserve(m_cloud->size());

		pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> calcnormals;

		calcnormals.setInputCloud(m_cloud);
		calcnormals.setSearchMethod(m_kdtree);
		calcnormals.setViewPoint(m_view[0], m_view[1], m_view[2]); //0,0,0 as default
		calcnormals.setRadiusSearch(nrad);
		calcnormals.setNumberOfThreads(omp_get_num_procs());	
		calcnormals.compute(*normals);

		GD_CORE_WARN("--> Normal calculation time: {0} ms\n",
			GeoDetection::Time::getDuration(start));

		return normals;
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr
		Cloud::getVoxelDownSample(float voxel_size)
	{
		GD_CORE_TRACE(":: Creating downsampled cloud with voxels...\
			\n--> voxel filter size: {0}", voxel_size);
		auto start = GeoDetection::Time::getStart();

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_down(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::VoxelGrid<pcl::PointXYZ> grid;
		grid.setInputCloud(m_cloud);
		grid.setLeafSize(voxel_size, voxel_size, voxel_size);
		grid.filter(*cloud_down);
		
		GD_CORE_WARN("-->   Full cloud size: {0}\n --> Downsampled cloud size: {1}",
			m_cloud->size(), cloud_down->size());

		GD_CORE_WARN("--> Downsample time: {0} ms\n",
			GeoDetection::Time::getDuration(start));

		return cloud_down;
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr
		Cloud::getDistanceDownSample(float distance)
	{
		GD_CORE_TRACE(":: Subsampling cloud by distance...\
			\n--> Distance: {0}", distance);
		auto start = GeoDetection::Time::getStart();

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_down(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
		inliers->indices.reserve(m_cloud->size()); //conservatively reserve space for entire cloud indices. 

		std::vector<char> markers;
		markers.resize(m_cloud->size(), -1); //set to -1 by default **remember bool(-1) --> True**

		for (int64_t i = 0; i < m_cloud->size(); i++)
		{
			if (markers[i])
			{
				std::vector<int> IDRadiusSearch;
				std::vector<float> DistanceRadiusSearch;

				m_kdtreeFLANN->radiusSearch(m_cloud->points[i], distance, IDRadiusSearch, DistanceRadiusSearch);

				if (IDRadiusSearch.size() > 1) {
					for (std::vector<int>::iterator it = IDRadiusSearch.begin() + 1; it != IDRadiusSearch.end(); ++it) {
						markers[*it] = false;
					}
				}

				inliers->indices.push_back(i);
			}
		}

		size_t size_original = m_cloud->size();
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(m_cloud);
		extract.setIndices(inliers);
		extract.filter(*cloud_down);

		GD_CORE_WARN("-->   Full cloud size: {0}\n --> Downsampled cloud size: {1}",
			m_cloud->size(), cloud_down->size());

		GD_CORE_WARN("--> Downsample time: {0} ms\n",
			GeoDetection::Time::getDuration(start));

		return cloud_down;
	}

	void
		Cloud::applyTransformation(const Eigen::Matrix4f& transformation)
	{
		GD_CORE_TRACE(":: Applying transformation...");
		auto start = GeoDetection::Time::getStart();

		pcl::transformPointCloud(*m_cloud, *m_cloud, transformation);
		Eigen::Matrix4d temp = transformation.cast<double>(); //using doubles for more accurate arithmitic
		m_transformation = temp * m_transformation; //eigen matrix multiplication

		GD_CORE_WARN("--> Transformation time: {0} ms\n",
			GeoDetection::Time::getDuration(start));
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr
		Cloud::getKeyPoints()
	{
		GD_CORE_TRACE(":: Computing intrinsic shape signature keypoints...");
		auto start = GeoDetection::Time::getStart();

		pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss_detector;

		iss_detector.setSearchMethod(m_kdtree);

		//Revisit parameters. Automatic selection using scale, subsample scale?
		iss_detector.setSalientRadius(0.5); 
		iss_detector.setNonMaxRadius(1.5); 
		iss_detector.setMinNeighbors(5);

		iss_detector.setInputCloud(m_cloud);

		iss_detector.setThreshold21(0.975); 
		iss_detector.setThreshold32(0.975); 
		iss_detector.setNumberOfThreads(omp_get_num_procs());
		iss_detector.compute(*keypoints);
		
		GD_CORE_WARN("--> Keypoint calculation time: {0} ms\n",
			GeoDetection::Time::getDuration(start));

		return keypoints;
	}

	//Enable AVX in Properties --> C/C++ --> Enable Enhanced Instruction Set --> /arch:AVX
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr
		Cloud::getFPFH(const pcl::PointCloud<pcl::PointXYZ>::Ptr& keypoints)
	{
		GD_CORE_TRACE("Computing fast point feature histograms...");
		auto start = GeoDetection::Time::getStart();

		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh(new pcl::PointCloud<pcl::FPFHSignature33>);
		pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> computefpfh;

		computefpfh.setNumberOfThreads(omp_get_num_procs());
		computefpfh.setInputCloud(keypoints);
		computefpfh.setInputNormals(m_normals);
		computefpfh.setSearchSurface(m_cloud);
		computefpfh.setSearchMethod(m_kdtree);
		computefpfh.setRadiusSearch(3.0); 

		computefpfh.compute(*fpfh);

		GD_CORE_WARN("--> FPFH calculation time: {0} ms\n",
			GeoDetection::Time::getDuration(start));

		return fpfh; 
	}

}