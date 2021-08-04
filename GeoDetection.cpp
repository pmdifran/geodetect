#include "GeoDetection.h"
#include <chrono>
#include <fstream>
#include <iomanip> //for std::setprecision
#include <algorithm> //for std::remove
#include <omp.h> //for Open MP

#include <pcl/common/impl/transforms.hpp>

#include <pcl/features/fpfh_omp.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <pcl/keypoints/iss_3d.h>

void 
GeoDetection::removeNaN()
{
	m_cloud->is_dense = false;
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*m_cloud, *m_cloud, indices);
	m_cloud->is_dense = true;
}

void
GeoDetection::writeRT(const char* fname)
{
	std::cout << "Writing RT file\n" << std::endl;

	std::ofstream ofs;
	ofs.open(fname, std::ios::out | std::ios::binary | std::ios::trunc);
	ofs << std::fixed << std::setprecision(16) << m_transformation << std::endl;
	ofs.close();

}

void
GeoDetection::setKdTrees()
{
	std::cout << ":: Computing KdTrees..." << std::endl;
	auto start = std::chrono::steady_clock::now();

	m_kdtreeFLANN->setInputCloud(m_cloud);
	m_kdtree->setInputCloud(m_cloud);

	auto timer = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start);
	std::cout << "--> KdTree construction time: " << timer.count() << " seconds\n" << std::endl;
}

std::vector<float> GeoDetection::getResolution(int num_neighbors)
{
	std::cout << "Computing cloud resolution..." << std::endl;
	std::cout << ":: k-nearest neighbors used for distances: " << num_neighbors << std::endl;
	auto start = std::chrono::steady_clock::now();

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
	auto timer = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start);

	std::cout << "Point cloud resolution: " << m_resolution << " meters" << std::endl;
	std::cout << "--> calculation time: "<< timer.count() << " seconds\n" << std::endl;

	return resolution;	
}

pcl::PointCloud<pcl::Normal>::Ptr
GeoDetection::getNormals(float nrad)
{
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	normals->reserve(m_cloud->size());

	std::cout << "Computing point cloud normals..." << std::endl;
	std::cout << ":: Normal scale:  " << nrad << std::endl;
	auto start = std::chrono::steady_clock::now();

	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> calcnormals;

	calcnormals.setInputCloud(m_cloud);
	calcnormals.setSearchMethod(m_kdtree);
	calcnormals.setViewPoint(m_view[0], m_view[1], m_view[2]); //0,0,0 as default
	calcnormals.setRadiusSearch(nrad);
	calcnormals.setNumberOfThreads(omp_get_num_procs());
	
	std::cout << ":: Threads automatically set to number of cores.\n" << ":: --> Number of threads: " << omp_get_num_procs() << std::endl;

	calcnormals.compute(*normals);
	auto timer = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start);

	std::cout << "Normals computed" << std::endl;
	std::cout << "--> calculation time: " << timer.count() << " seconds\n" << std::endl;

	return normals;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr 
GeoDetection::getVoxelDownSample(float voxel_size)
{
	std::cout << "Creating downsampled cloud with voxels..." << std::endl;
	std::cout << ":: voxel fitler size : " << voxel_size << std::endl;
	auto start = std::chrono::steady_clock::now();

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_down(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> grid;
	grid.setInputCloud(m_cloud);
	grid.setLeafSize(voxel_size, voxel_size, voxel_size);
	grid.filter(*cloud_down);
	auto timer = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start);
	
	std::cout << "::  Full cloud size: " << m_cloud->size() << std::endl;
	std::cout << ":: Downsampled size: " << cloud_down->size() << std::endl;
	std::cout << "--> calculation time: " << timer.count() << " seconds\n" << std::endl;
	return cloud_down;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr 
GeoDetection::getDistanceDownSample(float distance)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_down(new pcl::PointCloud<pcl::PointXYZ>);
	std::cout << "Subsampling cloud by distance: " << distance << " meters..." << std::endl;
	auto start = std::chrono::steady_clock::now();

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

	auto timer = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start);
	std::cout << ":: Initial cloud size: " << size_original << std::endl;
	std::cout << "::   Downsampled size: " << cloud_down->size() << std::endl;
	float temp = 100 - (size_original - cloud_down->size()) / (float)size_original * 100;
	std::cout << std::fixed << std::setprecision(2) << "Cloud has been reduced to " << temp << "% of its original size." << std::endl;
	std::cout << "--> calculation time: " << timer.count() << " seconds\n" << std::endl;
	
	return cloud_down;
}

void 
GeoDetection::applyTransformation(const Eigen::Matrix4f& transformation)
{
	std::cout << "Applying transformation..." << std::endl;
	auto start = std::chrono::steady_clock::now();

	pcl::transformPointCloud(*m_cloud, *m_cloud, transformation);
	Eigen::Matrix4d temp = transformation.cast<double>(); //using doubles for more accurate arithmitic
	m_transformation = temp * m_transformation; //eigen matrix multiplication

	auto timer = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start);
	std::cout << "--> calculation time: " << timer.count() << " seconds\n" << std::endl;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr 
GeoDetection::getKeyPoints()
{
	std::cout << "Computing intrinsic shape signature keypoints..." << std::endl;
	auto start = std::chrono::steady_clock::now();

	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss_detector;

	iss_detector.setSearchMethod(m_kdtree);

	//Have option to use scale to automatically determine parameters.
	//Declare hardcoded parameters together as const so they are easy to find.
	iss_detector.setSalientRadius(0.5); //***************************************************************************
	iss_detector.setNonMaxRadius(1.5); //***************************************************************************
	iss_detector.setMinNeighbors(5);

	iss_detector.setInputCloud(m_cloud);

	iss_detector.setThreshold21(0.975); //from ryan kromer
	iss_detector.setThreshold32(0.975); //***************************************************************************
	iss_detector.setNumberOfThreads(omp_get_num_procs()); //Explicitly doing this because PCL has been throwing user error 1001.
	iss_detector.compute(*keypoints);

	auto timer = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start);
	std::cout << "Keypoints computed: " << keypoints->size() << std::endl;
	std::cout << "--> calculation time: " << timer.count() << " seconds\n" << std::endl;
	
	return keypoints;
}

//Enable AVX in Properties --> C/C++ --> Enable Enhanced Instruction Set --> /arch:AVX
pcl::PointCloud<pcl::FPFHSignature33>::Ptr
GeoDetection::getFPFH(const pcl::PointCloud<pcl::PointXYZ>::Ptr& keypoints)
{
	std::cout << "Computing fast point feature histograms..." << std::endl;
	auto start = std::chrono::steady_clock::now();

	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh(new pcl::PointCloud<pcl::FPFHSignature33>);
	pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> computefpfh;
	
	computefpfh.setNumberOfThreads(omp_get_num_procs());
	computefpfh.setInputCloud(keypoints);
	computefpfh.setInputNormals(m_normals);

	computefpfh.setSearchMethod(m_kdtree);
	computefpfh.setRadiusSearch(3.0); //function of voxel subsample? ***************************************************************************

	computefpfh.setSearchSurface(m_cloud);

	computefpfh.compute(*fpfh);
	
	auto timer = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start);
	std::cout << "Fast point feature histograms computed" << std::endl;
	std::cout << "--> calculation time: " << timer.count() << " seconds\n" << std::endl;

	return fpfh; //--> causes heap error (ptr deleted twice?)
}
