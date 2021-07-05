#include "GeoDetection.h"
#include <chrono>
#include <fstream>
#include <iomanip>
#include <pcl/common/impl/transforms.hpp>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/flann_search.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/features/fpfh_omp.h>


//auto start = std::chrono::steady_clock::now();
//auto timer = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start);
//std::cout << "--> calculation time: "<< timer.count() << " seconds\n" << std::endl;

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
	ofs << std::fixed << std::setprecision(16) << m_RT << '\n';
	ofs.close();

}


std::vector<float> GeoDetection::getResolution(const int& k)
{
	std::cout << "Computing cloud resolution..." << std::endl;
	std::cout << ":: k-nearest neighbors used for distances: " << k << std::endl;
	auto start = std::chrono::steady_clock::now();

	std::vector<float> resolution(m_cloud->size());
	double temp = 0;
	double newresolution = 0;

	for (size_t i = 0; i < m_cloud->size(); i++)
	{
		std::vector<int> indices(k);
		std::vector<float> sq_distances(k);

		m_kdtreeFLANN->nearestKSearch(i, k, indices, sq_distances);
		for (int j = 1; j < k; j++)
			temp += sq_distances[j];

		temp /= k;
		resolution[i] = temp;
		newresolution += temp;

		temp = 0;
	}
	m_resolution = newresolution;
	auto timer = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start);

	std::cout << "Point cloud resolution: " << m_resolution << std::endl;
	std::cout << "--> calculation time: "<< timer.count() << " seconds\n" << std::endl;

	return resolution;	
}

void 
GeoDetection::computeNormals(const float& nrad)
{
	std::cout << "Computing point cloud normals..." << std::endl;
	std::cout << ":: normal scale:  " << nrad << std::endl;
	auto start = std::chrono::steady_clock::now();

	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> calcnormals;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

	calcnormals.setInputCloud(m_cloud);
	calcnormals.setViewPoint(m_view[0], m_view[1], m_view[2]); //0,0,0 as default
	calcnormals.setSearchMethod(tree);
	calcnormals.setRadiusSearch(nrad);
	//calcnormals.setNumberOfThreads();

	calcnormals.compute(*m_normals);
	auto timer = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start);

	std::cout << "Normals computed" << std::endl;
	std::cout << "--> calculation time: " << timer.count() << " seconds\n" << std::endl;
}

void 
GeoDetection::computeNormals(const float& nrad, pcl::PointCloud<pcl::Normal>::Ptr& normals)
{
	std::cout << "Computing point cloud normals..." << std::endl;
	std::cout << ":: normal scale:  " << nrad << std::endl;
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> calcnormals;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	auto start = std::chrono::steady_clock::now();


	calcnormals.setInputCloud(m_cloud);
	calcnormals.setSearchMethod(tree);
	calcnormals.setViewPoint(m_view[0], m_view[1], m_view[2]); //0,0,0 as default
	calcnormals.setRadiusSearch(nrad);
	//calcnormals.setNumberOfThreads(); //set number of openmp threads

	calcnormals.compute(*normals);
	auto timer = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start);

	std::cout << "Normals computed" << std::endl;
	std::cout << "--> calculation time: " << timer.count() << " seconds\n" << std::endl;
}

void 
GeoDetection::VoxelDownSample(const float& voxelsize)
{
	std::cout << "Creating new downsampled cloud with voxels..." << std::endl;
	std::cout << ":: voxel fitler size : " << voxelsize << std::endl;
	auto start = std::chrono::steady_clock::now();

	size_t isize = m_cloud->size();
	pcl::VoxelGrid<pcl::PointXYZ> grid;
	grid.setInputCloud(m_cloud);
	grid.setLeafSize(voxelsize, voxelsize, voxelsize);
	grid.filter(*m_cloud);
	auto timer = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start);

	std::cout << ":: Initial cloud size: " << m_cloud->size() << std::endl;
	std::cout << "::   Downsampled size: " << isize << std::endl;
	std::cout << "--> calculation time: " << timer.count() << " seconds\n" << std::endl;

	if (m_resolution == 0) {
		std::cout << "Average cloud resolution variable remains uninitialized" << std::endl;
		std::cout << "--> Run GeoDetection::getResolution() prior to any resolution-based parameter selection \n" << std::endl;
	}
	else {
		float temp = (isize - m_cloud->size()) / isize * 100;
		std::cout << "Cloud has been reduced by " << temp << "%" << std::endl;
		std::cout << "--> The average cloud resolution may have changed" << std::endl;
		std::cout << "--> Its recommended to run GeoDetection::getResolution() prior to any further resolution-based parameter selection ** \n" << std::endl;
		}
}

pcl::PointCloud<pcl::PointXYZ>::Ptr 
GeoDetection::getVoxelDownSample(const float& voxelsize)
{
	std::cout << "Creating new downsampled cloud with voxels..." << std::endl;
	std::cout << ":: voxel fitler size : " << voxelsize << std::endl;
	auto start = std::chrono::steady_clock::now();

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_down(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> grid;
	grid.setInputCloud(m_cloud);
	grid.setLeafSize(voxelsize, voxelsize, voxelsize);
	grid.filter(*cloud_down);
	auto timer = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start);
	
	std::cout << "::  Full cloud size: " << m_cloud->size() << std::endl;
	std::cout << ":: Downsampled size: " << cloud_down->size() << std::endl;
	std::cout << "--> calculation time: " << timer.count() << " seconds\n" << std::endl;
	return cloud_down;
}

void 
GeoDetection::DistanceDownSample(const float& distance)
{
	std::cout << "Subsampling cloud by distance..." << std::endl;
	std::cout << ":: subsampling distance: " << std::endl;
	auto start = std::chrono::steady_clock::now();

	size_t isize = m_cloud->size();
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setNegative(true); //to remove the outliers

	size_t i = 0, n = m_cloud->size();

	while (i < n) {
		pcl::PointIndices::Ptr outliers(new pcl::PointIndices());
		std::vector<int> IDRadiusSearch;
		std::vector<float> DistanceRadiusSearch;

		m_kdtreeFLANN->radiusSearch(m_cloud->points[i], distance, IDRadiusSearch, DistanceRadiusSearch);

		if (IDRadiusSearch.size() < 2) {
			continue;
		}

		IDRadiusSearch.erase(IDRadiusSearch.begin()); //remove first point from the list (its the query point)
		outliers->indices = std::move(IDRadiusSearch);

		extract.setInputCloud(m_cloud);
		extract.setIndices(outliers);
		extract.filter(*m_cloud);

		i++;
		n = m_cloud->size();
	}
	auto timer = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start);
	std::cout << ":: Initial cloud size: " << m_cloud->size() << std::endl;
	std::cout << "::   Downsampled size: " << isize << std::endl;
	std::cout << "--> calculation time: " << timer.count() << " seconds\n" << std::endl;

	if (m_resolution == 0) {
		std::cout << "Average cloud resolution variable remains uninitialized" << std::endl;
		std::cout << "--> ** Run GeoDetection::getResolution() prior to any resolution-based parameter selection\n" << std::endl;
	}
	else {
		float temp = (isize - m_cloud->size()) / isize * 100;
		std::cout << "Cloud has been reduced by " << temp << "%" << std::endl;
		std::cout << "--> The average cloud resolution may have changed" << std::endl;
		std::cout << "--> Its recommended to run GeoDetection::getResolution() prior to any further resolution-based parameter selection ** \n" << std::endl;
	}
}

pcl::PointCloud<pcl::PointXYZ>::Ptr 
GeoDetection::getDistanceDownSample(const float& distance)
{
	std::cout << "Creating new downsampled cloud by distance..." << std::endl;
	std::cout << ":: minimum distance: " << distance << std::endl;
	auto start = std::chrono::steady_clock::now();

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_down(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*m_cloud, *cloud_down);
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setNegative(true); //to remove the outliers

	size_t i = 0, n = cloud_down->size();

	while (i < n) {
		pcl::PointIndices::Ptr outliers(new pcl::PointIndices());
		std::vector<int> IDRadiusSearch;
		std::vector<float> DistanceRadiusSearch;

		m_kdtreeFLANN->radiusSearch(cloud_down->points[i], distance, IDRadiusSearch, DistanceRadiusSearch);

		if (IDRadiusSearch.size() < 2) {
			continue;
		}

		IDRadiusSearch.erase(IDRadiusSearch.begin()); //remove first point from the list (its the query point)
		outliers->indices = std::move(IDRadiusSearch);

		extract.setInputCloud(cloud_down);
		extract.setIndices(outliers);
		extract.filter(*cloud_down);

		i++;
		n = cloud_down->size();
	}

	auto timer = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start);
	std::cout << "::  Full cloud size: " << m_cloud->size() << std::endl;
	std::cout << ":: Downsampled size: " << cloud_down->size() << std::endl;
	std::cout << "--> calculation time: " << timer.count() << " seconds\n" << std::endl;
	return cloud_down;
}

void 
GeoDetection::applyTransformation(Eigen::Matrix4f& transformation)
{
	std::cout << "Applying transformation..." << std::endl;
	auto start = std::chrono::steady_clock::now();

	pcl::transformPointCloud(*m_cloud, *m_cloud, transformation);
	Eigen::Matrix4d temp = transformation.cast<double>(); //using doubles for more accurate arithmitic
	m_RT = temp * m_RT; //eigen matrix multiplication

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


	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	iss_detector.setSearchMethod(tree);

	//Use point cloud resoltion, or subsample size, to determine the parameters
	iss_detector.setSalientRadius(0.5); //***************************************************************************
	iss_detector.setNonMaxRadius(1.5); //***************************************************************************

	iss_detector.setInputCloud(m_cloud); //or subsampled cloud? ***************************************************************************

	iss_detector.setThreshold21(0.975); //from ryan kromer
	iss_detector.setThreshold32(0.975); //***************************************************************************
	iss_detector.compute(*keypoints);

	auto timer = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start);
	std::cout << "Keypoints computed" << std::endl;
	std::cout << "--> calculation time: " << timer.count() << " seconds\n" << std::endl;
	
	return keypoints;
}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr 
GeoDetection::getFPFH(pcl::PointCloud<pcl::PointXYZ>::Ptr& keypoints)
{
	std::cout << "Computing fast point feature histograms..." << std::endl;
	auto start = std::chrono::steady_clock::now();

	pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> computefpfh;
	
	computefpfh.setInputCloud(keypoints);
	computefpfh.setInputNormals(m_normals);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	computefpfh.setSearchMethod(tree);
	computefpfh.setRadiusSearch(1.0); //function of voxel subsample? ***************************************************************************
	computefpfh.setSearchSurface(m_cloud);//subsample? ******************************************************************

	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh(new pcl::PointCloud<pcl::FPFHSignature33>());
	computefpfh.compute(*fpfh);
	
	auto timer = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start);
	std::cout << "Fast point feature histograms computed" << std::endl;
	std::cout << "--> calculation time: " << timer.count() << " seconds\n" << std::endl;

	return fpfh;
}
