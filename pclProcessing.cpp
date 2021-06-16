#include "pclProcessing.h"
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/flann_search.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>

std::vector<float> PclProcessing::getResolution(const int& k = 3)
{
	std::vector<float> resolution(m_cloud->size());
	double temp = 0;

	for (size_t i; i < m_cloud->size(); i++)
	{
		std::vector<int> indices(k);
		std::vector<float> sq_distances(k);

		m_kdtreeFLANN->nearestKSearch(i, k, indices, sq_distances);
		for (int j = 1; j < k; j++)
			temp += sq_distances[j];

		resolution[i] = temp / k;
		temp = 0;
	}
	return resolution;	
}

void PclProcessing::getNormals(const float& nrad)
{
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> calcnormals;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

	calcnormals.setInputCloud(m_cloud);
	calcnormals.setSearchMethod(tree);

	calcnormals.setViewPoint(0, 0, 0); //Explicit code. Its 0,0,0 by default.
	calcnormals.setRadiusSearch(nrad);
	//calcnormals.setNumberOfThreads();

	calcnormals.compute(*m_normals);

}

void PclProcessing::getNormals(const float& nrad, pcl::PointCloud<pcl::Normal>::Ptr& normals)
{
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> calcnormals;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

	calcnormals.setInputCloud(m_cloud);
	calcnormals.setSearchMethod(tree);

	calcnormals.setViewPoint(0, 0, 0); //Explicit code. Its 0,0,0 by default.
	calcnormals.setRadiusSearch(nrad);
	//calcnormals.setNumberOfThreads();

	calcnormals.compute(*normals);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PclProcessing::getVoxelDownSample(const float& voxelsize)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_down(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> grid;
	grid.setInputCloud(m_cloud);
	grid.setLeafSize(voxelsize, voxelsize, voxelsize);
	grid.filter(*cloud_down);
	
	return cloud_down;
}

void PclProcessing::DistanceDownSample(const float& distance)
{
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
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PclProcessing::getDistanceDownSample(const float& distance)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_copy(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*m_cloud, *cloud_copy);
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setNegative(true); //to remove the outliers

	size_t i = 0, n = cloud_copy->size();

	while (i < n) {
		pcl::PointIndices::Ptr outliers(new pcl::PointIndices());
		std::vector<int> IDRadiusSearch;
		std::vector<float> DistanceRadiusSearch;

		m_kdtreeFLANN->radiusSearch(cloud_copy->points[i], distance, IDRadiusSearch, DistanceRadiusSearch);

		if (IDRadiusSearch.size() < 2) {
			continue;
		}

		IDRadiusSearch.erase(IDRadiusSearch.begin()); //remove first point from the list (its the query point)
		outliers->indices = std::move(IDRadiusSearch);

		extract.setInputCloud(cloud_copy);
		extract.setIndices(outliers);
		extract.filter(*cloud_copy);

		i++;
		n = cloud_copy->size();
	}
	return cloud_copy;
}

void PclProcessing::getKeyPoints()
{

}

void PclProcessing::globalRegistration(PclProcessing reference)
{
	//Make sure reference cloud exists

	//Check for point normals, otherwise compute 
	


}