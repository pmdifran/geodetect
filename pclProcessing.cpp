#include "pclProcessing.h"
#include <pcl/features/normal_3d_omp.h>

double PclProcessing::getResolution(int k = 3)
{
	double resolution = 0; k++; //increase k, since the first neighbor of a point is itself.

	for (size_t i; i < m_cloud->size(); i++)
	{
		std::vector<int> indices(k);
		std::vector<float> sq_distances(k);

		m_kdtreeFLANN->nearestKSearch(i, k, indices, sq_distances);
		for (int j = 1; j < k; j++)
			resolution += sq_distances[j];
	}
	resolution /= (k - 1) * (m_cloud->size());
	return resolution;
}

pcl::PointCloud<pcl::Normal>::Ptr PclProcessing::getNormals()
{
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> calcnormals;
	
	//calcnormals.setViewPoint(0,0,0)
	calcnormals.setInputCloud(m_cloud);

	
	calcnormals.setSearchMethod(kdtree);


}