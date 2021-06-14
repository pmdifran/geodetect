#include "autoReg.h"
#include <pcl/console/time.h>

//Get the resolution of source pointcloud using k local neighbors (default = 3)
double pclPairProcess::getTargetResolution(int k = 3)
{
	double resolution = 0; k++; //increase k, since the first neighbor of a point is itself.

	for (size_t i; i < m_target->size(); i++)
	{
		std::vector<int> indices(k);
		std::vector<float> sq_distances(k);

		m_targetTree.nearestKSearch(i, k, indices, sq_distances);
		for (int j = 1; j < k; j++)
			resolution += sq_distances[j];
	}
	resolution /= (k - 1) * (m_target->size());
	return resolution;
}

//Get the resolution of source pointcloud using k local neighbors (default = 3)
double pclPairProcess::getSourceResolution(int k = 3)
{
	double resolution = 0; k++; //increase k, since the first neighbor of a point is itself.

	for (size_t i; i < m_target->size(); i++)
	{
		std::vector<int> indices(k);
		std::vector<float> sq_distances(k);

		m_targetTree.nearestKSearch(i, k, indices, sq_distances);
		for (int j = 1; j < k; j++)
			resolution += sq_distances[j];
	}
	resolution /= (k - 1) * (m_target->size());
	return resolution;
}

void pclPairProcess::downSampleClouds(double& resolution)
{
	
}

void autoRegister::globalRegister()
{
	pcl::console::TicToc time;
	time.tic();

	
}

