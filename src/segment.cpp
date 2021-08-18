#include "segment.h"

void segmentVegetation(GeoDetection::Cloud& cloud)
{
	std::vector<int> point_class(size_t(cloud.m_cloud->size()), 1);
	std::vector<float> vegetation_score(size_t(cloud.m_cloud->size()), 0);

	//compute normals at various scales.
	pcl::PointCloud<pcl::Normal>::Ptr normals = cloud.getNormals(0.2);
	
}