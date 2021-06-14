#pragma once
#include <pcl/point_types.h>
#include <pcl\kdtree\kdtree_flann.h>
#include <pcl/features/fpfh.h>

class PclProcessing
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = nullptr;

	double getResolution(int k = 3);
};

class pclPairProcessing
{
	PclProcessing target;
	PclProcessing source;


};

class pclPairProcess
{
public:
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_target = nullptr;
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_source = nullptr;

	pcl::KdTreeFLANN<pcl::PointXYZ> m_targetTree;
	pcl::KdTreeFLANN<pcl::PointXYZ> m_sourceTree;

	pclPairProcess(pcl::PointCloud<pcl::PointXYZ>::Ptr& target, pcl::PointCloud<pcl::PointXYZ>::Ptr& source)
		:
		m_target(target), m_source(source)
	{
		m_targetTree.setInputCloud(m_target);
		m_sourceTree.setInputCloud(m_source);
	}

	double getTargetResolution(int k = 3);
	double getSourceResolution(int k = 3);
	void downSampleClouds(double& resolution);
};

class autoRegister : public pclPairProcess
{
public:
	autoRegister(pcl::PointCloud<pcl::PointXYZ>::Ptr& target, pcl::PointCloud<pcl::PointXYZ>::Ptr& source)
		:pclPairProcess(target, source)
	{

	}

	void globalRegister();
};
