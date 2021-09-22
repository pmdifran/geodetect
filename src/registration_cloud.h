#pragma once
#include "GeoDetection.h"

namespace GeoDetection
{
	//Registration cloud for storing the features computed in auto registration \
	Typically a reference cloud which is used numerous times for registration in a batch processing
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

		RegistrationCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string name = "Registation Cloud Default")
			: Cloud(cloud, name), 
			m_keypoints(this->getKeyPoints()),
			m_fpfh(this->getFPFH(m_keypoints))

		{
			GD_CORE_INFO("--> Registration members added to empty GeoDetection Cloud");
		}

		inline pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints() { return m_keypoints; }
		inline pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh() { return m_fpfh; }

		inline bool hasKeypoints() { return m_keypoints->size() > 0; }
		inline bool hasFPFH() { return m_fpfh->size() > 0; }

		/**
		* Compute keypoints with current cloud.
		*/
		inline void updateKeypoints() { m_keypoints = this->getKeyPoints(); }

		/**
		* Compute fpfh with current keypoints and cloud.
		*/
		inline void updateFPFH()
		{
			if (this->hasKeypoints()) { this->getFPFH(m_keypoints); }
			else { GD_ERROR("Trying to compute reference FPFH without having first calculated keypoints"); }
		}

	};
}