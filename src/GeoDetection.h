#pragma once
#include "log.h"
#include "ScalarField.h"

//PCL core
#include <pcl/point_types.h>

//KdTrees
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>

//Filters
#include <pcl/filters/extract_indices.h>

#include <array>
#include <cmath>
#include <chrono>

// \brief Custom pointcloud object for geocomputation of LiDAR point clouds
namespace GeoDetection
{
	class Cloud
	{
		//Members
	private:
		std::string m_name;

		//Scalar Fields
		int m_num_fields = 0;
		std::vector<ScalarField> m_scalar_fields;

		//PCL Data Structures
		pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud;
		pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr m_kdtreeFLANN;
		pcl::search::KdTree<pcl::PointXYZ>::Ptr m_kdtree;
		pcl::PointCloud<pcl::Normal>::Ptr m_normals;

		//Transformation Matrix
		Eigen::Matrix4d m_transformation;
		std::array<float, 3> m_view = { 0, 0, 0 };

		//Scale and resoluton
		double m_resolution = 0.0; //average resolution of the point cloud.
		double m_scale = 0.0;

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW //So that dynamic allocation returns aligned pointer.

	//Constructors and assignments
	public:
		Cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string name = "Cloud Default")
			: m_name(name),
			m_cloud(cloud),
			m_kdtreeFLANN(new pcl::KdTreeFLANN<pcl::PointXYZ>),
			m_kdtree(new pcl::search::KdTree<pcl::PointXYZ>),
			m_normals(new pcl::PointCloud<pcl::Normal>),
			m_transformation(Eigen::Matrix4d::Identity())

		{
			GD_CORE_TITLE("GeoDetection Cloud Construction");
			GD_CORE_TRACE("Creating GeoDetection Cloud Object: '{0}' with {1} points", m_name, m_cloud->size());
			getKdTrees();
			GD_CORE_INFO("--> GeoDetection Cloud Created");
		}

		Cloud(const Cloud&) = default;
		Cloud(Cloud&&) = default;

		Cloud& operator=(const Cloud&) = default;
		Cloud& operator=(Cloud&&) = default;

		~Cloud() = default;

		//Accessors
	public:
		inline std::string name() { return m_name; }
		inline pcl::PointCloud<pcl::PointXYZ>::Ptr const cloud() const { return m_cloud; }
		inline pcl::search::KdTree<pcl::PointXYZ>::Ptr const tree() const { return m_kdtree; }
		inline pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr const flanntree() const { return m_kdtreeFLANN; }
		inline pcl::PointCloud<pcl::Normal>::Ptr const normals() const { return m_normals; }
		inline const std::vector <ScalarField> scalarfields() const { return m_scalar_fields; }
		inline Eigen::Matrix4d transformation() const { return m_transformation; }

		//Setters and checks
	public:
		//Sets the cloud to a new pcl::PointCloud, updates the KdTrees, and clears the normals. 
		inline void setScalarFields(std::vector<ScalarField> sf) { std::copy(sf.begin(), sf.end(), m_scalar_fields.begin()); }
		inline void setCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) { m_cloud = std::move(cloud); getKdTrees(); m_normals->clear(); }
		inline void setNormals(pcl::PointCloud<pcl::Normal>::Ptr& normals) { m_normals = std::move(normals); }
		inline void setView(float x, float y, float z) { m_view[0] = x; m_view[1] = y; m_view[2] = z; }
		void setScale(float scale) { m_scale = scale; }

		inline bool hasNormals() { return m_normals->size() > 0; }
		inline bool hasScalarFields() { return m_num_fields > 0; }

		//Methods
	public:
		/** \brief Method for adding another column of scalar fields
		* \param[in] new_fields: float vector which should be the same length as # of points (this is checked for).
		* \return Internal: m_scalar_fields is modified to include an additional pointer to the fields.
		*/
		inline void addScalarField(ScalarField& new_fields)
		{
			if (new_fields.size() == m_cloud->size()) { m_scalar_fields.push_back(std::move(new_fields)); m_num_fields++; }
			else { GD_CORE_ERROR(":: Scalar field size must agree with the cloud size"); }
		}

		/** \brief Method for removing column of scalar fields
		* \param[in] new_fields: index of scalar field to remove
		* \return Internal: m_scalar_fields is modified to remove an pointer to a fields column.
		*/
		inline void deleteScalarField(int index = -1)
		{
			if (index != -1 || index < 0 || m_num_fields == 0 || index >(m_num_fields - 1))
			{
				GD_CORE_ERROR(":: Attempting to access a scalar field index that does not exist");  return;
			}
			if (index == -1) { index = m_num_fields - 1; }
			m_scalar_fields.erase(m_scalar_fields.begin() + index);
			m_num_fields--;
		}

		/** \brief Method for averaging scalar fields within a defined search radius.*/
		void averageScalarFields(const float radius, int index = -1);

		void getKdTrees();

		/** \brief Method for computing normals. View point is set as 0,0,0 as default unless set with setView
		* \param[in] nrad: Radius for spherical neighbour search used for principle component analysis.
		* \param[in] set_m_normals: Whether to set member m_normals to the result (default = true).
		* \return shared pointer to the computed normals.
		*/
		pcl::PointCloud<pcl::Normal>::Ptr getNormals(const float nrad, const bool set_m_normals = true);

		/** \brief Method for averaging normals within a defined search radius.*/
		void averageNormals(const float radius);

		/** \brief Method for computing the local point cloud resolution (i.e. spacing).
		* \param[in] k: the number of neighbors to use for determining local resolution (default=2).
		* \return Vector of local resolutions, consistent with point cloud indices.
		Internal: updates member m_resolution (average cloud resolution).
		*/
		std::vector<float> getResolution(const int num_neighbors = 2);

		/** \brief Method for generating a new, subsampled cloud, using a voxel filter. The local cloud should be dense relative to voxel size
		{i.e. specify it based on the point cloud resolution from getResolution()}.
		* \param[in] voxel_size: cubic voxel size used to create average-point locations.
		* \return Shared pointer to the subsampeld cloud.
		*/
		pcl::PointCloud<pcl::PointXYZ>::Ptr getVoxelDownSample(const float voxel_size);

		/** \brief Similar to getVoxelDownsample, but directly modifies member m_cloud, resets KdTrees, 
		*    and averages normals/scalar fields.
		* \param[in] distance: minimum distance between points
		* \return Internal: modifies m_cloud, KdTrees, Normals, Scalar Fields
		*/
		void voxelDownSample(const float voxel_size);

		/** \brief Method for generating a new, subsampled cloud, using a minimum distance (similar to CloudCompare).
		* \param[in] distance: minimum distance between points
		* \return Shared pointer to the subsampled cloud.
		*/
		pcl::PointCloud<pcl::PointXYZ>::Ptr getDistanceDownSample(const float distance);

		/** \brief Similar to getDistanceDownsample, but directly modifies member m_cloud and resets normals/scalar fields.
		* \param[in] distance: minimum distance between points
		* \return Internal: Internal: modifies m_cloud, KdTrees, Normals, Scalar Fields
		*/
		void distanceDownSample(const float distance);

		/** \brief Method for computing intrinsic shape signature keypoints.
		* \return shared pointer to a pcl point cloud.
		*/
		pcl::PointCloud<pcl::PointXYZ>::Ptr getKeyPoints();

		/** \brief Method for computing fast point feature histograms
		* \param[in] shared pointer to a pcl point cloud containing keypoints
		*/
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr getFPFH(const pcl::PointCloud<pcl::PointXYZ>::Ptr& keypoints);

		/** \brief Method for transforming the cloud and updating the object's final transformation matrix
		* \param[in] transformation: affine matrix.
		* \return Internal: updates m_transformation and repositions m_cloud.
		*/
		void applyTransformation(const Eigen::Matrix4f& transformation);

		/** \brief Method for ONLY updating the object's final transformation matrix.
		* Should be called if a function has transformed the cloud without updating the matrix (i.e. generalized icp)
		* \param[in] transformation: affine matrix.
		* \return Internal: updates m_transformation.
		*/
		void updateTransformation(const Eigen::Matrix4f& transformation);

		/* \brief Method for filtering NaN values from the point cloud.
		*/
		void removeNaN();

		/* \brief Method for writing the transformation matrix to an ascii file.
		* \param[in] fname: Output file path/name.
		*/
		void writeTransformation(const char* fname);

		void writeAsASCII(const char* fname, bool write_normals = true, bool write_scalarfields = true);

		void writeAsPCD(const char* fname);
	};

}

//Abstract functions / helpers
namespace GeoDetection
{
		
	//Average-out normals around a given radius of core points. For entire cloud: set corepoints equal to cloud.
	pcl::PointCloud<pcl::Normal>::Ptr
		computeAverageNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
			const pcl::PointCloud<pcl::Normal>::Ptr normals,
			const pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree,
			float radius,
			pcl::PointCloud<pcl::PointXYZ>::Ptr corepoints = nullptr);

	//Average-out scalar fields around a given radius of core points. For entire cloud: set corepoints equal to cloud.
	ScalarField
		computeAverageFields(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, ScalarField fields,
			pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree, float radius,
			pcl::PointCloud<pcl::PointXYZ>::Ptr corepoints = nullptr);
}