#pragma once
#include "ScalarField.h"
#include "log.h"

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
		std::vector<ScalarField> m_scalarfields;

		//PCL Data Structures
		pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud;
		pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr m_kdtreeFLANN;
		pcl::search::KdTree<pcl::PointXYZ>::Ptr m_kdtree;
		pcl::PointCloud<pcl::Normal>::Ptr m_normals;

		//Transformation Matrix
		Eigen::Matrix4d m_transformation = Eigen::Matrix4d::Identity();
		std::array<float, 3> m_view = { 0, 0, 0 };

		//Scale and resoluton
		double m_resolution = 0.0; //average resolution of the point cloud.
		double m_scale = 0.0;

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW //So that dynamic allocation returns aligned pointer.

	//Constructors and assignments
	public:
		Cloud()
			: m_name("Cloud Default"),
			m_cloud(new pcl::PointCloud<pcl::PointXYZ>),
			m_kdtreeFLANN(new pcl::KdTreeFLANN<pcl::PointXYZ>),
			m_kdtree(new pcl::search::KdTree<pcl::PointXYZ>),
			m_normals(new pcl::PointCloud<pcl::Normal>)
		{
			GD_CORE_TRACE(":: Constructing empty GeoDetection Cloud...");
		}

		Cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string name = "Cloud Default")
			: m_name(name),
			m_cloud(cloud),
			m_kdtreeFLANN(new pcl::KdTreeFLANN<pcl::PointXYZ>),
			m_kdtree(new pcl::search::KdTree<pcl::PointXYZ>),
			m_normals(new pcl::PointCloud<pcl::Normal>)
		{
			GD_CORE_TITLE("GeoDetection Cloud Construction");
			GD_CORE_TRACE("Creating GeoDetection Cloud Object: '{0}' with {1} points", m_name, m_cloud->size());
			buildKdTrees();
			GD_CORE_INFO("--> GeoDetection Cloud Created\n");
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
		inline std::vector <ScalarField>* const scalarfields() { return &m_scalarfields; }
		inline Eigen::Matrix4d transformation() const { return m_transformation; }
		inline double resolution() const { return m_resolution; }

		//Setters and checks
	public:
		//Sets the cloud to a new pcl::PointCloud, updates the KdTrees, and clears the normals. 
		inline void setScalarFields(std::vector<ScalarField> sf) { std::copy(sf.begin(), sf.end(), m_scalarfields.begin()); }
		inline void setCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) { m_cloud = std::move(cloud); buildKdTrees(); m_normals->clear(); }
		inline void setNormals(pcl::PointCloud<pcl::Normal>::Ptr& normals) { m_normals = std::move(normals); }
		inline void setView(float x, float y, float z) { m_view[0] = x; m_view[1] = y; m_view[2] = z; }
		void setScale(float scale) { m_scale = scale; }
		
		// Computes normals with radius search and sets the member m_normals.
		void setNormalsRadiusSearch(float radius) { m_normals = this->getNormalsRadiusSearch(radius); }

		// Computes normals with k nearest neighbor search and sets the member m_normals.
		void setNormalsKSearch(int k) { m_normals = this->getNormalsKSearch(k); }

		inline bool hasNormals() { return m_normals->size() > 0; }
		inline bool hasScalarFields() { return m_scalarfields.size() > 0; }

		//Methods
	public:
		/** \brief Method for adding another column of scalar fields
		* \param[in] new_fields: float vector which should be the same length as # of points (this is checked for).
		* \return Internal: m_scalarfields is modified to include an additional pointer to the fields.
		*/
		inline void addScalarField(ScalarField&& new_fields)
		{
			if (new_fields.size() == m_cloud->size()) { m_scalarfields.push_back(std::move(new_fields)); }
			else { GD_CORE_ERROR(":: Scalar field size must agree with the cloud size"); }
		}

		/** \brief Method for removing column of scalar fields
		* \param[in] index: index of scalar field to remove (removes the largest index by default).
		* \return Internal: m_scalarfields is modified to remove an pointer to a fields column.
		*/
		inline void deleteScalarField(int index)
		{
			if (index < 0 || m_scalarfields.size() == 0 || index > (m_scalarfields.size() - 1))
			{
				GD_CORE_ERROR(":: Attempting to access a scalar field index that does not exist");  
				return;
			}

			m_scalarfields.erase(m_scalarfields.begin() + index);
		}

		inline void deleteFirstScalarField() { this->deleteScalarField(0); }
		inline void deleteLastScalarField() { this->deleteScalarField(m_scalarfields.size() - 1); }

		/** \brief Method for averaging specific scalar fields within a defined search radius, and at select corepoints*/
		void averageScalarFieldSubset(float radius, int field_index, pcl::PointCloud<pcl::PointXYZ>::Ptr corepoints);

		/** \brief Method for averaging specific scalar fields within a defined search radius, at all points (m_cloud) */
		void averageScalarField(float radius, int field_index);

		/** \brief Method for averaging scalar fields within a defined search radius, at select corepoints*/
		void averageAllScalarFieldsSubset(float radius, pcl::PointCloud<pcl::PointXYZ>::Ptr corepoints);

		/** \brief Method for averaging all scalar fields within a defined search radius, at all points (m_cloud) */
		void averageAllScalarFields(float radius);

		void buildKdTrees();

		/** \brief Method for computing normals. View point is set as 0,0,0 as default unless set with setView
		* \param[in] radius: Radius for spherical neighbour search used for principle component analysis.
		* \return shared pointer to the computed normals.
		*/
		pcl::PointCloud<pcl::Normal>::Ptr getNormalsRadiusSearch(float radius);

		/** \brief Method for computing normals. View point is set as 0,0,0 as default unless set with setView
		* \param[in] k: # neighbors to use for normal estimation
		* \return shared pointer to the computed normals.
		*/
		pcl::PointCloud<pcl::Normal>::Ptr getNormalsKSearch(int k);

		/** \brief Method for averaging normals within a defined search radius, at select subset of corepoints */
		void averageNormalsSubset(float radius, pcl::PointCloud<pcl::PointXYZ>::Ptr corepoints);

		/** \brief Method for averaging normals within a defined search radius, at all points (m_cloud) */
		void averageNormals(float radius);

		/** \brief Method for computing the local point cloud resolution (i.e. spacing).
		* \param[in] k: the number of neighbors to use for determining local resolution (default=2).
		* \return Vector of local resolutions, consistent with point cloud indices.
		Internal: updates member m_resolution (average cloud resolution).
		*/
		std::vector<float> getResolution(int num_neighbors = 2);

		/** \brief Method for generating a new, subsampled cloud, using a voxel filter. The local cloud should be dense relative to voxel size
		{i.e. specify it based on the point cloud resolution from getResolution()}.
		* \param[in] voxel_size: cubic voxel size used to create average-point locations.
		* \return Shared pointer to the subsampeld cloud.
		*/
		pcl::PointCloud<pcl::PointXYZ>::Ptr getVoxelDownSample(float voxel_size);

		/** \brief Similar to getVoxelDownsample, but directly modifies member m_cloud, resets KdTrees, 
		*    and averages normals/scalar fields.
		* \param[in] distance: minimum distance between points
		* \return Internal: modifies m_cloud, KdTrees, Normals, Scalar Fields
		*/
		void voxelDownSample(float voxel_size);

		/** \brief Method for generating a new, subsampled cloud, using a minimum distance (similar to CloudCompare).
		* \param[in] distance: minimum distance between points
		* \return Shared pointer to the subsampled cloud.
		*/
		pcl::PointCloud<pcl::PointXYZ>::Ptr getDistanceDownSample(float distance);

		/** \brief Similar to getDistanceDownsample, but directly modifies member m_cloud and resets normals/scalar fields.
		* \param[in] distance: minimum distance between points
		* \return Internal: Internal: modifies m_cloud, KdTrees, Normals, Scalar Fields
		*/
		void distanceDownSample(float distance);

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
	};

}