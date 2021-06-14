#pragma once
#include <pcl/point_types.h>
#include <pcl\kdtree\kdtree_flann.h>
#include <pcl/features/fpfh.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

// \brief Custom pointcloud object for geocomputation of LiDAR point clouds
class PclProcessing
{
public:
	std::string m_name = "Default";
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud = nullptr; //same as std::shared_ptr <pcl::PointCloud <pcl::PointXYZ> > = nullptr
	pcl::PointCloud<pcl::Normal>::Ptr m_normals = nullptr;

	pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr m_kdtreeFLANN = nullptr;

//Object creation
public:
	//Default constructor
	PclProcessing()
		:m_cloud(new pcl::PointCloud<pcl::PointXYZ>),
		m_normals(new pcl::PointCloud<pcl::Normal>),
		m_kdtreeFLANN(new pcl::KdTreeFLANN<pcl::PointXYZ>)
	{};

	//Constructor copying cloud smart_ptr
	PclProcessing(std::string& name, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
		:m_name(name),
		m_cloud(cloud)
	{}

	//Constructor for stealing the smart_ptr
	PclProcessing(std::string name, pcl::PointCloud<pcl::PointXYZ>::Ptr&& cloud)
		:m_name(name),
		m_cloud(std::move(cloud))
	{
		cloud = nullptr;
	}

	//Copy constructor
	PclProcessing(PclProcessing& other)
		:m_name(other.m_name),
		m_cloud(other.m_cloud),
		m_normals(other.m_normals),
		m_kdtreeFLANN(other.m_kdtreeFLANN)
	{
		std::cout << "\nCloud: " << other.m_name << "copied to " << m_name << '\n' << std::endl;
	}

	PclProcessing(PclProcessing&& other) noexcept
		:m_name(std::move(other.m_name)),
		m_cloud(std::move(other.m_cloud)),
		m_normals(std::move(other.m_normals)),
		m_kdtreeFLANN(std::move(other.m_kdtreeFLANN))

	{
		other.~PclProcessing();
	}

	~PclProcessing() {}

//methods
public:
    /** \brief Method for computing the average local point cloud resolution (i.e. spacing).
 * \param[in] k: the number of neighbors to use for determining average resolution (default=3).
 * * \param[out] resolution: the average point cloud resolution.
 */
	double getResolution(int k = 3);

	/** \brief Method for computing the local point cloud resolution (i.e. spacing).
	* \param[in] k: the number of neighbors to use for determining local resolution (default=3).
	* \param[out] vector containing local point resolution with matching indices. 
	*/
	std::vector<double> getLocalResolution(int k = 3);

	/** \brief Method for computing the local scale normals. 
	* \param[in] rad: the spherical neighborhood for normals.
	*/
	void getNormals();

	pcl::PointCloud<pcl::PointXYZ>::Ptr downSample();
	pcl::PointCloud<pcl::Normal>::Ptr getNormals();
	

};




/** \brief Search for k-nearest neighbors for the given query point (zero-copy).
  *
  * \attention This method does not do any bounds checking for the input index
  * (i.e., index >= cloud.size () || index < 0), and assumes valid (i.e., finite) data.
  *
  * \param[in] index a \a valid index representing a \a valid query point in the dataset given
  * by \a setInputCloud. If indices were given in setInputCloud, index will be the position in
  * the indices vector.
  *
  * \param[in] k the number of neighbors to search for
  * \param[out] k_indices the resultant indices of the neighboring points (must be resized to \a k a priori!)
  * \param[out] k_sqr_distances the resultant squared distances to the neighboring points (must be resized to \a k
  * a priori!)
  * \return number of neighbors found
  *
  * \exception asserts in debug mode if the index is not between 0 and the maximum number of points
  */