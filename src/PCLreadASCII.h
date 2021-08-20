//Class definitions and functon declarations for importing custom .txt pointcloud data, and converting it to a pcl type.
	//Structure of input data is: x,y,z,M3C2distance,Nx,Ny,Nz

#pragma once

#include <vector>
#include <string>
#include <iostream>	//for std::cout
#include <fstream>
#include <iomanip>  //for setprecision()
#include <locale> //for csv reader locale
#include <cstdlib> //for exit()
#include <chrono>

#include <pcl\point_types.h>
#include <pcl\point_cloud.h>

namespace GeoDetection
{
	//Reads xyz data from an ASCII file into the provided PCL xyz point cloud.
	pcl::PointCloud<pcl::PointXYZ>::Ptr
		PCLreadASCIIxyz(char const* fname);

	void
		PCLwriteASCIIxyz(char const* fname, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudptr);

	//Custom input data type xyz. You can modify this to add more fields. 
	typedef struct _XYZ
	{
		float x, y, z;  // X, Y, Z position
	}XYZ;

	//Reads xyz data from an ASCII file into a templated vector point cloud (needs xyz fields --> see example)
	template <typename point_t>
	void
		ASCIIreadXYZ(std::string& filename, std::vector<point_t>& points);

	//Copies the vector xyz data from an ASCII file into the provided vector point cloud
	void
		ASCIItoPCL(std::vector<XYZ>& xyzpoints, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr);

	//Locale to treat commas, spaces, and newlines as whitespace. 
	struct csv_reader : std::ctype<char>
	{
		csv_reader()
			: std::ctype<char>(get_table()) {} //construct csv_reader by calling get_table

		static std::ctype_base::mask const* get_table()
		{
			static std::vector<std::ctype_base::mask> rc(table_size, std::ctype_base::mask());

			//define these characters as whitespace
			rc[','] = std::ctype_base::space;
			rc['\n'] = std::ctype_base::space;
			rc[' '] = std::ctype_base::space;
			rc[';'] = std::ctype_base::space;
			return &rc[0]; //returns ctype mapping table address.
		}
	};

}

