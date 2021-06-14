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

//Custom input data type xyz with numerous fields (M3C2 Distance, Normals, Cluster ID). 
typedef struct XYZ
{
	float x, y, z;  // X, Y, Z position
}XYZ;

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

void readXYZ(std::string& filename, std::vector<XYZ>& xyzpoints); 

void convert_toPCL(std::vector<XYZ>& xyzpoints,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr, unsigned int& num_points);

inline std::chrono::high_resolution_clock::time_point getTime() { return std::chrono::high_resolution_clock::now(); }