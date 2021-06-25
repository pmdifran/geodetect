#include "PCLreadASCII.h"
#include "GeoDetection.h"

int main (int argc, char* argv[])
{
	const char* file = "ttt.txt";
	std::cout << file << std::endl;
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr(new pcl::PointCloud<pcl::PointXYZ>);

	static uintmax_t lines = getLineCount(file);
	std::cout << lines << std::endl;

	bool hasheader = hasHeader(file);
	std::cout << "Header: " << hasheader << std::endl;

	size_t columns = getNumColumns(file);
	std::cout << "delimeter: " << columns << std::endl;

	char delimeter = getDelimeter(file);
	std::cout << "delimeter: " << delimeter << std::endl;


	std::cin.get();
}