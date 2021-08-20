// Functions for importing custom .txt pointcloud data, and converting it to a pcl type.

#include "PCLreadASCII.h" //includes our custom point cloud structures, and those from pcl.
#include "log.h"

#include <cstring> //for memchr
#include <cstdlib> //for strtod
#include "fast_float.h" //Single header library version of fast_float

#include <limits>
#include <cstdio>
#include <filesystem>

namespace GeoDetection
{
	//HELPER FUNCTIONS
	size_t
		getNumColumns(const char* fname)
	{
		size_t num_columns = 0;

		//input filestream
		std::string fnamestr(fname);
		std::ifstream in(fnamestr);

		//objects for parsing
		float temp = 0;
		std::string line; //string object for getline

		if (!in.is_open()) {
			GD_ERROR("Failed to open file: {0} \n", fname);
			std::exit(EXIT_FAILURE);
		}

		std::getline(in, line);
		std::getline(in, line); //take second line incase of header
		std::istringstream iss(line);
		iss.imbue(std::locale(std::locale(), new csv_reader())); //treat CSV delimiters as whitespace

		while (iss >> temp) num_columns++;

		in.close();
		return num_columns;
	}

	void
		checkIfXYZ(char const* fname)
	{
		//Check that there is XYZ data
		size_t num_columns = getNumColumns(fname);
		if (num_columns < 3) {
			GD_ERROR(":: Number of columns detected: {0}\
			\n--> The inputted file must contain XYZ coordinates.", num_columns);
			std::exit(EXIT_FAILURE);
		}
	}

	uintmax_t
		getLineCount(char const* fname)
	{
		FILE* file;
		errno_t err;

		if ((err = fopen_s(&file, fname, "r")) != 0)
		{
			GD_ERROR("Failed to open file: {0} \n", fname);
		}

		static const auto BUFFER_SIZE = 16 * 1024;
		char buf[BUFFER_SIZE + 1];
		uintmax_t lines = 1;

		while (size_t bytes_read = fread(&buf, 1, BUFFER_SIZE, file)) {

			if (bytes_read == std::numeric_limits<size_t>::max())
				GD_ERROR("Read failed \n");
			if (!bytes_read)
				break;

			//memchr searches first x bytes of buffer and returns pointer to the first instance of '\n'. If none found, returns nullptr
			for (char* p = buf; (p = (char*)memchr(p, '\n', (buf + bytes_read) - p)); p++)				// --> nullptr is implictly converted into booleans false, which terminates the loop :)
				lines++;								  // ^^^^^^^^^^^^^^^^^ Gets the remaining # of addresses in our buffer
		}

		fclose(file);

		//If last character is newline, reduce the number of lines by 1
		std::ifstream in(fname);

		if (!in.is_open()) {
			GD_ERROR("Failed to open file: {0} \n", fname);
		}

		char ch;
		in.seekg(-1, std::ios_base::end);
		in.get(ch);

		if (ch == '\n') { lines--; }

		in.close();
		return lines;
	}

	bool
		checkIfHeader(char const* fname)
	{
		//input filestream
		std::ifstream in(fname);

		//objects for parsing
		pcl::PointXYZ xyz; // temporary point
		std::string line; //string object for getline

		if (!in.is_open()) {
			GD_ERROR("Failed to open file: {0} \n", fname);
			std::exit(EXIT_FAILURE);
		}

		//Check if the file has a header
		std::getline(in, line);
		std::istringstream iss(line); //construct into stringstream object
		iss.imbue(std::locale(std::locale(), new csv_reader())); //using locale that treats CSV delimiters as whitesapce.

		if (iss >> xyz.x >> xyz.y >> xyz.z) { // stream extraction returns false if a non-numeric value is entered
			std::cout << ":: No header detected." << std::endl;
			return false;
		}
		else {
			GD_TRACE(":: Header found.");
			return true;
		}
	}

	void
		reserveCloudSpace(const char* fname, bool header_present,
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr)
	{
		size_t num_points = getLineCount(fname);
		if (num_points < 2) {
			GD_ERROR("Needs more than 2 points to create a point cloud object.\
			\n Exiting the program...");
			std::exit(EXIT_FAILURE);
		}

		if (header_present) { num_points--; }

		cloudptr->points.reserve(num_points);
		GD_TRACE(":: Number of points: {0}", num_points);
	}

	char
		getDelimeter(const char* fname)
	{
		const char delimeters[] = { ',', ' ', ';', 0 };

		//input filestream
		std::string fnamestr(fname);
		std::ifstream in(fnamestr);

		//objects for parsing
		float temp = 0;
		std::string line; //string object for getline

		in.imbue(std::locale(std::locale(), new csv_reader())); //for the rest of stream, treat commas as whitespace
		if (!in.is_open()) {
			GD_ERROR("Failed to open file: {0} \n", fname);
			std::exit(EXIT_FAILURE);
		}

		std::getline(in, line);
		std::getline(in, line); //take second line incase of header

		int i = 0;
		while (i < strlen(delimeters)) {
			if (line.find(delimeters[i]) != std::string::npos)
				break;
			i++;
		}

		if (delimeters[i] == ' ')
		{
			GD_INFO("Delimeter detected: ' '", fname);
		}
		else
		{
			GD_INFO("Delimeter detected: {}", delimeters[i]);
		}

		in.close();
		return delimeters[i];
	}


	//IMPORT FUNCTIONS
	pcl::PointCloud<pcl::PointXYZ>::Ptr
		PCLreadASCIIxyz(char const* fname)
	{
		GD_TITLE("XYZ Data Import");
		auto start = GeoDetection::Time::getStart();

		//point cloud object that will be filled by import
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr(new pcl::PointCloud<pcl::PointXYZ>);

		//check that the ascii data has 3 columns (isXYZ)
		checkIfXYZ(fname);

		//check if file has a header
		bool header_present = checkIfHeader(fname);

		//Reserve space in cloud through number of lines and whether a header is present
		reserveCloudSpace(fname, header_present, cloudptr);

		//Read file into buffer (c-style because its faster) and import into pcl::PointCloud
		GD_TRACE(":: Reading Data...");

		FILE* file;
		errno_t err;

		if ((err = fopen_s(&file, fname, "r")) != 0)
		{
			GD_ERROR("Failed to open file: {0} \n", fname);
			std::exit(EXIT_FAILURE);
		}

		static const auto BUFFER_SIZE = 16 * 1024;
		char buf[BUFFER_SIZE + 1];
		pcl::PointXYZ mypoint;

		//If there's a header, skip the first line.
		if (header_present) {
			char* str = fgets(buf, BUFFER_SIZE, file);
		}

		while (size_t bytes_read = fread(&buf, 1, BUFFER_SIZE, file)) {

			char* p = buf; //pointer to the beginning of the line that we'll read

			if (bytes_read == std::numeric_limits<size_t>::max())
				GD_ERROR("Read failed!");
			if (!bytes_read) //not sure if this is neccessary
				break;

			for (char* p_next = buf; (p_next = (char*)memchr(p_next, '\n', (buf + bytes_read) - p_next)); p_next++) {
				auto answer = fast_float::from_chars(p, p_next, mypoint.x);
				answer = fast_float::from_chars(answer.ptr + 1, p_next, mypoint.y);
				fast_float::from_chars(answer.ptr + 1, p_next, mypoint.z);

				cloudptr->points.push_back(mypoint);

				p = p_next + 1;
			}
			// CASE: The xyz point is split across the buffer. Reset the file location to include it at the beginning of the next buffer fill.
			if (bytes_read == BUFFER_SIZE) {
				size_t offset = (buf + bytes_read) - p; //number of bytes from the end of buffer to the preceding char to the final \n. 
				fseek(file, (long)-offset, SEEK_CUR); //offset the stream back that many bytes, so we're at the final \n in the next memchr.
				continue;
			}
			// EOF CASE: The last line in the file didn't terminate with \n
			else if (cloudptr->points.size() < cloudptr->points.capacity()) {
				char* p_end; //pointer which lands on the delimeters.

				mypoint.x = strtod(p, &p_end);
				mypoint.y = strtod(p_end + 1, &p_end);
				mypoint.z = strtod(p_end + 1, NULL);
				cloudptr->points.push_back(mypoint);
			}
		}

		fclose(file);
		GD_WARN("--> Data import time: {0} ms", GeoDetection::Time::getDuration(start));

		return cloudptr;
	}

	void
		PCLwriteASCIIxyz(char const* fname, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudptr)
	{
		GD_TITLE("Exporting XYZ data...");
		auto start = GeoDetection::Time::getStart();

		static const auto BUFFER_SIZE = 16 * 1024;
		char buf[BUFFER_SIZE + 1];

		std::fstream fstream;
		fstream.open(fname, std::ios::out | std::ios::binary | std::ios::trunc);

		size_t i = 0;

		while (i < cloudptr->size()) {
			size_t cx = 0;
			size_t cx_increment = 0;

			while (true)
			{
				cx_increment = snprintf(buf + cx, BUFFER_SIZE - cx, "%.8f %.8f %.8f\n", cloudptr->points[i].x,
					cloudptr->points[i].y, cloudptr->points[i].z);

				cx += cx_increment;
				i++;

				if (cx >= BUFFER_SIZE) {
					i--; //Last point is inbetween the buffer; redo at the beginning of new buffer.
					break;
				}

				if (i == cloudptr->size()) {
					cx_increment = 0;
					break;
				}

			}

			fstream.write(buf, cx - cx_increment);
		}
		fstream.close();

		GD_WARN("--> Data export time: {0} ms", GeoDetection::Time::getDuration(start));
	}

}
