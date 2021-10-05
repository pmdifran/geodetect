#include "readascii_core.h"
#include "readascii.h"
#include <fast_float/fast_float.h> // for fast parsing (similar to atof).

namespace geodetection
{
	//Helper to parse the file and return a geodetection Cloud. Currently only imports scalar fields and XYZ coordinates.
	//@TODO: Add option for inputing normals, and specifying their location.
	Cloud 
		parseData(FILE* file, size_t num_points, size_t num_columns, bool header_present)
	{
		size_t num_fields = num_columns - 3;

		//Get pointers to all of the member variables, which we will fill. 
		Cloud geodetect;
		auto cloud = geodetect.cloud();
		auto normals = geodetect.normals();
		auto scalarfields = geodetect.scalarfields();

		//Reserve space, considering num_columns and num_points
		cloud->points.reserve(num_points);
		normals->points.reserve(num_points);

		// reserve space for all the scalar fields
		for (size_t i = 0; i < num_fields; i++) 
		{ 
			scalarfields->emplace_back().data.reserve(num_points);
		}

		//temporary containers for parsing and pushing into classes
		pcl::PointXYZ xyz;
		pcl::Normal n;
		float sf; 
		
		GD_CORE_TRACE(":: Parsing ascii file and filling empty GeoDetection Cloud...\n \n");
		//initialize buffer
		static const int64_t BUFFER_SIZE = 16 * 1024;
		char buf[BUFFER_SIZE + 1];

		//If there's a header, skip the first line.
		if (header_present) {
			char* str = fgets(buf, BUFFER_SIZE, file);
		}
		
		//Fill buffer until the file is read.
		while (size_t bytes_read = fread(&buf, 1, BUFFER_SIZE, file)) {

			char* p = buf; //pointer location in buffer

			if (bytes_read == std::numeric_limits<size_t>::max())
				GD_ERROR("Read failed!");
			if (!bytes_read) //zero bytes read (at end of file)
				break;

			//p_next is the pointer to the next '\n' in the buffer
			for (char* p_next = buf; (p_next = (char*)memchr(p_next, '\n', (buf + bytes_read) - p_next)); p_next++) {
				
				//Parse first 3 columns (xyz)
				auto result = fast_float::from_chars(p, p_next, xyz.x);
				result = fast_float::from_chars(result.ptr + 1, p_next, xyz.y);
				result = fast_float::from_chars(result.ptr + 1, p_next, xyz.z);
				cloud->points.push_back(xyz);

				for (size_t i = 0; i < num_fields; i++)
				{
					result = fast_float::from_chars(result.ptr + 1, p_next, sf);
					scalarfields->at(i).data.push_back(sf);
				}

				p = p_next + 1;
			}
			// CASE: The xyz point is split across the buffer. Reset the file location to include it at the beginning of the next buffer fill.
			if (bytes_read == BUFFER_SIZE) {
				size_t offset = (buf + bytes_read) - p; //number of bytes from end of buffer to the last read `\n`. 
				fseek(file, (long)-offset, SEEK_CUR); //move file pointer to the beginning of the point for next buffer fill.
				continue;
			}
			// EOF CASE: The last line in the file didn't terminate with \n
			else if (cloud->points.size() < num_points) {
				char* p_end; //pointer which lands on the delimeters.

				xyz.x = strtod(p, &p_end);
				xyz.y = strtod(p_end + 1, &p_end);
				xyz.z = strtod(p_end + 1, &p_end);
				cloud->points.push_back(xyz);

				for (size_t i = 0; i < num_fields; i++) {scalarfields[i].push_back(strtod(p_end + 1, &p_end));}
			}
		}
		return geodetect;
	}

	//Import ASCII data at m_filename, using c - style file reading.
	//Mainly uses the parse data function above, and some read ascii core functions for getting line count, number of columns etc. 
	Cloud 
		AsciiReader::import() const
	{
			GD_TITLE("Ascii Data Import");
			GD_TRACE(":: File: {}", m_filename);
			Timer timer;

			//Get number of points in the file, and reduce by one if header is present.
			bool header_present = hasHeader(m_filename);
			size_t num_points = getLineCount(m_filename) - (int)header_present;
			GD_TRACE(":: Number of points: {0}", num_points);

			//Get number of columns in the file
			size_t num_columns = getNumColumns(m_filename);
			GD_TRACE(":: Number of columns: {0}",  num_columns);

			//Open the c-style file for reading
			const char* fname_cstr = m_filename.c_str();
			FILE* file;
			errno_t err;

			if ((err = fopen_s(&file, fname_cstr, "r")) != 0)
			{
				GD_ERROR("Failed to open file: {0} \n", m_filename);
				std::exit(EXIT_FAILURE);
			}

			//Parse the data and create a GeoDetection object
			Cloud geodetect = parseData(file, num_points, num_columns, header_present);

			fclose(file);
			GD_WARN("--> Data import time: {0} ms", timer.getDuration());

			//Build the KdTree. geodetection Clouds use both KdTrees and Octrees. 
			geodetect.buildKdTree();

			return geodetect;
	}

}