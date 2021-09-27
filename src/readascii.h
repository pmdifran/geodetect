#pragma once
#include "log.h"
#include "readascii_core.h"
#include "GeoDetection.h"
#include "ScalarField.h"

#include <string>

//PCL point cloud and point types.
#include <pcl\point_types.h>
#include <pcl\point_cloud.h>

namespace geodetection
{
	class AsciiReader
	{
		std::string m_filename;

	public: 
		AsciiReader() = default;
		AsciiReader(const char* f) : m_filename(std::string(f)) {}
		AsciiReader(std::string f) : m_filename(f) {}
		
		inline void setFilename(const char* fname) { m_filename = std::string(fname); }
		inline void setFilename(const std::string fname) { m_filename = fname; }

		/**
		* Import ASCII data at m_filename, using c-style file reading.
		*/
		geodetection::Cloud import() const;
	};

}