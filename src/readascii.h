#pragma once
#include "log.h"
#include "readascii_core.h"
#include "GeoDetection.h"
#include "ScalarField.h"

#include <string>


#include <pcl\point_types.h>
#include <pcl\point_cloud.h>

namespace GeoDetection
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

		GeoDetection::Cloud import();
	};

}