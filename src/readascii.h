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
		//Import() uses c-style IO, so we need to provide filenames as c-strings
		const char* m_filename;

	public: 
		AsciiReader() = default;
		AsciiReader(const char* f) : m_filename(f) {}
		AsciiReader(std::string fname_str) : m_filename(fname_str.c_str()) {}
		
		inline void setFilename(const char* fname) { m_filename = fname; }
		inline void setFilename(const std::string fname_str) { m_filename = fname_str.c_str(); }

		GeoDetection::Cloud import();
	};

}