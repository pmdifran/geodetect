#pragma once
#include "log.h"
#include "readascii_core.h"
#include "GeoDetection.h"
#include "ScalarField.h"


#include <pcl\point_types.h>
#include <pcl\point_cloud.h>

namespace GeoDetection
{
	class AsciiReader
	{
		const char* m_filename;

	public: 
		AsciiReader(const char* f) : m_filename(f) {}

		inline void setFilename(const char* fname) { m_filename = fname; }
		GeoDetection::Cloud import();
	};

}
