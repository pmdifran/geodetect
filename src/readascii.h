#pragma once
#include "log.h"
#include "readascii_core.h"
#include "GeoDetection.h"
#include "ScalarField.h"


#include <pcl\point_types.h>
#include <pcl\point_cloud.h>

namespace GeoDetection
{
	class ReadAscii
	{
		const char* m_filename;

	public: 
		ReadAscii(const char* f) : m_filename(f)
		{
		}

		GeoDetection::Cloud import();

	};

}
