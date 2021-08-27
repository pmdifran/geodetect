#pragma once
#include "GeoDetection.h"

namespace GeoDetection
{
	void segmentVegetation(GeoDetection::Cloud& geodetect);

	void segmentVegetationAveraging(GeoDetection::Cloud& geodetect);
}