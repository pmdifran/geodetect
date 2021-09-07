#pragma once
#include "GeoDetection.h"

namespace GeoDetection
{
	void classifyPoints(Cloud& mask, Cloud& source, int num_neighbors, size_t mask_field_index = 0);

	void classifyClusters(Cloud& mask, Cloud& source, int num_neighbors, size_t mask_field_index = 0);
}