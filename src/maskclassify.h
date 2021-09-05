#pragma once
#include "GeoDetection.h"

namespace GeoDetection
{
	void classify(Cloud& mask, Cloud& source, int num_neighbors, size_t mask_field_index = 0);
}