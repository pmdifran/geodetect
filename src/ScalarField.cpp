#include "ScalarField.h"
#include <algorithm>

namespace GeoDetection
{
	void
		ScalarField::normalizeMinMax()
	{
		const auto min = std::min_element(data.begin(), data.end()); //returns iterator
		const auto max = std::max_element(data.begin(), data.end());

		const float range = *max - *min;

#pragma omp parallel for
		for (size_t i = 0; i < data.size(); i++)
		{
			data[i] = (data[i] - *min) / range;
		}
	}
}