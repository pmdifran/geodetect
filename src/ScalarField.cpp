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
		for (int64_t i = 0; i < (int64_t)data.size(); i++)
		{
			data[i] = (data[i] - *min) / range;
		}
	}

	void 
		ScalarField::NaNtoMax()
	{
		const auto max = *(std::max_element(data.begin(), data.end()));
#pragma omp parallel for
		for (int64_t i = 0; i < (int64_t)data.size(); i++)
		{
			if (data[i] == data[i]) { continue; } //false if is NaN
			data[i] = max;
		}
	}

	void
		ScalarField::NaNtoMin()
	{
		const auto min = *(std::min_element(data.begin(), data.end()));
#pragma omp parallel for
		for (int64_t i = 0; i < (int64_t)data.size(); i++)
		{
			if (data[i] == data[i]) { continue; } //false if is NaN
			data[i] = min;
		}
	}

	void
		ScalarField::NaNtoZero()
	{
#pragma omp parallel for
		for (int64_t i = 0; i < (int64_t)data.size(); i++)
		{
			if (data[i] == data[i]) { continue; } //false if is NaN
			data[i] = 0;
		}
	}
}