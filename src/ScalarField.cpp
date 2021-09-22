#include "ScalarField.h"
#include <algorithm>

namespace GeoDetection
{
	//Modify scalar field data with min-max normalization
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

	//Make all NaN data equal to the max
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

	//Make all NaN data equal to the min
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

	//Make all NaN data equal to zero
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