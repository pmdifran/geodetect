#include "ScalarField.h"
#include <algorithm>

namespace geodetection
{
	//Modify scalar field data with min-max normalization
	void
		ScalarField::normalizeMinMax()
	{
		auto min = std::min_element(data.begin(), data.end()); //returns iterator
		auto max = std::max_element(data.begin(), data.end());

		float range = *max - *min;
		if (range == 0) { return;  } //if all scalarfields are 0, leave them.

#pragma omp parallel for
		for (int64_t i = 0; i < (int64_t)data.size(); i++)
		{
			data[i] = (data[i] - *min) / range;
		}
	}

	//Make all NaN data equal to zero
	void
		ScalarField::NaNtoValue(float value)
	{
#pragma omp parallel for
		for (int64_t i = 0; i < (int64_t)data.size(); i++)
		{
			if (data[i] == data[i]) { continue; } //false if is NaN
			data[i] = value;
		}
	}
}