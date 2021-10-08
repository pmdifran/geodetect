#include "core_impl.h"

namespace geodetection
{
	//Core template implementations
	template std::vector<size_t> sortIndicesDescending(const std::vector<int>& vec);
	template std::vector<size_t> sortIndicesDescending(const std::vector<float>& vec);

	template double getStandardDeviation(const std::vector<double>& vec, double mean);
	template double getStandardDeviation(const std::vector<float>& vec, float mean);

	template float getNonZeroMinimum(const std::vector<float>& vec);
	template double getNonZeroMinimum(const std::vector<double>& vec);

	template std::vector<double> vectorGetSquared<float, double>(const std::vector<float>& vec);
	template std::vector<double> vectorGetSphereVolumes(const std::vector<float>& scales);

	template std::vector<int> getProximalIndices(const std::vector<int>& indices, const std::vector<float>& distances, float max_distance);

	template int computeMode(typename std::vector<int>::const_iterator, typename std::vector<int>::const_iterator);
	template float computeMode(typename std::vector<float>::const_iterator, typename std::vector<float>::const_iterator);

	template int computeModeFromIndices<int, int>(std::vector<int>&, std::vector<int>&);
	template float computeModeFromIndices<int, float>(std::vector<int>&, std::vector<float>&);

	//Determines the standard deviation of a vector.
	double getStandardDeviation(const std::vector<float>& vec, double mean)
	{
		double variance = 0.0;
		int64_t size = vec.size();

#pragma omp parallel for reduction(+: variance)
		for (int64_t i = 0; i < size; i++)
		{
			variance += pow(vec[i] - mean, 2);
		}

		return sqrt(variance / size);
	}

	//Inplace, reorders the sqdistances and indices resulting from an octree query, so that are acending in proximity of points.
	void sortOctreeQuery(std::vector<int>& indices, std::vector<float>& sqdistances)
	{
		// get sort map for most proximal neighbors. 
		std::vector<size_t> sort_map = sortIndicesAscending(sqdistances);

		// sort sqdistances in place
		std::sort(sqdistances.begin(), sqdistances.end());

		// reorder indices in place based on sort map
		reorderVector(indices, sort_map);
	}
}

