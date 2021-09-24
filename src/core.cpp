#include "core_impl.h"

namespace GeoDetection
{
	//Core template implementations
	template std::vector<size_t> sortIndicesDescending(const std::vector<int>& vec);
	template std::vector<size_t> sortIndicesDescending(const std::vector<float>& vec);

	template int computeMode(typename std::vector<int>::const_iterator, typename std::vector<int>::const_iterator);
	template float computeMode(typename std::vector<float>::const_iterator, typename std::vector<float>::const_iterator);

	template int computeModeFromIndices<int, int>(std::vector<int>&, std::vector<int>&);
	template float computeModeFromIndices<int, float>(std::vector<int>&, std::vector<float>&);

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

