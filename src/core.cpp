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
}

