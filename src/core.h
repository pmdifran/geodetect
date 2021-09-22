#pragma once
#include <vector>

namespace GeoDetection
{
	//Gets index mapping to the input vector if it was sorted by descending values
	template <typename T>
	std::vector<size_t> sortIndicesDescending(const std::vector<T>& vec);

	//Gets index mapping to the input vector if it was sorted by ascending values
	template <typename T>
	std::vector<size_t> sortIndicesAscending(const std::vector<T>& vec);

	//Assemble contiguous array of fields from indices
	template <typename T_index, typename T_field>
	std::vector<T_field> vectorFromIndices(std::vector<T_index>& indices, std::vector<T_field>& field);

	template <typename T>
	std::vector<T> vectorSortedCopy(typename std::vector<T>::const_iterator begin_it, typename std::vector<T>::const_iterator end_it);

	//Get unique lists from a sorted vector
	template <typename T>
	std::vector<T> getUniqueList(const std::vector<T>& vec);

	//Compute the mode across the range of an input vector.
	template <typename T>
	T computeMode(std::vector<T>& vec);

	//Compute the mode across the contiguous range of a vector given by iterators.
	template <typename T>
	T computeMode(typename std::vector<T>::const_iterator begin_it, typename std::vector<T>::const_iterator end_it);

	template <typename T_index, typename T_field>
	T_field computeModeFromIndices(std::vector<T_index>& indices, std::vector<T_field>& field);
}