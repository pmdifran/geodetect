#pragma once
#include <vector>

namespace GeoDetection
{
	/**
	* Determines an index mapping to the input vector if it was sorted by descending values.
	* @param vec: Vector of arbitrary template type T.
	* @return Index map to a sorted version of the input vector (descending).
	*/
	template <typename T>
	std::vector<size_t> sortIndicesDescending(const std::vector<T>& vec);

	/**
	* Determines an index mapping to the input vector if it was sorted by ascending values.
	* @param vec: Vector of arbitrary template type T.
	* @return Index map to a sorted version of the input vector (ascending).
	*/
	template <typename T>
	std::vector<size_t> sortIndicesAscending(const std::vector<T>& vec);

	/**
	* Reorders a vector in place, given a mapping to the indices.
	* @param[out] vec: Vector of arbitrary template type T. This vector is sorted in place.
	* @param[out] sort_map: index vector of sorting indices. **This vector is also sorted in place.
	*/
	template <typename T>
	void reorderVector(std::vector<T>& vec, std::vector<size_t>& sort_map);

	/**
	* Inplace, reorders the sqdistances and indices resulting from an octree query, so that are acending in proximity of points.
	* @param[out] sqdistances: Vector of arbitrary template type T. This vector is sorted in place.
	* @param[out] indices: index vector of sorting indices. This vector is also sorted in place.
	*/
	void sortOctreeQuery(std::vector<int>& indices, std::vector<float>& sqdistances);

	/**
	* Assembles a contiguous array (i.e. vector) from vector subset indices.
	* @param indices: Subset indices of std::vector<T_field>
	* @param field: Vector of the (superset) data, of type T_field.
	* @return Subset of field vector, corresponding to indices, in the same order as they were provided.
	*/
	template <typename T_index, typename T_field>
	std::vector<T_field> vectorFromIndices(std::vector<T_index>& indices, std::vector<T_field>& field);

	/**
	* Returns a sorted copy of the vector, given begin and end iterators.
	* @param begin_it: Beginning iterator.
	* @param end_it: Ending iterator (i.e. past-the-end iterator).
	* @return A sorted copy of the vector.
	*/
	template <typename T>
	std::vector<T> vectorSortedCopy(typename std::vector<T>::const_iterator begin_it, typename std::vector<T>::const_iterator end_it);

	/**
	* Gets a unique list from a sorted vector.
	* @param vec: Sorted vector of type T.
	* @return A vector of type T, containing the unique entries.
	*/
	template <typename T>
	std::vector<T> getUniqueList(const std::vector<T>& vec);

	/**
	* Compute the mode across the range of an input vector.
	* @param vec: Unsorted, vector of type T. (This functions creates a sorted copy)
	* @return Type T mode. 
	*/
	template <typename T>
	T computeMode(std::vector<T>& vec);

	/**
	* Compute the mode across the contiguous range of a vector, of type T, given by iterators.
	* @param begin_it: Beginning iterator.
	* @param end_it: Ending iterator (i.e. past-the-end iterator).
	* @return Type T mode.
	*/
	template <typename T>
	T computeMode(typename std::vector<T>::const_iterator begin_it, typename std::vector<T>::const_iterator end_it);

	/**
	* Compute the mode for a subset of a vector, with the subset indices given.
	* @param indices: Vector of indices, of type T_index, pointing to a subset of <field>
	* @param field: Vector of the (superset) data, of type T_field.
	* @return Type T_field mode.
	*/
	template <typename T_index, typename T_field>
	T_field computeModeFromIndices(std::vector<T_index>& indices, std::vector<T_field>& field);
}