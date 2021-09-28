#pragma once
#include "core.h"
#include <algorithm>
#include <numeric>
#include <cassert>

#define _USE_MATH_DEFINES
#include <math.h>

namespace geodetection
{
	//Gets index mapping to the input vector if it was sorted by descending values
	template <typename T>
	std::vector<size_t>
		sortIndicesDescending(const std::vector<T>& vec)
	{
		std::vector<size_t> idx(vec.size());
		std::iota(idx.begin(), idx.end(), 0);
		std::stable_sort(idx.begin(), idx.end(), [&vec](size_t i1, size_t i2) {return vec[i1] > vec[i2]; });
		return idx;
	}

	//Gets index mapping to the input vector if it was sorted by ascending values
	template <typename T>
	std::vector<size_t>
		sortIndicesAscending(const std::vector<T>& vec)
	{
		std::vector<size_t> idx(vec.size());
		std::iota(idx.begin(), idx.end(), 0);
		std::stable_sort(idx.begin(), idx.end(), [&vec](size_t i1, size_t i2) {return vec[i1] < vec[i2]; });
		return idx;
	}

	//Returns a reordered vector, given a mapping to the indices.
	//Specifically useful to reorder sqdistances and indices following an octree neighborhood query, since they are not ordered.
	//--> Must input std::vector<size_t> from sortIndicesDescending or sortIndicesAscending
	template <typename T>
	void reorderVector(std::vector<T>& vec, std::vector<size_t>& order)
	{
		assert(vec.size() == order.size());

		// for all elements to put in place (last element falls into place)
		for (size_t i = 0; i < vec.size() - 1; ++i)
		{
			// while the element i is not yet in place 
			while (i != order[i])
			{
				// swap it with the element at its final place
				size_t alt = order[i];
				std::swap(vec[i], vec[alt]);
				std::swap(order[i], order[alt]);
			}
		}
	}

	//Returns a subset of the indices, who correspond to distance < max_distance.
	template <typename T_index, typename T_distance>
	std::vector<T_index> getProximalIndices(const std::vector<T_index>& indices, const std::vector<T_distance>& distances, float max_distance)
	{
		//Find first element (iterator) that is less than the scale.
		 auto iter_end = std::find_if(distances.begin(), distances.end(), [&max_distance](T_distance x)
			{return x > max_distance; });

		//Get subset of new indices.
		size_t subcloud_end = iter_end - distances.begin();
		std::vector<T_index> subindices(indices.begin(), indices.begin() + subcloud_end);

		return subindices;
	}

	//Assemble contiguous array of fields from indices
	template <typename T_index, typename T_field>
	std::vector<T_field>
		vectorFromIndices(std::vector<T_index>& indices, std::vector<T_field>& field)
	{
		std::vector<T_field> vec;
		vec.reserve(indices.size());

		for (auto it = indices.begin(); it != indices.end(); it++)
		{
			vec.push_back(field[*it]);
		}
		return vec;
	}

	//Returns a sorted copy of the vector, given begin and end iterators.
	template <typename T>
	std::vector<T> vectorSortedCopy(typename std::vector<T>::const_iterator begin_it, typename std::vector<T>::const_iterator end_it)
	{
		std::vector<T> vec(end_it - begin_it);
		std::copy(begin_it, end_it, vec.begin());
		std::sort(vec.begin(), vec.end());
		return vec;
	}

	//Returns a squared copy of the vector.
	template <typename T_in, typename T_out>
	std::vector<T_out> vectorGetSquared(const std::vector<T_in>& vec)
	{
		std::vector<T_out> sqvec;
		sqvec.reserve(vec.size());
		
		for (T_in x : vec) { sqvec.push_back(x * x); }
		return sqvec;
	}

	//Returns Sphere volume, given a scale.
	template <typename T>
	T getSphereVolume(T scale)
	{
		return (T)((4.0f / 3.0f) * M_PI * pow(scale, 3));
	}

	//Returns vector of sphere volumes, given vector of scales.
	template <typename T_in>
	std::vector<double> vectorGetSphereVolumes(const std::vector<T_in>& scales)
	{
		std::vector<double> volumes;
		volumes.reserve(scales.size());

		for (T_in x : scales) { volumes.push_back(getSphereVolume<T_in>(x)); }
		return volumes;
	}

	//Gets a unique list from a sorted vector.
	template <typename T>
	std::vector<T> getUniqueList(const std::vector<T>& vec)
	{
		//Get list of classes in the SF
		std::vector<T> unique_list = vec;

		auto new_end = std::unique(unique_list.begin(), unique_list.end());
		unique_list.resize(new_end - unique_list.begin());

		return unique_list;
	}

	//Compute the mode across the range of an input vector.
	//The input vector is sorted when we determine the unique_list (i.e. input can be unsorted).
	template <typename T>
	T
		computeMode(std::vector<T>& vec)
	{
		std::sort(vec.begin(), vec.end());
		std::vector<T> unique_list = getUniqueList(vec);
		if (unique_list.size() == 1) { return unique_list[0]; }

		std::vector<size_t> counts(unique_list.size(), 0);

		for (size_t i = 0; i < unique_list.size(); i++)
		{
			T value = unique_list[i];
			auto begin_it = std::find_if(vec.begin(), vec.end(), [&value](T element) {return element == value; });
			auto end_it = std::find_if(vec.rbegin(), vec.rend(), [&value](T element) {return element == value; }).base();

			while (begin_it != end_it)
			{
				counts[i]++;
				begin_it++;
			}
		}

		size_t max_index = std::max_element(counts.begin(), counts.end()) - counts.begin();
		T mode = unique_list[max_index];
		return mode;
	}

	//Compute the mode across the contiguous range of a vector, of type T, given by iterators.
	//The input is sorted when we determine the unique_list (i.e. input can be unsorted).
	template <typename T>
	T
		computeMode(typename std::vector<T>::const_iterator begin_it, typename std::vector<T>::const_iterator end_it)
	{
		std::vector<T> vec = vectorSortedCopy<T>(begin_it, end_it);
		std::vector<T> unique_list = getUniqueList(vec);
		if (unique_list.size() == 1) { return unique_list[0]; }

		std::vector<size_t> counts(unique_list.size(), 0);

		for (size_t i = 0; i < unique_list.size(); i++)
		{
			T value = unique_list[i];
			typename std::vector<T>::iterator begin_it = std::find_if(vec.begin(), vec.end(), [&value](T element) {return element == value; });
			typename std::vector<T>::const_iterator end_it = std::find_if(vec.rbegin(), vec.rend(), [&value](T element) {return element == value; }).base();

			while (begin_it != end_it)
			{
				counts[i]++;
				begin_it++;
			}
		}

		size_t max_index = std::max_element(counts.begin(), counts.end()) - counts.begin();
		T mode = unique_list[max_index];
		return mode;
	}

	//Compute the mode for a subset of a vector, with the subset indices given.
	//The input is sorted when we determine the unique_list (i.e. input can be unsorted).
	template <typename T_index, typename T_field>
	T_field
		computeModeFromIndices(std::vector<T_index>& indices, std::vector<T_field>& field)
	{
		std::vector<T_field> vec = vectorFromIndices(indices, field);
		return computeMode(vec);
	}
}