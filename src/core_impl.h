#pragma once
#include "core.h"
#include <algorithm>
#include <numeric>

namespace GeoDetection
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

	template <typename T>
	std::vector<T> vectorSortedCopy(typename std::vector<T>::const_iterator begin_it, typename std::vector<T>::const_iterator end_it)
	{
		std::vector<T> vec(end_it - begin_it);
		std::copy(begin_it, end_it, vec.begin());
		std::sort(vec.begin(), vec.end());
		return vec;
	}

	//Get unique lists from a sorted vector
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

	//Compute the mode across the contiguous range of a vector given by iterators.
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

	template <typename T_index, typename T_field>
	T_field
		computeModeFromIndices(std::vector<T_index>& indices, std::vector<T_field>& field)
	{
		std::vector<T_field> vec = vectorFromIndices(indices, field);
		return computeMode(vec);
	}
}