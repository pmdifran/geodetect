#include "maskclassify.h"
#include "features.h" 

std::vector<float> getUniqueList(std::vector<float>& fields)
{
	//Get list of classes in the SF
	std::vector<float> class_list(fields.size());
	std::partial_sort_copy(fields.begin(), fields.end(), class_list.begin(), class_list.end());
	auto new_end = std::unique(class_list.begin(), class_list.end());
	
	class_list.resize(new_end - class_list.begin());

	return class_list;
}

float computeMode(std::vector<int>::iterator& begin_it, std::vector<int>::iterator& end_it, GeoDetection::ScalarField& sf)
{
	//Get the query of classes into a separete vector
	std::vector<float> classes(end_it - begin_it);
	for (auto it = begin_it; it != end_it; it++)
	{
		classes.push_back(sf[*it]);
	}

	//Get list of unique classes to search for as mode and count number of times they're repeated.
	std::vector<float> mode_candidates = getUniqueList(classes);
	std::vector<int> counts(mode_candidates.size(), 0);

	//Compute counts for each mode candidate
	for (size_t i = 0; i < classes.size(); i++)
	{
		for (size_t j = 0; j < mode_candidates.size(); j++)
		{
			if (classes[i] == mode_candidates[j]) { counts[j]++; }
		}
	}
	
	return *(std::max_element(counts.begin(), counts.end()));
}

namespace GeoDetection
{
	void classify(Cloud& mask, Cloud& source, int num_neighbors, size_t mask_field_index /* = 0 */)
	{
		if (num_neighbors % 2) { num_neighbors++; } //hard classification using mode(neighborhood) --> always use odd numbers.

		
		auto source_cloud = source.cloud();
		std::vector<float> classification(source_cloud->size());

		//Get pointers to mask members
		auto mask_cloud = mask.cloud();
		auto mask_tree = mask.tree();
		auto mask_scalarfields = mask.scalarfields();

		//Get pointer to classes
		ScalarField* mask_classes = &mask_scalarfields->at(mask_field_index);

		//Initialize containers
		std::vector<float> sqdistances(num_neighbors);
		std::vector<int> indices(num_neighbors);
		auto begin_it = indices.begin();
		auto end_it = indices.end();
		
		//Iterate through cloud and classify points based on the mode class within their k-nearest neighbors.
		for (size_t i = 0; i < source_cloud->size(); i++)
		{
			mask_tree->nearestKSearch(source_cloud->points[i], num_neighbors, indices, sqdistances);
			classification[i] = computeMode(begin_it, end_it, *mask_classes);
		}
	}


}