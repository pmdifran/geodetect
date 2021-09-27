#include "maskclassify.h"
#include "features.h" 
#include "core.h"

namespace geodetection
{
	//Classifies points with respect to a mask.
	//Classifies based on the mode of a local neighborhood (odd #) of nearest neighbors.
	//@TODO: Add option to specify location of mask classifications. 
	void classifyPoints(Cloud& mask, Cloud& source, int num_neighbors)
	{
		if (num_neighbors % 2) { num_neighbors++; } //hard classification using mode(neighborhood) --> always use odd numbers.
		
		auto source_cloud = source.cloud();
		auto source_scalarfields = source.scalarfields();

		std::vector<float> classification(source_cloud->size());

		//Get pointers to mask members
		auto mask_cloud = mask.cloud();
		auto mask_tree = mask.kdtree();
		auto mask_scalarfields = mask.scalarfields();

		//mask must have scalar fields. Assumed classes are the last scalar field.
		if (mask_scalarfields->size() == 0) { GD_ERROR("Mask cloud must have scalar fields of classes (last field assumed"); }
		ScalarField& mask_classes = mask_scalarfields->back();
		
		//Initialize containers
		std::vector<float> sqdistances(num_neighbors);
		std::vector<int> indices(num_neighbors);
		auto begin_it = indices.begin();
		auto end_it = indices.end();
		
		//Iterate through cloud and classify points based on the mode class within their k-nearest neighbors.
		for (size_t i = 0; i < source_cloud->size(); i++)
		{
			mask_tree->nearestKSearch(source_cloud->points[i], num_neighbors, indices, sqdistances);
			classification[i] = computeModeFromIndices<int, float>(indices, mask_classes.data);
		}

		source.addScalarField(std::move(classification));
	}

	//Classifies clusters with respect to a mask. 
	//Classifies based on the mode of a local neighborhood (odd #) of nearest neighbors.
	//@TODO: Add option to specify location of mask classifications. 
	//Assumes clusters are sorted (i.e. 1111, 2222, 33, 44444)
	//@TODO: Write a GeoDetection method which:
											//sorts all fields, points, normals, based on the entries of a single scalarfield (i.e. for cluster #). 
	void classifyClusters(Cloud& mask, Cloud& source, int num_neighbors)
	{
		ScalarField clusters_classified(source.cloud()->size());
		auto source_scalarfields = source.scalarfields(); //source scalar fields (clusterID is the last field).

		//classify the points based on nearest neighbor analysis. 
		classifyPoints(mask, source, num_neighbors);

		//get references to the classes and cluster ids, last and second to last, respectively.
		ScalarField& classes = source_scalarfields->rbegin()[0];
		ScalarField& clusters = source_scalarfields->rbegin()[1];

		//get list of clusters
		std::vector<float> cluster_ids = getUniqueList(clusters.data);

#pragma omp parallel for
		for (int64_t i = 0; i < cluster_ids.size(); i++)
		{
			float current_id = cluster_ids[i];

			//Get the array bounds of the current cluster
			std::vector<float>::const_iterator begin_it = std::find_if(clusters.begin(), clusters.end(), 
				[&current_id](float id) {return id == current_id; });

			std::vector<float>::const_iterator end_it = std::find_if(clusters.rbegin(), clusters.rend(), 
				[&current_id](float id) {return id == current_id; }).base();

			//get indices
			size_t index_begin = begin_it - clusters.begin();
			size_t index_end = end_it - clusters.begin();

			//get iterators to class bounds
			begin_it = classes.begin() + index_begin;
			end_it = classes.begin() + index_end;

			//compute cluster class
			float mode = computeMode<float>(begin_it, end_it);
			
			//set cluster classes.
			size_t index = index_begin;
			while (index < index_end)
			{
				clusters_classified[index] = mode;
				index++;
			}
		}

		source.addScalarField(std::move(clusters_classified));
	}

}