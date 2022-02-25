#pragma once
#include "geodetect.h"

namespace geodetect
{
	/**
	* Classifies points with respect to a mask. 
	* @param[in] mask: geodetect::Cloud which contains classifications at <m_scalar_fields.back()>.
	* @param[out] source: geodetect::Cloud which will be classified. Classifications are appended to the scalar field.
	* @param[in] num_neighbors: number of neighbors used for classification. If an even number is provided, we increase it by one,
	* so that all classifications are hard. 
	*/
	void classifyPoints(Cloud& mask, Cloud& source, int num_neighbors);

	/**
	* Classifies clusters with respect to a mask. 
	* @param[in] mask: geodetect::Cloud which contains classifications at <m_scalar_fields.back()>.
	* @param[out] source: geodetect::Cloud which will have its clusters classified. 
	* --> Cloud cluster IDs should be its last ScalarField (i.e. at the back).
	* @param[in] num_neighbors: number of neighbors used for classification. If an even number is provided, we increase it by one,
	* so that all classifications are hard.
	*/
	void classifyClusters(Cloud& mask, Cloud& source, int num_neighbors);
}