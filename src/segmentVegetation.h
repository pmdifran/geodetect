#pragma once
#include "geodetect.h"

namespace geodetect
{
	/**
	* Segments vegetation using weighted differences of multiscale volumetric densities and curvatures.
	* @param[out] geodetect: geodetect::Cloud, where a vegetation index is appended to m_scalar_fields.
	*/
	void segmentVegetation(geodetect::Cloud& geodetect);

	/**
	* Segments vegetation using weighted differences of volumetric densities and curvatures, averaged at multiple scales.
	* @param[out] geodetect: geodetect::Cloud, where a vegetation index is appended to m_scalar_fields.
	*/
	void segmentVegetationSimplified(geodetect::Cloud& geodetect);
}