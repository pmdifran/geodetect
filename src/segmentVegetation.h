#pragma once
#include "GeoDetection.h"

namespace geodetection
{
	/**
	* Segments vegetation using weighted differences of multiscale volumetric densities and curvatures.
	* @param[out] geodetect: geodetection::Cloud, where a vegetation index is appended to m_scalar_fields.
	*/
	void segmentVegetation(geodetection::Cloud& geodetect);

	/**
	* Segments vegetation using weighted differences of volumetric densities and curvatures, averaged at multiple scales.
	* @param[out] geodetect: geodetection::Cloud, where a vegetation index is appended to m_scalar_fields.
	*/
	void segmentVegetationSimplified(geodetection::Cloud& geodetect);
}