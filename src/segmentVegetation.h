#pragma once
#include "GeoDetection.h"

namespace GeoDetection
{
	/**
	* Segments vegetation using weighted differences of multiscale volumetric densities and curvatures.
	* @param[out] geodetect: GeoDetection::Cloud, where a vegetation index is appended to m_scalar_fields.
	*/
	void segmentVegetation(GeoDetection::Cloud& geodetect);

	/**
	* Segments vegetation using weighted differences of volumetric densities and curvatures, averaged at multiple scales.
	* @param[out] geodetect: GeoDetection::Cloud, where a vegetation index is appended to m_scalar_fields.
	*/
	void segmentVegetationSimplified(GeoDetection::Cloud& geodetect);
}