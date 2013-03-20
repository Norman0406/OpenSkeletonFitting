#pragma once

namespace osf
{
	class FeaturePoint;
	class Joint;
	
	double cmEnergyAffectors(const Joint*, void*);
	double cmEnergyNearestNeighbor(const Joint*, void*);
	double cmEnergyUnderlyingPoint(const Joint*, void*);
}