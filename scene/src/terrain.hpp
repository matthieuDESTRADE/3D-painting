#pragma once

#include "cgp/cgp.hpp"


struct SplinePoint {
  float x;
  float y;
};
float fade(float t);
float perlinNoise(float x);
float continuousFunction(float x);

std::tuple<float, bool> evaluate_terrain_height(float x, float y,  std::vector<cgp::vec2> splinePoints);
std::tuple<float, bool> evaluate_terrain_height(float x, float y);

std::vector<cgp::vec2> generateSplinePoints(const std::vector<cgp::vec2>& characteristicPoints, int numSegments);
std::vector<cgp::vec2> generateCurvedSplinePoints(const std::vector<cgp::vec2>& controlPoints, int numSegments, float tension);
/** Compute a terrain mesh 
	The (x,y) coordinates of the terrain are set in [-length/2, length/2].
	The z coordinates of the vertices are computed using evaluate_terrain_height(x,y).
	The vertices are sampled along a regular grid structure in (x,y) directions. 
	The total number of vertices is N*N (N along each direction x/y) 	*/
std::tuple<cgp::mesh, std::vector<cgp::vec3>> create_terrain_mesh(int N, float length);


