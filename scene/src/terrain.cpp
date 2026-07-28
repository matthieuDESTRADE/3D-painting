
#include "terrain.hpp"
#include "interpolation.hpp"
using namespace cgp;


#include <vector>


std::vector<vec2> generateCurvedSplinePoints(const std::vector<vec2>& controlPoints, int numSegments, float tension) {
  std::vector<vec2> splinePoints;

  // Check if the number of segments is sufficient
  if (numSegments < 1 || controlPoints.size() < 2) {
    return splinePoints;
  }

  // Calculate the number of control points
  int numControlPoints = controlPoints.size();

  // Generate the spline points
  for (int i = 0; i < numControlPoints - 1; ++i) {
    // Calculate the control points for the current segment
    vec2 p0 = controlPoints[i];
    vec2 p1 = controlPoints[i + 1];

    // Calculate the tangent vectors for the current segment
    vec2 tangent0 = tension * (p1 - controlPoints[i > 0 ? i - 1 : 0]);
    vec2 tangent1 = tension * (controlPoints[i < numControlPoints - 2 ? i + 2 : numControlPoints - 1] - p0);

    // Generate points along the current segment
    for (int j = 0; j < numSegments; ++j) {
      float t = static_cast<float>(j)/static_cast<float>(numSegments);
      float tSquared = t * t;
      float tCubed = tSquared * t;

      // Calculate the position on the cardinal spline
      vec2 position = (2.0f * tCubed - 3.0f * tSquared + 1.0f) * p0 +
                         (tCubed - 2.0f * tSquared + t) * tangent0 +
                         (-2.0f * tCubed + 3.0f * tSquared) * p1 +
                         (tCubed - tSquared) * tangent1;

      splinePoints.push_back(position);
    }
  }
  return splinePoints;
}



std::vector<cgp::vec2> generateSplinePoints(const std::vector<cgp::vec2>& characteristicPoints, int numSegments) {
  std::vector<cgp::vec2> splinePoints;

  // Check if the number of segments is sufficient
  if (numSegments < 1 || characteristicPoints.size() < 2) {
    return splinePoints;
  }

  // Calculate the step for linear interpolation
  float step = 1.0f / static_cast<float>(numSegments);

  // Iterate through the characteristic points of the spline
  for (size_t i = 0; i < characteristicPoints.size() - 1; ++i) {
    const vec2& startPoint = characteristicPoints[i];
    const vec2& endPoint = characteristicPoints[i + 1];

    // Calculate the difference between the x and y coordinates of the characteristic points
    vec2 delta = endPoint - startPoint;

    // Iterate through the segments and interpolate the intermediate points
    for (int j = 0; j < numSegments; ++j) {
      float t = step * static_cast<float>(j);

      // Calculate the interpolated point using linear interpolation
      vec2 interpolatedPoint = startPoint + t * delta;

      // Add the interpolated point to the list of spline points
      splinePoints.push_back(interpolatedPoint);
    }
  }

  return splinePoints;
}

float continuousFunction(float x) {
  // Scale the input to the desired range (0 to 1)
  float scaledX = (x - 0) / (1 - 0);

  // Map the scaled value to the desired range (0.02 to 0.1)
  float mappedValue = 0.04 * sin(2 * 3.1415 * scaledX) + 0.06;

  return mappedValue;
}



// Evaluate 3D position of the terrain for any (x,y)
std::tuple<float, bool> evaluate_terrain_height(float x, float y, std::vector<vec2> splinePoints)
{
    bool MyBoolean = true;
    vec2 p_i[6] = { {0,0}, {0.2f,0.2f}, {0.5f,-0.5f}, {0.6f,-0.25f} , {-0.5f, -0.2f}, {-0.6f, -0.4f} };
    float h_i[6] = { 0.2f, 0.1f, 0.15f, 0.1f, 0.06f, 0.13f };
    float sigma_i[6] = { 0.6f,0.6f,0.3f,0.2f, 0.4f, 0.3f };
    float z = 0.0f;

    for (int i = 0; i < 6; i++) {
        float d = norm(vec2(x, y) - p_i[i]) / sigma_i[i];
        z += h_i[i] * std::exp(-d * d);
    }
    float d = norm(vec2(x, y)) / 3.0f;
    z += .5f - .5f * std::exp(-d * d);
    z += std::min(.1f,d/5.f) * (noise_perlin({x / 2.f,y / 2.f}) - 0.5f);

    return std::make_tuple(z, MyBoolean);
}

std::tuple<float, bool> evaluate_terrain_height(float x, float y)
{
    bool MyBoolean = true;
    vec2 p_i[6] = { {0,0}, {0.2f,0.2f}, {0.5f,-0.5f}, {0.6f,-0.25f} , {-0.5f, -0.2f}, {-0.6f, -0.4f} };
    float h_i[6] = { 0.2f, 0.1f, 0.15f, 0.1f, 0.06f, 0.13f };
    float sigma_i[6] = { 0.6f,0.6f,0.3f,0.2f, 0.4f, 0.3f };
    float z = 0.0f;

    for (int i = 0; i < 6; i++) {
        float d = norm(vec2(x, y) - p_i[i]) / sigma_i[i];
        z += h_i[i] * std::exp(-d * d);
    }
    float d = norm(vec2(x, y))/3.0f;
    z += .5f - .5f * std::exp(-d * d);
    z += std::min(.1f, d/5.f) *(noise_perlin({ x/2.f,y/2.f })-0.5f);

    return std::make_tuple(z, MyBoolean);
}

std::tuple<mesh, std::vector<vec3>> create_terrain_mesh(int N, float terrain_length)
{
    std::vector<vec3> ground_pos;
    mesh terrain; // temporary terrai storage (CPU only)
    terrain.position.resize(N*N);
    //Spline points of the path
    std::vector<vec2> drop_i = {{0.8f,0.8f},{0.6f,0.4f}, {0.4f,0.1f}, {0.4f,-0.1f}, {0.2f,-0.2f}, {0.1f,-0.3f}, {-0.1f,-0.5f},{-0.7f,-0.7f}, {-0.6f,-0.4f}, {-0.62f,-0.35f},{-0.65f,-0.4f},{-0.2f,0.3f}, {0.0f,0.4f}, {0.1f,0.5f}, {0.3f,0.5f}, {0.6f,0.4f}, {0.8f,0.8f}, {0.6f,0.7f}, {0.5f,0.6f}, {0.5f,0.5f}};
    int numSegments = 20;
    std::vector<vec2> splinePoints = generateCurvedSplinePoints(drop_i, numSegments,1);

    // Fill terrain geometry
    for(int ku=0; ku<N; ++ku)
    {
        for(int kv=0; kv<N; ++kv)
        {
            // Compute local parametric coordinates (u,v) \in [0,1]
            float u = ku/(N-1.0f);
            float v = kv/(N-1.0f);

            // Compute the real coordinates (x,y) of the terrain in [-terrain_length/2, +terrain_length/2]
            float x = (u - 0.5f) * terrain_length;
            float y = (v - 0.5f) * terrain_length;

            // Compute the surface height function at the given sampled coordinate
            std::tuple<float, bool> result = evaluate_terrain_height(x,y,splinePoints);
            float const z = std::get<0>(result)*2;
            bool MyBoolean = std::get<1>(result);
            ground_pos.push_back(vec3(x,y,z));
            if (MyBoolean){
                float const noise = noise_perlin({u, v}, 5, 0.8f, 2.0f);
                // use the noise as height value
			    terrain.position[kv+N*ku] = {x,y,z+0.02*std::abs(noise)};
			    // use also the noise as color value
			    // terrain.color[kv+N*ku] = 0.3f*vec3(0,0.5f,0)+0.7f*noise*vec3(1,1,1);
            } else{
                // Store vertex coordinates
                terrain.position[kv+N*ku] = {x,y,z};
            }
            
        }
    }

    // Generate triangle organization
    //  Parametric surface with uniform grid sampling: generate 2 triangles for each grid cell
    for(int ku=0; ku<N-1; ++ku)
    {
        for(int kv=0; kv<N-1; ++kv)
        {
            unsigned int idx = kv + N*ku; // current vertex offset

            uint3 triangle_1 = {idx, idx+1+N, idx+1};
            uint3 triangle_2 = {idx, idx+N, idx+1+N};

            terrain.connectivity.push_back(triangle_1);
            terrain.connectivity.push_back(triangle_2);
        }
    }

    // need to call this function to fill the other buffer with default values (normal, color, etc)
	terrain.fill_empty_field(); 

    return std::make_tuple(terrain, ground_pos);

}