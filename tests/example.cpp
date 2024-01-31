/*
 * This file is part of the TQMesh library.
 * This code was written by Florian Setzwein in 2022,
 * and is covered under the MIT License
 * Refer to the accompanying documentation for details
 * on usage and license.
 */

#include <cassert>
#include <iostream>

#include <Eigen/Eigen>
#include <fcpw/fcpw.h>

/*********************************************************************
 * This example covers the generation of a simple triangular mesh
 *********************************************************************/

using namespace fcpw;

int main()
{
  // make test data
  const int nVertices = 4;
  const int nTriangles = 2;

  const Eigen::Vector2f positions[nVertices] = {
      {0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}, {0.0, 0.0}};

  const Eigen::Vector3i indices[nTriangles] = {{0, 1, 2}, {0, 2, 3}};

  // initialize a 2d scene
  Scene<2> scene;

  // set the PrimitiveType for each object in the scene;
  // in this case, we have a single object consisting of triangles
  scene.setObjectTypes({{PrimitiveType::Triangle}});

  // set the vertex and triangle count of the (0th) object
  scene.setObjectVertexCount(nVertices, 0);
  scene.setObjectTriangleCount(nTriangles, 0);

  // specify the vertex positions
  for (int i = 0; i < nVertices; i++)
  {
    scene.setObjectVertex(positions[i], i, 0);
  }

  // specify the triangle indices
  for (int i = 0; i < nTriangles; i++)
  {
    scene.setObjectTriangle(indices[i].data(), i, 0);
  }

  // compute vertex & edge normals (optional)
  scene.computeObjectNormals(0);

  // compute silhouette data (required only for closest silhouette point queries)
  scene.computeSilhouettes();

  // now that the geometry has been specified, build the acceleration structure
  scene.build(AggregateType::Bvh_SurfaceArea,
              true); // the second boolean argument enables vectorization

  const Eigen::Vector2f queryPoint = {0.5, 0.5};

  // perform a closest point query
  Interaction<2> cpqInteraction;
  scene.findClosestPoint(queryPoint, cpqInteraction);

  // perform a closest silhouette point query
  Interaction<2> cspqInteraction;
  scene.findClosestSilhouettePoint(queryPoint, cspqInteraction);

  const Eigen::Vector2f rayOrigin = {0.5, 0.5};
  const Eigen::Vector2f rayDir = {0, 0};

  Ray<2> queryRay(rayOrigin, rayDir);

  // perform a ray intersection query
  std::vector<Interaction<2>> rayInteractions;
  scene.intersect(queryRay, rayInteractions, false,
                  true); // don't check for occlusion, and record all hits

  return 0;
}
