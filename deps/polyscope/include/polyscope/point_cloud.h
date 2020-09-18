// Copyright 2017-2019, Nicholas Sharp and the Polyscope contributors. http://polyscope.run.
#pragma once

#include "polyscope/affine_remapper.h"
#include "polyscope/color_management.h"
#include "polyscope/persistent_value.h"
#include "polyscope/point_cloud_quantity.h"
#include "polyscope/polyscope.h"
#include "polyscope/render/engine.h"
#include "polyscope/scaled_value.h"
#include "polyscope/standardize_data_array.h"
#include "polyscope/structure.h"

#include "polyscope/point_cloud_color_quantity.h"
#include "polyscope/point_cloud_scalar_quantity.h"
#include "polyscope/point_cloud_vector_quantity.h"

#include <vector>

namespace polyscope {

// Forward declare point cloud
class PointCloud;

// Forward declare quantity types
class PointCloudColorQuantity;
class PointCloudScalarQuantity;
class PointCloudVectorQuantity;


template <> // Specialize the quantity type
struct QuantityTypeHelper<PointCloud> {
  typedef PointCloudQuantity type;
};

class PointCloud : public QuantityStructure<PointCloud> {
public:
  // === Member functions ===

  // Construct a new point cloud structure
  PointCloud(std::string name, std::vector<glm::vec3> points);

  // === Overrides

  // Build the imgui display
  virtual void buildCustomUI() override;
  virtual void buildCustomOptionsUI() override;
  virtual void buildPickUI(size_t localPickID) override;

  // Standard structure overrides
  virtual void draw() override;
  virtual void drawPick() override;
  virtual double lengthScale() override;
  virtual std::tuple<glm::vec3, glm::vec3> boundingBox() override;
  virtual std::string typeName() override;

  // === Quantities

  // Scalars
  template <class T>
  PointCloudScalarQuantity* addScalarQuantity(std::string name, const T& values, DataType type = DataType::STANDARD);

  // Colors
  template <class T>
  PointCloudColorQuantity* addColorQuantity(std::string name, const T& values);

  // Vectors
  template <class T>
  PointCloudVectorQuantity* addVectorQuantity(std::string name, const T& vectors,
                                              VectorType vectorType = VectorType::STANDARD);
  template <class T>
  PointCloudVectorQuantity* addVectorQuantity2D(std::string name, const T& vectors,
                                                VectorType vectorType = VectorType::STANDARD);

  // === Mutate
  template <class V>
  void updatePointPositions(const V& newPositions);
  template <class V>
  void updatePointPositions2D(const V& newPositions);

  // The points that make up this point cloud
  std::vector<glm::vec3> points;
  size_t nPoints() const { return points.size(); }

  // Misc data
  static const std::string structureTypeName;

  // Small utilities
  void deleteProgram();
  void writePointsToFile(std::string filename = "");
  void setPointCloudUniforms(render::ShaderProgram& p);

  // === Get/set visualization parameters

  // set the base color of the points
  PointCloud* setPointColor(glm::vec3 newVal);
  glm::vec3 getPointColor();

  // set the radius of the points
  PointCloud* setPointRadius(double newVal, bool isRelative = true);
  double getPointRadius();

  // Material
  PointCloud* setMaterial(std::string name);
  std::string getMaterial();

private:
  // === Visualization parameters
  PersistentValue<glm::vec3> pointColor;
  PersistentValue<ScaledValue<float>> pointRadius;
  PersistentValue<std::string> material;

  // Drawing related things
  // if nullptr, prepare() (resp. preparePick()) needs to be called
  std::shared_ptr<render::ShaderProgram> program;
  std::shared_ptr<render::ShaderProgram> pickProgram;

  // === Helpers
  // Do setup work related to drawing, including allocating openGL data
  void prepare();
  void preparePick();
  void geometryChanged();


  // === Quantity adder implementations
  PointCloudScalarQuantity* addScalarQuantityImpl(std::string name, const std::vector<double>& data, DataType type);
  PointCloudColorQuantity* addColorQuantityImpl(std::string name, const std::vector<glm::vec3>& colors);
  PointCloudVectorQuantity* addVectorQuantityImpl(std::string name, const std::vector<glm::vec3>& vectors,
                                                  VectorType vectorType);
};


// Shorthand to add a point cloud to polyscope
template <class T>
PointCloud* registerPointCloud(std::string name, const T& points);
template <class T>
PointCloud* registerPointCloud2D(std::string name, const T& points);

// Shorthand to get a point cloud from polyscope
inline PointCloud* getPointCloud(std::string name = "");
inline bool hasPointCloud(std::string name = "");
inline void removePointCloud(std::string name = "", bool errorIfAbsent = true);


} // namespace polyscope

#include "polyscope/point_cloud.ipp"
