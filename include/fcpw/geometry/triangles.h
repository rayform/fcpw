#pragma once

#include <fcpw/geometry/polygon_soup.h>

namespace fcpw {

template <size_t DIM>
class Triangle: public GeometricPrimitive<DIM> {
public:
    // constructor
    Triangle();

    // returns bounding box
    BoundingBox<DIM> boundingBox() const;

    // returns centroid
    Vector<DIM> centroid() const;

    // returns surface area
    float surfaceArea() const;

    // returns signed volume
    float signedVolume() const;

    // returns normal
    Vector<DIM> normal(bool normalize=false) const;

    // returns the normalized normal based on the local parameterization
    Vector<DIM> normal(const Vector2& uv) const;

    // returns barycentric coordinates
    Vector2 barycentricCoordinates(const Vector<DIM>& p) const;

    // samples a random point on the geometric primitive and returns sampling pdf
    float samplePoint(float *randNums, Vector2& uv, Vector<DIM>& p, Vector<DIM>& n) const;

    // returns texture coordinates
    Vector2 textureCoordinates(const Vector2& uv) const;

    // returns the corner angle at the given vertex
    float angle(int vIndex) const;

    // splits the triangle along the provided coordinate and axis
    void split(int dim, float splitCoord, BoundingBox<3>& boxLeft,
               BoundingBox<3>& boxRight) const;

    // intersects with ray
    int intersect(Ray<3>& r, std::vector<Interaction<3>>& is,
                  bool checkForOcclusion=false, bool recordAllHits=false) const;

    // intersects with sphere
    int intersect(const BoundingSphere<3>& s, std::vector<Interaction<3>>& is,
                  bool recordOneHit=false) const;

    // finds closest point to sphere center
    bool findClosestPoint(BoundingSphere<DIM>& s, Interaction<DIM>& i, bool recordNormal=false) const;

    // get and set index
    int getIndex() const { return pIndex; }
    void setIndex(int index) { pIndex = index; }

    // members
    int indices[3];
    int pIndex;
    const PolygonSoup<DIM> *soup;

protected:
    // returns normalized vertex or edge normal if available;
    // otherwise computes normalized triangle normal
    Vector<DIM> normal(int vIndex, int eIndex) const;
};

} // namespace fcpw

#include "triangles.inl"
