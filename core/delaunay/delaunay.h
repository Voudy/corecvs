/*
 * \file delaunay.h
 * \brief delaunay triangulation
 *
 * \ingroup cppcorefiles
 * \date Dec 22, 2018
 * \author Spirin Egor
 */

#ifndef _DELAUNAY_H
#define _DELAUNAY_H

#include <vector>

#include "core/math/vector/vector2d.h"
#include "core/geometry/polygons.h"

namespace corecvs {

class DelaunayTriangulation {
 public:
    explicit DelaunayTriangulation(const std::vector<Vector2dd>& points);

    void GetPoints(std::vector<Vector2dd>* points);

    void GetTriangulation(std::vector<Triangle2dd>* triangles);

    void BuildTriangulation();

 private:
    const double EPSILON = 1e-5;
    const int OFFSET = 10;
    std::vector<Vector2dd> points_;
    std::vector<Triangle2dd> triangles_;

    void AddPointToTriangulation(const Vector2dd& point);

    Vector2dd GetCircumcircleCenter(const Triangle2dd& triangle);

    bool PointInsideCircumcircle(const Vector2dd& point, const Triangle2dd& triangle);

    double Length(const Vector2dd& point1, const Vector2dd& point2);

    bool AlmostEqualSegments(const std::pair<Vector2dd, Vector2dd>& a,
                             const std::pair<Vector2dd, Vector2dd>& b);

};

}  // namespace corecvs


#endif  // _DELAUNAY_H
