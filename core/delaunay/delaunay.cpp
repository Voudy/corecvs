/*
 * \file delaunay.h
 * \brief delaunay triangulation
 *
 * \ingroup cppcorefiles
 * \date Dec 22, 2018
 * \author Spirin Egor
 */

#include "core/delaunay/delaunay.h"

#include "core/math/mathUtils.h"
#include "delaunay.h"


corecvs::DelaunayTriangulation::DelaunayTriangulation(const vector<Vector2dd>& points) :
        points_(points) {
    BuildTriangulation();
}

void corecvs::DelaunayTriangulation::GetPoints(vector<Vector2dd>* points) {
    for (auto& point : points_) {
        points->emplace_back(point);
    }
}

void corecvs::DelaunayTriangulation::GetTriangulation(vector<Triangle2dd>* triangles) {
    for (auto& tri : triangles_) {
        triangles->emplace_back(tri);
    }
}

// Build delaunay triangulation using Bowyer-Watson algorithm
// https://en.wikipedia.org/wiki/Bowyer–Watson_algorithm
void corecvs::DelaunayTriangulation::BuildTriangulation() {
    triangles_.clear();
    // add super-triangle, which cover all points
    auto minMaxX = std::minmax_element(points_.begin(), points_.end(),
                                       [](const Vector2dd& a, const Vector2dd& b) {
                                           return a[0] < b[0];
                                       });
    auto minMaxY = std::minmax_element(points_.begin(), points_.end(),
                                       [](const Vector2dd& a, const Vector2dd& b) {
                                           return a[1] < b[1];
                                       });
    // adding offsets for nonchecking, that point lies on side of supertriangle
    auto minX = (*minMaxX.first)[0] - OFFSET, maxX = (*minMaxX.second)[0] + OFFSET,
            minY = (*minMaxY.first)[1] - OFFSET, maxY = (*minMaxY.second)[1] + OFFSET;
    // form vertexes of supertriangle
    auto p1 = Vector2dd(maxX + 1. * (maxY - minY) / tan(degToRad(60)), minY),
            p2 = Vector2dd(minX - 1. * (maxY - minY) / tan(degToRad(60)), minY),
            p3 = Vector2dd((minX + maxX) / 2, maxY + 1. * (minX + maxX) / 2 / tan(degToRad(30)));
    triangles_.emplace_back(p1, p2, p3);
    // iterate over all points and insert each into triangulation
    for (auto& point : points_) {
        AddPointToTriangulation(point);
    }
}

void corecvs::DelaunayTriangulation::AddPointToTriangulation(const corecvs::Vector2dd& point) {
    std::vector<Triangle2dd> cur_triangulation = std::move(triangles_);
    triangles_.clear();
    std::vector<std::pair<std::pair<Vector2dd, Vector2dd>, bool>> polygon;
    // filter triangles by containing point in circumcircle
    for (auto& tri : cur_triangulation) {
        if (PointInsideCircumcircle(point, tri)) {
            polygon.emplace_back(std::make_pair(tri.p1(), tri.p2()), true);
            polygon.emplace_back(std::make_pair(tri.p2(), tri.p3()), true);
            polygon.emplace_back(std::make_pair(tri.p3(), tri.p1()), true);
        } else {
            triangles_.emplace_back(tri);
        }
    }
    for (auto i = 0; i < polygon.size(); ++i) {
        for (auto j = i + 1; j < polygon.size(); ++j) {
            // check that this segments are equal
            if (AlmostEqualSegments(polygon[i].first, polygon[j].first)) {
                polygon[i].second = false;
                polygon[j].second = false;
            }
        }
    }
    // create new triangles
    for (auto& seg : polygon) {
        if (seg.second) {
            triangles_.emplace_back(seg.first.first, seg.first.second, point);
        }
    }
}

bool corecvs::DelaunayTriangulation::PointInsideCircumcircle(const corecvs::Vector2dd& point,
                                                             const corecvs::Triangle2dd& triangle) {
    // calculate radius of circumcircle
    // R = (a * b * c) / (4 * S)
    auto a = Length(triangle.p1(), triangle.p2()),
            b = Length(triangle.p2(), triangle.p3()),
            c = Length(triangle.p3(), triangle.p1());
    auto p = (a + b + c) / 2;
    auto S = sqrt(p * (p - a) * (p - b) * (p - c));
    auto R = a * b * c / (4 * S);
    return Length(point, GetCircumcircleCenter(triangle)) <= R;
}

double corecvs::DelaunayTriangulation::Length(const corecvs::Vector2dd& point1,
                                              const corecvs::Vector2dd& point2) {
    return (point1 - point2).getLengthStable();
}

bool corecvs::DelaunayTriangulation::AlmostEqualSegments(
        const std::pair<corecvs::Vector2dd, corecvs::Vector2dd>& a,
        const std::pair<corecvs::Vector2dd, corecvs::Vector2dd>& b) {
    if (a.first.notTooFar(b.first, EPSILON) && a.second.notTooFar(b.second, EPSILON)) {
        return true;
    } else if (a.second.notTooFar(b.first, EPSILON) && a.first.notTooFar(b.second, EPSILON)) {
        return true;
    } else {
        return false;
    }
}

corecvs::Vector2dd corecvs::DelaunayTriangulation::GetCircumcircleCenter(
        const corecvs::Triangle2dd& triangle) {
    // solution of  system: (x - a)^2 + (y - b)^2 = r^2
    auto x1 = triangle.p1().x(), y1 = triangle.p1().y();
    auto x2 = triangle.p2().x(), y2 = triangle.p2().y();
    auto x3 = triangle.p3().x(), y3 = triangle.p3().y();
    Matrix22 A(
            2 * (x2 - x1), 2 * (y2 - y1),
            2 * (x3 - x1), 2 * (y3 - y1)
            );
    Vector2dd B(
            y2 * y2 + x2 * x2 - y1 * y1 - x1 * x1,
            y3 * y3 + x3 * x3 - y1 * y1 - x1 * x1
            );
    return Matrix22::solve(A, B);
}
