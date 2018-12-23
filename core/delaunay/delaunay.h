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
#include "core/math/vector/vector3d.h"

namespace corecvs {

int Triangulate(const std::vector<Vector2dd>& points, std::vector<Vector3du16>* triangles);

}  // namespace corecvs


#endif  // _DELAUNAY_H
