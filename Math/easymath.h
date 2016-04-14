// Copyright 2016 Carrie Rebhuhn
#ifndef MATH_EASYMATH_H_
#define MATH_EASYMATH_H_

#include <utility>
#include <set>

#include "MatrixTypes.h"
#include "XY.h"

namespace easymath {

//! Returns bin assignment based on bounds. Bounds must be sorted
int bin(const double& n, const matrix1d& bounds);

//! Calculates the manhattan distance between two points
double manhattan_distance(const XY &p1, const XY &p2);

//! Calculates the euclidean distance (l2 norm)
double euclidean_distance(const XY &p1, const XY &p2);

//! Calculates the cardinal direction of a vector
int cardinal_direction(const XY &dx_dy);

//! Cross product between vectors.
//! This assumes U and V are endpoints of vectors that originate at (0,0)
double cross(const XY &U, const XY &V);

//! Overloaded type definition to allow for additional calculations
typedef std::pair<XY, XY> line_segment;

//! Checks whether lines intersect in the center.
//! Coinciding endpoints excluded). Returns true if so.
bool intersects_in_center(line_segment edge1, line_segment edge2);

//! Returns a random number between some bounds.
double rand(double low, double high);

//! Error function (this exists in linux but not windows)
double erfc(double x);

std::set<XY> get_n_unique_points(double xmin, double xmax,
    double ymin, double ymax, size_t n);
}  // namespace easymath
#endif  // MATH_EASYMATH_H_
