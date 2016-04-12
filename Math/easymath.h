// Copyright 2016 Carrie Rebhuhn
#ifndef MATH_EASYMATH_H_
#define MATH_EASYMATH_H_


#include "MatrixTypes.h"
#include <queue>
#include <utility>
#include <set>
#include <algorithm>

namespace easymath {
//! A class for locations. Contains many overloads for vector arithmetic.
class XY : public std::pair<double, double> {
 public:
    XY(const double &x, const double &y) :
        pair<double, double>(x, y), x(x), y(y) {}

    XY() {};
    ~XY() {};

    double x, y;

    //! Vector subtraction
    XY friend operator-(const XY &lhs, const XY &rhs) {
        return XY(lhs.x - rhs.x, lhs.y - rhs.y);
    }

    //! Orders first by x values, then by y values
    friend bool operator<(const XY &lhs, const XY &rhs) {
        if (lhs.x != rhs.x) return lhs.x < rhs.x;
        return lhs.y < rhs.y;
    }

    //! Checks equality of both elements
    friend bool operator==(const XY &lhs, const XY &rhs) {
        return lhs.x == rhs.x && lhs.y == rhs.y;
    }

    //! Scalar multiplication
    friend XY operator*(const XY &lhs, double rhs) {
        return XY(lhs.x*rhs, lhs.y*rhs);
    }

    //! Dot multiplication
    friend double operator*(const XY &U, const XY &V) {
        return U.x*V.x + U.y*V.y;
    }

    //! Vector addition
    friend XY operator+(const XY &lhs, const XY &rhs) {
        return XY(lhs.x + rhs.x, lhs.y + rhs.y);
    }

    //! Assignment operator
    XY& operator=(const XY &rhs) {
        x = rhs.x;
        y = rhs.y;
        first = rhs.first;
        second = rhs.second;
        return *this;
    }
};

//! Returns bin assignment based on bounds. Bounds must be sorted
int bin(const double& n, const matrix1d& bounds);

//! Clears using the swap idiom
template <class Container>
void clear(Container *q) {
    Container empty;
    std::swap(*q, empty);
}

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

//! Remove-erase-if idiom
template <class Container, class UnaryPredicate>
void remove_erase_if(Container stl, UnaryPredicate pred) {
    stl.erase(std::remove_if(stl.begin(), stl.end(), pred), stl.end());
    /*auto it = stl.begin();
    while (it!=stl.end()){
        if (!pred(*it)){
            stl.erase(it++);
        } else {
            it++;
        }
    }*/
}

//! Returns a random number between some bounds.
double rand(double low, double high);

//! Error function (this exists in linux but not windows)
double erfc(double x);

std::set<XY> get_n_unique_points(double xmin, double xmax,
    double ymin, double ymax, size_t n);
}  // namespace easymath
#endif  // MATH_EASYMATH_H_
