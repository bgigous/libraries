// Copyright 2016 Carrie Rebhuhn
#ifndef MATH_XY_H_
#define MATH_XY_H_
#include <utility>

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
        return lhs.x == rhs.x && lhs.y == rhs.y
            && lhs.first == rhs.first && lhs.second == rhs.second;
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
}  // namespace easymath
#endif  // MATH_XY_H_
