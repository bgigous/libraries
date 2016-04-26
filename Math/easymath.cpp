// Copyright Carrie Rebhuhn 2016
#include "easymath.h"
#include <set>
#include <vector>
#include <utility>
#include <algorithm>

namespace easymath {
std::set<XY> get_n_unique_points(double x_min, double x_max,
    double y_min, double y_max, size_t n) {
    std::set<XY> pt_set;
    while (pt_set.size() < n) {
        XY p = XY(rand(x_min, x_max), rand(y_min, y_max));
        pt_set.insert(p);
    }
    return pt_set;
}

int get_nearest_square(int n) {
    return static_cast<int>(pow(ceil(sqrt(n)), 2));
}

std::pair<int, int> ind2sub(const int sub, const int cols, const int rows) {
    int row = sub / cols;
    int col = sub%rows;
    return std::make_pair(row, col);
}

std::vector<std::pair<int, int> > get_n_unique_square_subscripts(size_t n) {
    int square = get_nearest_square(n);
    std::vector<int> inds(square);
    for (size_t i = 0; i < inds.size(); i++) {
        inds[i] = i;
    }

    int n_surplus = square - n;
    for (int i = 0; i < n_surplus; i++) {
        inds.erase(inds.begin() + std::rand() % inds.size());
    }

    int base = sqrt(square);
    std::vector<std::pair<int, int> > subs(inds.size());
    for (size_t i = 0; i < subs.size(); i++) {
        subs[i] = ind2sub(inds[i], base, base);
    }
    return subs;
}

std::set<XY> get_n_unique_square_points(double x_min, double x_max,
    double y_min, double y_max, size_t n) {

    std::vector<std::pair<int, int> > subs = get_n_unique_square_subscripts(n);

    std::set<XY> pts;
    for (size_t i = 0; i < subs.size(); i++) {
        double base = sqrt(get_nearest_square(n));
        double xval = (x_max - x_min)
            *static_cast<double>(subs[i].first) / base;
        double yval = (y_max - y_min)
            *static_cast<double>(subs[i].second) / base;
        pts.insert(XY(xval, yval));
    }
    return pts;
}

double manhattan_distance(const XY &p1, const XY &p2) {
    XY diff = p1 - p2;
    return abs(diff.x) + abs(diff.y);
}


int cardinal_direction(const XY &dx_dy) {
    if (dx_dy.y >= 0) {  // Going up
        if (dx_dy.x >= 0) return 0;  // up-right
        else
            return 1;  // up-left
    } else {
        if (dx_dy.x >= 0) return 2;  // down-right
        else
            return 3;  // down-left
    }
}

double euclidean_distance(const XY &p1, const XY &p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return sqrt(dx*dx + dy*dy);
}

int bin(const double &n, const matrix1d& bounds) {
    for (size_t i = 0; i < bounds.size() - 1; i++)
        if (n < bounds[i + 1])
            return i;
    return bounds.size();
}

double rand(double low, double high) {
    double r = static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX);
    return r*(high - low) + low;
}

double cross(const XY &U, const XY &V) {
    return U.x*V.y - U.y*V.x;
}

bool intersects_in_center(line_segment edge1, line_segment edge2) {
    // Detects whether line intersects, but not at origin
    XY p = edge1.first;
    XY q = edge2.first;
    XY r = edge1.second - edge1.first;
    XY s = edge2.second - edge2.first;
    XY qpdiff = q - p;
    double rscross = cross(r, s);
    double t = cross((qpdiff), s) / rscross;
    double u = cross((qpdiff), r) / rscross;

    if (rscross == 0) {
        if (cross(qpdiff, r)) {
            // need to check bounds
            if (!(0 < t && t < 1 && 0 < u && u < 1)) {
                return false;
            }
            return true;  // collinear
        } else {
            return false;  // parallel non-intersecting
        }
    } else if (0 < t && t < 1 && 0 < u && u < 1) {
        // if you care about origins, use <= rather than <
        return true;  // intersects at p+tr = q+us
    } else {
        return false;  // not parallel, don't inersect
    }
}




double erfc(double x) {
    // constants
    double a1 = 0.254829592;
    double a2 = -0.284496736;
    double a3 = 1.421413741;
    double a4 = -1.453152027;
    double a5 = 1.061405429;
    double p = 0.3275911;

    // Save the sign of x
    int sign = 1;
    if (x < 0)
        sign = -1;
    x = fabs(x);

    // A&S formula 7.1.26
    double t = 1.0 / (1.0 + p*x);
    double y = 1.0 - (((((a5*t + a4)*t) + a3)*t + a2)*t + a1)*t*exp(-x*x);

    return 1 - sign*y;
}
}  // namespace easymath
