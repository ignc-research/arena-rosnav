#pragma once

#include <algorithm>
#include <cmath>
#include <exception>
#include <iostream>
#include <limits>
#include <memory>
#include <utility>
#include <vector>


namespace delaunator {

const std::vector<double> DEFAULT_SPEED(1, 1.0);
const std::vector<double> DEFAULT_ANGLE(1, 0.0);
constexpr double EPSILON = std::numeric_limits<double>::epsilon();
constexpr std::size_t INVALID_INDEX = std::numeric_limits<std::size_t>::max();


//@see https://stackoverflow.com/questions/33333363/built-in-mod-vs-custom-mod-function-improve-the-performance-of-modulus-op/33333636#33333636
inline size_t fast_mod(const size_t i, const size_t c) {
    return i >= c ? i % c : i;
}

// Kahan and Babuska summation, Neumaier variant; accumulates less FP error
inline double sum(const std::vector<double>& x) {
    double sum = x[0];
    double err = 0.0;

    for (size_t i = 1; i < x.size(); i++) {
        const double k = x[i];
        const double m = sum + k;
        err += std::fabs(sum) >= std::fabs(k) ? sum - m + k : k - m + sum;
        sum = m;
    }
    return sum + err;
}

inline double dist(
    const double ax,
    const double ay,
    const double bx,
    const double by) {
    const double dx = ax - bx;
    const double dy = ay - by;
    return dx * dx + dy * dy;
}

inline double circumradius(
    const double ax,
    const double ay,
    const double bx,
    const double by,
    const double cx,
    const double cy) {
    const double dx = bx - ax;
    const double dy = by - ay;
    const double ex = cx - ax;
    const double ey = cy - ay;

    const double bl = dx * dx + dy * dy;
    const double cl = ex * ex + ey * ey;
    const double d = dx * ey - dy * ex;

    const double x = (ey * bl - dy * cl) * 0.5 / d;
    const double y = (dx * cl - ex * bl) * 0.5 / d;

    if ((bl > 0.0 || bl < 0.0) && (cl > 0.0 || cl < 0.0) && (d > 0.0 || d < 0.0)) {
        return x * x + y * y;
    } else {
        return std::numeric_limits<double>::max();
    }
}

inline bool orient(
    const double px,
    const double py,
    const double qx,
    const double qy,
    const double rx,
    const double ry) {
    return (qy - py) * (rx - qx) - (qx - px) * (ry - qy) < 0.0;
}

inline std::pair<double, double> circumcenter(
    const double ax,
    const double ay,
    const double bx,
    const double by,
    const double cx,
    const double cy) {
    const double dx = bx - ax;
    const double dy = by - ay;
    const double ex = cx - ax;
    const double ey = cy - ay;

    const double bl = dx * dx + dy * dy;
    const double cl = ex * ex + ey * ey;
    const double d = dx * ey - dy * ex;

    const double x = ax + (ey * bl - dy * cl) * 0.5 / d;
    const double y = ay + (dx * cl - ex * bl) * 0.5 / d;

    return std::make_pair(x, y);
}

struct compare {

    std::vector<double> const& coords;
    double cx;
    double cy;

    bool operator()(std::size_t i, std::size_t j) {
        const double d1 = dist(coords[2 * i], coords[2 * i + 1], cx, cy);
        const double d2 = dist(coords[2 * j], coords[2 * j + 1], cx, cy);
        const double diff1 = d1 - d2;
        const double diff2 = coords[2 * i] - coords[2 * j];
        const double diff3 = coords[2 * i + 1] - coords[2 * j + 1];

        if (diff1 > 0.0 || diff1 < 0.0) {
            return diff1 < 0;
        } else if (diff2 > 0.0 || diff2 < 0.0) {
            return diff2 < 0;
        } else {
            return diff3 < 0;
        }
    }
};

inline bool in_circle(
    const double ax,
    const double ay,
    const double bx,
    const double by,
    const double cx,
    const double cy,
    const double px,
    const double py) {
    const double dx = ax - px;
    const double dy = ay - py;
    const double ex = bx - px;
    const double ey = by - py;
    const double fx = cx - px;
    const double fy = cy - py;

    const double ap = dx * dx + dy * dy;
    const double bp = ex * ex + ey * ey;
    const double cp = fx * fx + fy * fy;

    return (dx * (ey * cp - bp * fy) -
            dy * (ex * cp - bp * fx) +
            ap * (ex * fy - ey * fx)) < 0.0;
}


inline bool check_pts_equal(double x1, double y1, double x2, double y2) {
    return std::fabs(x1 - x2) <= EPSILON &&
           std::fabs(y1 - y2) <= EPSILON;
}

// monotonically increases with real angle, but doesn't need expensive trigonometry
inline double pseudo_angle(const double dx, const double dy) {
    const double p = dx / (std::abs(dx) + std::abs(dy));
    return (dy > 0.0 ? 3.0 - p : 1.0 + p) / 4.0; // [0..1)
}

struct DelaunatorPoint {
    std::size_t i;
    double x;
    double y;
    std::size_t t;
    std::size_t prev;
    std::size_t next;
    bool removed;
};


class Delaunator {

public:
    // properties of moving objects
    std::vector<double> const speeds;
    std::vector<double> const angles;
    std::vector<double> const coords;

    // main data structures of graph
    std::vector<std::size_t> triangles;
    std::vector<std::size_t> halfedges;

    // convex hull of the graph
    std::vector<std::size_t> hull_prev;
    std::vector<std::size_t> hull_next;
    std::vector<std::size_t> hull_tri;
    std::size_t hull_start;

    Delaunator(std::vector<double> const& in_coords,
               std::vector<double> const& in_speeds,
               std::vector<double> const& in_angles);
    double get_hull_area();

private:
    std::vector<std::size_t> m_hash;
    double m_center_x;
    double m_center_y;
    std::size_t m_hash_size;
    std::vector<std::size_t> m_edge_stack;

    std::size_t legalize(std::size_t a);
    std::size_t hash_key(double x, double y) const;
    std::size_t add_triangle(
        std::size_t i0,
        std::size_t i1,
        std::size_t i2,
        std::size_t a,
        std::size_t b,
        std::size_t c);
    void link(std::size_t a, std::size_t b);
};


} //namespace delaunator
