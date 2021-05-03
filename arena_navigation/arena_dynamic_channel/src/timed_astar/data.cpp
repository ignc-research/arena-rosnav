#include "arena_dynamic_channel/timed_astar/data.h"

namespace timed_astar
{   
    // Vec2i
    bool operator == (const Vec2i& a, const Vec2i& b){
        return (a.x == b.x && a.y == b.y);
    }

    // Vec2d
    Vec2d operator + (const Vec2d &a, const Vec2d &b) {
        return Vec2d(a.x + b.x, a.y + b.y);
    }
    Vec2d operator - (const Vec2d &a, const Vec2d &b) {
        return Vec2d(a.x - b.x, a.y - b.y);
    }
    Vec2d operator * (const Vec2d &a, const Vec2d &b) {
        return Vec2d(a.x * b.x, a.y * b.y);
    }
    Vec2d operator * (const Vec2d &a, const double scale) {
        return Vec2d(a.x * scale, a.y * scale);
    }
    Vec2d operator / (const Vec2d &a, const double scale) {
        return Vec2d(a.x / scale, a.y / scale);
    }

    double dot (const Vec2d &a, const Vec2d &b) {
        return a.x * b.x + a.y * b.y;
    }

    double cross (const Vec2d &a, const Vec2d &b) {
        return a.x * b.y - a.y * b.x;
    }

    bool operator == (const Vec2d &a, const Vec2d &b) {
        return (fabs(a.x - b.x) < DBL_EPSILON  && fabs(a.y - b.y) < DBL_EPSILON);
    }
    bool operator < (const Vec2d &a, const Vec2d &b) {
        return (a.x < b.x) && (a.y < b.y);
    }
    bool operator <= (const Vec2d &a, const Vec2d &b) {
        return (a.x <= b.x) && (a.y <= b.y);
    }
    bool operator > (const Vec2d &a, const Vec2d &b) {
        return (a.x > b.x) && (a.y > b.y);
    }
    bool operator >= (const Vec2d &a, const Vec2d &b) {
        return (a.x >= b.x) && (a.y >= b.y);
    }


}


