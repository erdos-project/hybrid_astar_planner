#ifndef HYBRID_ASTAR_PLANNER_UTILS_H
#define HYBRID_ASTAR_PLANNER_UTILS_H

#include <cmath>
#include <vector>

using namespace std;

typedef vector<double> Point;
typedef vector<double> Pose;

enum direction_t {left='l', straight='s', right='r'};

inline Point calcTurnCenter(Point point, char dir, double radius) {
    double x, y, ang;
    Point turn_center;

    ang = point[2];
    if (dir == direction_t::left) ang += M_PI_2;
    else if (dir == direction_t::right) ang -= M_PI_2;

    x = point[0] + cos(ang) * radius;
    y = point[1] + sin(ang) * radius;
    turn_center.push_back(x);
    turn_center.push_back(y);
    return turn_center;
}

inline double distance(Point p1, Point p2) {
    return sqrt(pow(p2[0] - p1[0], 2) + pow(p2[1]- p1[1], 2));
}

#endif //HYBRID_ASTAR_PLANNER_UTILS_H
