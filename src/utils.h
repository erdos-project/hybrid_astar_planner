#ifndef HYBRID_ASTAR_PLANNER_UTILS_H
#define HYBRID_ASTAR_PLANNER_UTILS_H

#include <cmath>
#include <vector>

using namespace std;

typedef vector<double> Point;
typedef vector<double> Pose;

inline Point calcTurnCenter(Point point, char dir, double radius) {
    double x, y, ang;
    Point turn_center;

    ang = point[2];
    if (dir == 'l') ang += M_PI_2;
    else if (dir == 'r') ang -= M_PI_2;

    x = point[0] + cos(ang) * radius;
    y = point[1] + sin(ang) * radius;
    turn_center.push_back(x);
    turn_center.push_back(y);
    return turn_center;
}
#endif //HYBRID_ASTAR_PLANNER_UTILS_H
