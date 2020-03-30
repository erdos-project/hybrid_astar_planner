#ifndef HYBRID_ASTAR_PLANNER_DUBINS_H
#define HYBRID_ASTAR_PLANNER_DUBINS_H

#include "utils.h"

#include <vector>
#include <utility>

using namespace std;

typedef pair<char, double> DubinsPoint;
typedef vector<DubinsPoint> DubinsPath;

class Dubins {
public:
    Dubins(Pose start_pose, Pose end_pose,
           double radius_): start(move(start_pose)),
               end(move(end_pose)), radius(radius_) {}
    vector<DubinsPoint> getShortestPath();
    static vector<Pose> generatePath(Pose s, vector<DubinsPoint> path,
            double radius);
private:
    Pose start, end;
    double radius;
    static double mod2Pi(double theta);
    Pose calcEnd();
    vector<DubinsPoint> calcLSL(Pose e);
    vector<DubinsPoint> calcRSR(Pose e);
    vector<DubinsPoint> calcLSR(Pose e);
    vector<DubinsPoint> calcRSL(Pose e);
    vector<DubinsPoint> calcLRL(Pose e);
    vector<DubinsPoint> calcRLR(Pose e);
    vector<DubinsPath> calcPaths();
};


#endif //HYBRID_ASTAR_PLANNER_DUBINS_H
