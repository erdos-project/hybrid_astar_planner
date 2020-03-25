#ifndef HYBRID_ASTAR_PLANNER_DUBINS_H
#define HYBRID_ASTAR_PLANNER_DUBINS_H

#include <vector>
#include <utility>
#include <vector>

#include "src/utils.h"

using namespace std;

typedef pair<char, double> DubinsPoint;
typedef vector<DubinsPoint> DubinsPath;

class Dubins {
public:
    Dubins(vector<double> start_pose, vector<double> end_pose,
           double radius_): start(move(start_pose)),
               end(move(end_pose)), radius(radius_) {}
    vector<DubinsPoint> getShortestPath();
    vector<Pose> generatePath(vector<double> s,
            vector<DubinsPoint> path);
    static vector<Pose> generatePath(vector<double> s,
        vector<DubinsPoint> path, double radius);
private:
    vector<double> start, end;
    double radius;
    static double mod2Pi(double theta);
    Pose calcEnd();
    vector<DubinsPoint> calcLSL(vector<double> e);
    vector<DubinsPoint> calcRSR(vector<double> e);
    vector<DubinsPoint> calcLSR(vector<double> e);
    vector<DubinsPoint> calcRSL(vector<double> e);
    vector<DubinsPoint> calcLRL(vector<double> e);
    vector<DubinsPoint> calcRLR(vector<double> e);
    vector<DubinsPath> calcPaths();
};


#endif //HYBRID_ASTAR_PLANNER_DUBINS_H
