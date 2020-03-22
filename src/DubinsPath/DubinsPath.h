#ifndef HYBRID_ASTAR_PLANNER_DUBINSPATH_H
#define HYBRID_ASTAR_PLANNER_DUBINSPATH_H

#include <tuple>
#include <utility>
#include <vector>

using namespace std;

class DubinsPath {
public:
    DubinsPath(tuple<double, double, double> start_pose,
               tuple<double, double, double> end_pose,
               double radius_): start(std::move(start_pose)), end(std::move(end_pose)),
               radius(radius_) {}
    tuple<pair<char, double>, pair<char, double>, pair<char, double>>
        getShortestPath();
    vector<tuple<double, double, double>> generatePath();
private:
    tuple<double, double, double> start, end;
    double radius;
    double mod2Pi(double theta);
    tuple<double, double, double> calcEnd();
    tuple<double, double> calcTurnCenter(tuple<double, double, double> point,
            char dir);
    tuple<pair<char, double>, pair<char, double>, pair<char, double>>
        calcLSL(tuple<double, double, double> e);
    tuple<pair<char, double>, pair<char, double>, pair<char, double>>
        calcRSR(tuple<double, double, double> e);
    tuple<pair<char, double>, pair<char, double>, pair<char, double>>
        calcLSR(tuple<double, double, double> e);
    tuple<pair<char, double>, pair<char, double>, pair<char, double>>
        calcRSL(tuple<double, double, double> e);
    tuple<pair<char, double>, pair<char, double>, pair<char, double>>
        calcLRL(tuple<double, double, double> e);
    tuple<pair<char, double>, pair<char, double>, pair<char, double>>
        calcRLR(tuple<double, double, double> e);
    vector<tuple<pair<char, double>, pair<char, double>, pair<char, double>>>
        calcPaths();
};


#endif //HYBRID_ASTAR_PLANNER_DUBINSPATH_H
