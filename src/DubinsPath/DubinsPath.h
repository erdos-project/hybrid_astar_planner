#ifndef HYBRID_ASTAR_PLANNER_DUBINSPATH_H
#define HYBRID_ASTAR_PLANNER_DUBINSPATH_H

#include <vector>
#include <utility>
#include <vector>

using namespace std;

class DubinsPath {
public:
    DubinsPath(vector<double> start_pose, vector<double> end_pose,
               double radius_): start(move(start_pose)),
               end(move(end_pose)), radius(radius_) {}
    vector<pair<char, double>>
        getShortestPath();
    vector<vector<double>> generatePath(vector<double> s,
            vector<pair<char, double>> path);
private:
    vector<double> start, end;
    double radius;
    static double mod2Pi(double theta);
    vector<double> calcEnd();
    vector<double> calcTurnCenter(vector<double> point, char dir);
    vector<pair<char, double>> calcLSL(vector<double> e);
    vector<pair<char, double>> calcRSR(vector<double> e);
    vector<pair<char, double>> calcLSR(vector<double> e);
    vector<pair<char, double>> calcRSL(vector<double> e);
    vector<pair<char, double>> calcLRL(vector<double> e);
    vector<pair<char, double>> calcRLR(vector<double> e);
    vector<vector<pair<char, double>>> calcPaths();
};


#endif //HYBRID_ASTAR_PLANNER_DUBINSPATH_H
