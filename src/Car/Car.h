#ifndef HYBRID_ASTAR_PLANNER_CAR_H
#define HYBRID_ASTAR_PLANNER_CAR_H

#include <vector>
#include <tuple>
#include <utility>

using namespace std;
class Car {
public:
    Car(vector<double> dimensions,
        vector<double> pose_);
    vector<vector<double>> getOutline();
private:
    double length;
    double width;
    vector<double> pose; // x, y, yaw
};


#endif //HYBRID_ASTAR_PLANNER_CAR_H
