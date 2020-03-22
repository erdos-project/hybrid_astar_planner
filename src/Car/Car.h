#ifndef HYBRID_ASTAR_PLANNER_CAR_H
#define HYBRID_ASTAR_PLANNER_CAR_H

#include <vector>
#include <tuple>
#include <utility>

class Car {
public:
    Car(std::pair<double, double> dimensions,
            std::tuple<double, double, double> pose_);
    std::vector<std::pair<double, double>> getOutline();
private:
    double length;
    double width;
    std::tuple<double, double, double> pose; // x, y, yaw
};


#endif //HYBRID_ASTAR_PLANNER_CAR_H
