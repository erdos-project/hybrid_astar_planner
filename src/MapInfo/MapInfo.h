#ifndef HYBRID_ASTAR_PLANNER_MAPINFO_H
#define HYBRID_ASTAR_PLANNER_MAPINFO_H

#include <vector>

#include "src/Car/Car.h"
#include "src/Obstacle/Obstacle.h"
#include "src/utils.h"

using namespace std;

class MapInfo {
public:
    vector<double> start;
    vector<double> end;
    double width;
    double length;
    MapInfo(vector<double> dimensions, vector<double> start_,
            vector<double> end_, vector<Obstacle *> obstacles_);
    MapInfo(vector<double> dimensions, vector<double> start_,
            vector<double> end_, vector<Obstacle *> obstacles_,
            vector<double> car_dimensions, Pose car_pose);
    void setCarPose(Pose p);
    vector<Point> getCarOutline();
    bool isCollision(vector<vector<double>> car_outline);
    bool isCollision(vector<double> point);

private:
    Car car;
    vector<Obstacle *> obstacles;
};


#endif //HYBRID_ASTAR_PLANNER_MAPINFO_H
