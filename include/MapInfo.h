#ifndef HYBRID_ASTAR_PLANNER_MAPINFO_H
#define HYBRID_ASTAR_PLANNER_MAPINFO_H

#include "include/Car.h"
#include "Obstacle.h"
#include "utils.h"

#include <vector>

using namespace std;

class MapInfo {
public:
    vector<double> start;
    vector<double> end;
    double width;
    double length;
    MapInfo(vector<double> dimensions, Pose start_,
            Pose end_, vector<Obstacle *> obstacles_);
    void setCarPose(Pose p);
    vector<Point> getCarOutline();
    bool isCollision(vector<Point> car_outline);
    bool isCollision(Point point);
private:
    Car car;
    vector<Obstacle *> obstacles;
};


#endif //HYBRID_ASTAR_PLANNER_MAPINFO_H
