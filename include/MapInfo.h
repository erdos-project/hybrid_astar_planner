#ifndef HYBRID_ASTAR_PLANNER_MAPINFO_H
#define HYBRID_ASTAR_PLANNER_MAPINFO_H

#include "Car.h"
#include "py_cpp_struct.h"
#include "Obstacle.h"
#include "utils.h"

#include <eigen3/Eigen/Dense>
#include <vector>

using namespace std;
using namespace Eigen;

class MapInfo {
public:
    Pose start, end;
    HybridAStarInitialConditions *hastar_ic;
    HybridAStarHyperparameters *hastar_hp;
    MapInfo(HybridAStarInitialConditions *hastar_ic_,
        HybridAStarHyperparameters *hastar_hp_);
    ~MapInfo();
    void setCarPose(Pose p);
    void setObstacles();
    void addObstacle(Vector2f first_point, Vector2f second_point);
    vector<Point> getCarOutline();
    bool isCollision(vector<Point> car_outline);
    double getMapArea();
private:
    Car car;
    vector<Obstacle *> obstacles;
    vector<double> origin, bounds;
    void setStateSpace();
    bool isOutOfBounds(Vector2f p);
};


#endif //HYBRID_ASTAR_PLANNER_MAPINFO_H
