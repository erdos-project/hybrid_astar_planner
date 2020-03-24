#ifndef HYBRID_ASTAR_PLANNER_MAPINFO_H
#define HYBRID_ASTAR_PLANNER_MAPINFO_H

#include <vector>

#include "src/Obstacle/Obstacle.h"

using namespace std;

class MapInfo {
public:
    MapInfo(vector<double> dimensions, vector<double> start_,
            vector<double> end_, vector<Obstacle *> obstacles_);
    bool isCollision(vector<vector<double>> car_outline);
    bool isCollision(vector<double> point);
    vector<double> start;
    vector<double> end;
    double width;
    double length;
private:
    vector<Obstacle *> obstacles;
};


#endif //HYBRID_ASTAR_PLANNER_MAPINFO_H
