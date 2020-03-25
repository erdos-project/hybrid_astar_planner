#ifndef HYBRID_ASTAR_PLANNER_HYBRIDASTAR_H
#define HYBRID_ASTAR_PLANNER_HYBRIDASTAR_H

#include "src/HybridAStar/AStar.h"
#include "src/DubinsPath/Dubins.h"

struct HybridAStarPoint
{
    Pose pose;
    double g, h, f;
    Pose camefrom;
    vector<Pose> path;
};

class HybridAStar {
public:
    HybridAStar(MapInfo *map_info_, double radius_);
    vector<Pose> runHybridAStar();
private:
    vector<HybridAStarPoint> openlist, closelist;
    MapInfo *map_info;
    double radius;
    double hCost(Pose &p);
    bool isCollision(vector<Pose> path);
    vector<DubinsPath> getNeighbors(Pose &p);
    vector<Pose> reconstructPath(Pose p);
};


#endif //HYBRID_ASTAR_PLANNER_HYBRIDASTAR_H
