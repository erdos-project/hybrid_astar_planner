#ifndef HYBRID_ASTAR_PLANNER_HYBRIDASTAR_H
#define HYBRID_ASTAR_PLANNER_HYBRIDASTAR_H

#include "Dubins.h"
#include "py_cpp_struct.h"
#include "MapInfo.h"
#include "utils.h"

#include <queue>

struct HybridAStarPoint
{
    Pose pose;
    double g, h, f;
    Pose camefrom;
    vector<Pose> path;
    inline bool operator < (const HybridAStarPoint& p) {
        return f > p.f;
    }
};

class HybridAStar {
public:
    HybridAStarHyperparameters *hastar_hp;
    HybridAStar(MapInfo *map_info_, HybridAStarHyperparameters *hastar_hp_);
    vector<Pose> runHybridAStar();
private:
    vector<HybridAStarPoint> openlist, closelist;
    MapInfo *map_info;
    double hCost(Pose &p);
    bool isCollision(vector<Pose> path);
    vector<DubinsPath> getNeighbors(Pose &p);
    vector<Pose> reconstructPath(Pose &p);
};


#endif //HYBRID_ASTAR_PLANNER_HYBRIDASTAR_H
