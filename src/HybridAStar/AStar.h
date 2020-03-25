#ifndef HYBRID_ASTAR_PLANNER_ASTAR_H
#define HYBRID_ASTAR_PLANNER_ASTAR_H

#include <vector>

#include "src/MapInfo/MapInfo.h"
#include "src/utils.h"

struct AStarPoint
{
    Point point;
    double g, h, f;
    Point camefrom;
};
class AStar {
public:
    AStar(MapInfo *map_info_);
    vector<Point> runAStar();
    static double distance(Point p1, Point p2);
private:
    vector<AStarPoint> openlist, closelist;
    MapInfo *map_info;
    vector<Point> getNeighbors(Point &p);
    vector<Point> reconstructPath(Point p);
};


#endif //HYBRID_ASTAR_PLANNER_ASTAR_H
