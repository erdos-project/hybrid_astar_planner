#ifndef HYBRID_ASTAR_PLANNER_ASTAR_H
#define HYBRID_ASTAR_PLANNER_ASTAR_H

#include <vector>

#include "src/MapInfo/MapInfo.h"

typedef vector<double> Point;
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
private:
    vector<AStarPoint> openlist, closelist;
    MapInfo *map_info;
    static double distance(Point &p1, Point &p2);
    vector<Point> getNeighbors(Point &p);
    vector<Point> reconstructPath(Point p);
};


#endif //HYBRID_ASTAR_PLANNER_ASTAR_H
