#include <iostream>

#include "AStar.h"

AStar::AStar(MapInfo *map_info_):
    map_info(map_info_) {
    AStarPoint p;
    p.point.assign(map_info->start.begin(), map_info->start.end());
    p.g = 0.0;
    p.h = distance(map_info->start, map_info->end);
    p.f = p.g + p.h;
    openlist.push_back(p);
}

double AStar::distance(Point p1, Point p2) {
    return sqrt(pow(p2[0] - p1[0], 2) + pow(p2[1]- p1[1], 2));
}

vector<Point> AStar::getNeighbors(Point &p) {
    double neighbor[8][2] = {{-1, -1}, {0, -1}, {1, -1}, {1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}};
    vector<Point> points;
    for (auto & i : neighbor)
    {
        Point p_ = {p[0] + i[0], p[1] + i[1]};
        if (!map_info->isCollision(p_))
        {
            points.push_back(move(p_));
        }
    }
    return points;
}

vector<Point> AStar::reconstructPath(Point p) {
    vector<Point> path;
    while (p != map_info->start) {
        path.push_back(p);
        auto it = find_if(
            closelist.begin(), closelist.end(),
            [&](AStarPoint &pt) {
                return (p == pt.point);
            }
        );
        p = it->camefrom;
    }
    path.push_back(map_info->start);
    reverse(path.begin(), path.end());
    return path;
}

vector<Point> AStar::runAStar() {
    int count = 0;
    while (count < map_info->width * map_info->length) {
        count += 1;
        auto it_min = min_element(
            openlist.begin(), openlist.end(),
            [](AStarPoint &p1, AStarPoint &p2) {
                return (p1.f < p2.f);
            }
        );
        AStarPoint x = *it_min;
        openlist.erase(it_min);
        closelist.push_back(x);
        // sqrt(2) / 2 ~ 0.71 is the furthest distance for step size 1
        if (distance(x.point, map_info->end) < 0.71) {
            return reconstructPath(x.point);
        }
        vector<Point> neighbors = getNeighbors(x.point);
        for (auto y : neighbors) {
            if (std::find_if(
                    closelist.begin(), closelist.end(),
                    [&](AStarPoint &p) {
                        return (p.point == y);
                    }) != closelist.end()
                )
                continue;
            double tentative_g = x.g + distance(y, x.point);
            bool tentative_is_better = true;
            auto it_y = std::find_if(
                openlist.begin(), openlist.end(),
                [&](AStarPoint &p) {
                    return (p.point == y);
                }
            );
            if (it_y == openlist.end())
                tentative_is_better = true;
            else if (tentative_g < it_y->g)
                tentative_is_better = true;
            else
                tentative_is_better = false;
            if (tentative_is_better)
            {
                if (it_y == openlist.end())
                {
                    AStarPoint y_;
                    y_.point.assign(y.begin(), y.end());
                    y_.g = tentative_g;
                    y_.h = distance(y, map_info->end);
                    y_.f = y_.g + y_.h;
                    y_.camefrom.assign(x.point.begin(), x.point.end());
                    openlist.push_back(y_);
                }
                else
                {
                    it_y->g = tentative_g;
                    it_y->f = it_y->g + it_y->h;
                    it_y->camefrom.assign(x.point.begin(), x.point.end());
                }
            }
        }
    }
    vector<Point> ret;
    return ret;
}
