#include <iostream>
#include <chrono>

#include "HybridAStar.h"

using namespace std::chrono;

HybridAStar::HybridAStar(MapInfo *map_info_, double radius_):
    map_info(map_info_), radius(radius_) {
    HybridAStarPoint p;
    double d = hCost(map_info->start);
    p.pose.assign(map_info->start.begin(), map_info->start.end());
    p.g = 0.0;
    p.h = d;
    p.f = d;
    openlist.push_back(p);
}

vector<DubinsPath> HybridAStar::getNeighbors(Pose &p) {
    vector<DubinsPath> paths;
    double rad = RAD_RANGE * radius;
    while (rad >= radius) {
        rad -= RAD_STEP;
        DubinsPath dpl (1, make_pair('l', STEP_SIZE / rad));
        DubinsPath dpr (1, make_pair('r', STEP_SIZE / rad));
        paths.push_back(dpl);
        paths.push_back(dpr);
    }
    DubinsPath straight(1, make_pair('s', STEP_SIZE));

    // can make the STEP_SIZE negative to enable reversing
    paths.push_back(straight);

    return paths;
}

vector<Pose> HybridAStar::reconstructPath(Pose p) {
    vector<Pose> path;
    while (p != map_info->start) {
        auto it = find_if(
                closelist.begin(), closelist.end(),
                [&](HybridAStarPoint &pt) {
                    return (p == pt.pose);
                }
        );
        for (auto wp_it = it->path.rbegin(); wp_it != it->path.rend(); wp_it++) {
            path.push_back(*wp_it);
        }
        p = it->camefrom;
    }
    path.push_back(map_info->start);
    reverse(path.begin(), path.end());
    return path;
}

double HybridAStar::hCost(Pose &p) {
    double d = AStar::distance(p, map_info->end);
    return d;
}

bool HybridAStar::isCollision(vector<Pose> path) {
    Pose p;
    for (size_t i = 0; i < path.size(); i += 1) {
        p = path[i];
        map_info->setCarPose(p);
        if (map_info->isCollision(map_info->getCarOutline())) {
            return true;
        }
    }
    return false;
}

vector<Pose> HybridAStar::runHybridAStar() {
    int count = 0;
    double closest = INFINITY;
    while (count < map_info->width * map_info->length) {
        cout << closest << endl;
        count += 1;
        auto it_min = min_element(
                openlist.begin(), openlist.end(),
                [](HybridAStarPoint p1, HybridAStarPoint p2) {
                    return (p1.f < p2.f);
                }
        );
        HybridAStarPoint x = *it_min;
        openlist.erase(it_min);
        closelist.push_back(x);
        Dubins dbp(x.pose, map_info->end, radius);
        DubinsPath shortest_dp = dbp.getShortestPath();
        vector<Pose> path = dbp.generatePath(x.pose, shortest_dp);
        closest = min(closest, AStar::distance(x.pose, map_info->end));
        if (!isCollision(path) &&
            AStar::distance(x.pose, map_info->end) < sqrt(2)) {
            return reconstructPath(x.pose);
        }

        vector<DubinsPath> neighbors = getNeighbors(x.pose);
        for (auto neighbor : neighbors) {
            vector<Pose> neighbor_path =
                Dubins::generatePath(x.pose, neighbor, radius);
            Pose y = neighbor_path.back();
            if (std::find_if(
                    closelist.begin(), closelist.end(),
                    [&](HybridAStarPoint &p) {
                        return (
                            AStar::distance(p.pose, y) < sqrt(2)
                        );
                    }) != closelist.end()
                    )
                continue;
            if (isCollision(neighbor_path)) continue;
            double tentative_g_score = x.g;
            if (neighbor[0].first == 's')
                tentative_g_score += abs(neighbor[0].second);
            else tentative_g_score += abs(neighbor[0].second * radius);
            bool tentative_is_better = true;
            auto it_y = std::find_if(
                    openlist.begin(), openlist.end(),
                    [&](HybridAStarPoint &p) {
                        return (p.pose == y);
                    }
            );
            if (it_y == openlist.end())
                tentative_is_better = true;
            else if (tentative_g_score < it_y->g)
                tentative_is_better = true;
            else
                tentative_is_better = false;
            if (tentative_is_better) {
                if (it_y == openlist.end())
                {
                    double d = hCost(y);
                    HybridAStarPoint y_;
                    y_.pose.assign(y.begin(), y.end());
                    y_.g = tentative_g_score;
                    y_.h = d;
                    y_.f = y_.g + y_.h;
                    y_.path.assign(neighbor_path.begin(), neighbor_path.end());
                    y_.camefrom.assign(x.pose.begin(), x.pose.end());
                    openlist.push_back(y_);
                }
                else
                {
                    it_y->g = tentative_g_score;
                    it_y->f = it_y->g + it_y->h;
                    it_y->path.assign(neighbor_path.begin(), neighbor_path
                    .end());
                    it_y->camefrom.assign(x.pose.begin(), x.pose.end());
                }
            }
        }
    }
    vector<Pose> ret;
    return ret;
}