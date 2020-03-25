#include "include/HybridAStar.h"

#include <algorithm>

HybridAStar::HybridAStar(MapInfo *map_info_, double radius_):
    map_info(map_info_), radius(radius_) {
    HybridAStarPoint p;
    double d = hCost(map_info->start);
    p.pose.assign(map_info->start.begin(), map_info->start.end());
    p.g = 0.0;
    p.h = d;
    p.f = d;
    openlist.push_back(p);
    push_heap(openlist.begin(), openlist.end());
}

// Get the reachable neighbors via Dubin's path
// Arguments:
//      p: pose x, y, yaw
// Returns:
//      a list of Dubin's paths
vector<DubinsPath> HybridAStar::getNeighbors(Pose &p) {
    vector<DubinsPath> paths;
    double rad = radius + RAD_UPPER_RANGE;
    // apply different turning angles / distances to produce neighbors
    while (rad >= radius - RAD_LOWER_RANGE) {
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

// Backwards traverse through the graph to reconstruct a path
// Arguments:
//      p: ending pose
// Returns:
//      reconstructed path vector containing pose x, y, yaw
vector<Pose> HybridAStar::reconstructPath(Pose &p) {
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

// Calculate the heuristic cost to the end goal
double HybridAStar::hCost(Pose &p) {
    double d = distance(p, map_info->end);
    return d;
}

// Determine whether the car will collide with an obstacle along a path
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

// Compute the distance between two points
double HybridAStar::distance(Point p1, Point p2) {
    return sqrt(pow(p2[0] - p1[0], 2) + pow(p2[1]- p1[1], 2));
}

// Run Hybrid A Star algorithm
vector<Pose> HybridAStar::runHybridAStar() {
    int count = 0;
    while (!openlist.empty() && count < map_info->width * map_info->length) {
        count += 1;

        // get the lowest total cost node and add it to close list
        pop_heap(openlist.begin(), openlist.end());
        HybridAStarPoint x = openlist.front();
        openlist.pop_back();
        closelist.push_back(x);

        // construct Dubin's path to end
        Dubins dbp(x.pose, map_info->end, radius);
        DubinsPath shortest_dp = dbp.getShortestPath();
        vector<Pose> path = dbp.generatePath(x.pose, shortest_dp);
        if (!isCollision(path) &&
                distance(x.pose, map_info->end) < COMPLETION_THRESHOLD) {
            return reconstructPath(x.pose);
        }

        // unable to connect to end, explore Dubin's neighbors
        vector<DubinsPath> neighbors = getNeighbors(x.pose);
        for (auto neighbor : neighbors) {
            vector<Pose> neighbor_path =
                Dubins::generatePath(x.pose, neighbor, radius);
            Pose y = neighbor_path.back();

            // continue if a nearby point was already explored or collision
            if (std::find_if(
                    closelist.begin(), closelist.end(),
                    [&](HybridAStarPoint &p) {
                        return (
                            distance(p.pose, y) < COMPLETION_THRESHOLD
                        );
                    }) != closelist.end() || isCollision(neighbor_path)
                    )
                continue;

            // update current heuristic cost
            double tentative_g_score = x.g;
            if (neighbor[0].first == 's')
                tentative_g_score += abs(neighbor[0].second);
            else tentative_g_score += abs(neighbor[0].second * radius);

            // keep the neighbor if it is unexplored
            auto it_y = std::find_if(
                    openlist.begin(), openlist.end(),
                    [&](HybridAStarPoint &p) {
                        return (p.pose == y);
                    }
            );
            if (it_y == openlist.end()) {
                double d = hCost(y);
                HybridAStarPoint y_;
                y_.pose.assign(y.begin(), y.end());
                y_.g = tentative_g_score;
                y_.h = d;
                y_.f = y_.g + y_.h;
                y_.path.assign(neighbor_path.begin(), neighbor_path.end());
                y_.camefrom.assign(x.pose.begin(), x.pose.end());
                openlist.push_back(y_);
                push_heap(openlist.begin(), openlist.end());
            }
        }
    }
    vector<Pose> ret;
    return ret;
}