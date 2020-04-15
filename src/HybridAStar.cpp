#include "include/HybridAStar.h"

#include <algorithm>
#include <cmath>

HybridAStar::HybridAStar(MapInfo *map_info_,
    HybridAStarHyperparameters *hastar_hp_):
    map_info(map_info_) {
    hastar_hp = hastar_hp_;
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
    double rad = p[2] + hastar_hp->rad_upper_range;
    // apply different turning angles / distances to produce neighbors
    while (rad >= p[2] - hastar_hp->rad_lower_range) {
        // dont divide by 0
        if (abs(rad) < 0.01) {
            rad -= hastar_hp->rad_step;
            continue;
        }
        DubinsPath dpl (1, make_pair(direction_t::left,
            abs(hastar_hp->step_size / rad)));
        DubinsPath dpr (1, make_pair(direction_t::right,
            abs(hastar_hp->step_size / rad)));
        paths.push_back(dpl);
        paths.push_back(dpr);
        rad -= hastar_hp->rad_step;
    }
    DubinsPath straight(1, make_pair(direction_t::straight, hastar_hp->step_size));

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

// Run Hybrid A Star algorithm
vector<Pose> HybridAStar::runHybridAStar() {
    double closest_distance = INFINITY;
    Pose best_pose = map_info->start;
    int count = 0;
    while (!openlist.empty() && count < hastar_hp->max_iterations) {
        count += 1;
        // get the lowest total cost node and add it to close list
        pop_heap(openlist.begin(), openlist.end());
        HybridAStarPoint x = openlist.back();
        openlist.pop_back();
        closelist.push_back(x);
        // construct Dubin's path to end
        Dubins dbp(x.pose, map_info->end, hastar_hp->radius);
        DubinsPath shortest_dp = dbp.getShortestPath();
        vector<Pose> path = Dubins::generatePath(x.pose, shortest_dp, hastar_hp->radius);
        if (!isCollision(path) &&
            distance(x.pose, map_info->end) < closest_distance) {
            best_pose = x.pose;
            closest_distance = distance(x.pose, map_info->end);
        }

        if (distance(x.pose, map_info->end) <= hastar_hp->completion_threshold &&
            abs(x.pose[2] - map_info->end[2]) <= hastar_hp->angle_completion_threshold) {
            best_pose = x.pose;
            break;
        }
        // unable to connect to end, explore Dubin's neighbors
        vector<DubinsPath> neighbors = getNeighbors(x.pose);
        for (auto neighbor : neighbors) {
            vector<Pose> neighbor_path =
                Dubins::generatePath(x.pose, neighbor, hastar_hp->radius);
            Pose y = neighbor_path.back();

            // continue if a nearby point was already explored or collision
            if (std::find_if(
                    closelist.begin(), closelist.end(),
                    [&](HybridAStarPoint &p) {
                        return (
                            distance(p.pose, y) < hastar_hp->completion_threshold
                        );
                    }) != closelist.end()
                ) {
                continue;
            }

            if (isCollision(neighbor_path)) {
                continue;
            }

            // update current heuristic cost
            double tentative_g_score = x.g;
            if (neighbor[0].first == direction_t::straight)
                tentative_g_score += abs(neighbor[0].second);
            else tentative_g_score += abs(neighbor[0].second * hastar_hp->radius);

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

    return reconstructPath(best_pose);
}