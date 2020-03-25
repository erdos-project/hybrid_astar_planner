#ifndef HYBRID_ASTAR_PLANNER_OBSTACLE_H
#define HYBRID_ASTAR_PLANNER_OBSTACLE_H

#include <eigen3/Eigen/Dense>

#include "src/constants.h"

class Obstacle {
public:
    Obstacle(Eigen::Vector2f first_point, Eigen::Vector2f second_point);
    bool isSegmentInObstacle(Eigen::Vector2f &p1, Eigen::Vector2f &p2);
    bool isPointNearObstacle(Eigen::Vector2f &p, double radius);
    double getArea();
    std::pair<Eigen::Vector2f, Eigen::Vector2f> bbox;
private:
};


#endif //HYBRID_ASTAR_PLANNER_OBSTACLE_H