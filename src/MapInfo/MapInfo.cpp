#include "MapInfo.h"

#include <utility>
#include <eigen3/Eigen/Dense>

using namespace Eigen;

MapInfo::MapInfo(vector<double> dimensions, vector<double> start_,
                 vector<double> end_, vector<Obstacle *> obstacles_):
                 start(move(start_)), end(move(end_)),
                 obstacles(move(obstacles_)) {
    width = dimensions[0];
    length = dimensions[1];
}

bool MapInfo::isCollision(vector<vector<double>> car_outline) {
    Vector2f p1, p2;
    for (int i = 0; i < car_outline.size(); i++) {
        p1.x() = car_outline[i][0];
        p1.y() = car_outline[i][1];
        p2.x() = car_outline[(i+1) % car_outline.size()][0];
        p2.y() = car_outline[(i+1) % car_outline.size()][1];
        for (auto obstacle: obstacles) {
            if (obstacle->isSegmentInObstacle(p1, p2)) return true;
        }
    }
    return false;
}