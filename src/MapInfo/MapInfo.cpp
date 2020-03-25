#include "MapInfo.h"

#include <utility>
#include <eigen3/Eigen/Dense>
#include <iostream>

#include "src/constants.h"

using namespace Eigen;

MapInfo::MapInfo(vector<double> dimensions, vector<double> start_,
                 vector<double> end_, vector<Obstacle *> obstacles_):
                 start(move(start_)), end(move(end_)),
                 obstacles(move(obstacles_)) {
    width = dimensions[0];
    length = dimensions[1];
    vector<double> car_dimensions ({DEFAULT_CAR_LENGTH, DEFAULT_CAR_WIDTH});
    Pose car_pose ({0, 0, 0});
    car = Car(car_dimensions, car_pose);
}

MapInfo::MapInfo(vector<double> dimensions, vector<double> start_,
                 vector<double> end_, vector<Obstacle *> obstacles_,
                 vector<double> car_dimensions, Pose car_pose):
                 start(move(start_)), end(move(end_)),
                 obstacles(move(obstacles_)) {
    width = dimensions[0];
    length = dimensions[1];
    car = Car(car_dimensions, car_pose);
}

void MapInfo::setCarPose(Pose p) {
    car.setPose(p);
}

vector<Point> MapInfo::getCarOutline() {
    return car.getOutline();
}

// Determine if the outline of the car collides with any obstacles
bool MapInfo::isCollision(vector<Point> car_outline) {
    Vector2f p1, p2;
    for (size_t i = 0; i < car_outline.size(); i++) {
        p1.x() = car_outline[i][0];
        p1.y() = car_outline[i][1];
        p2.x() = car_outline[(i+1) % car_outline.size()][0];
        p2.y() = car_outline[(i+1) % car_outline.size()][1];
        for (auto obstacle: obstacles) {
            if (obstacle->isSegmentInObstacle(p1, p2)) {
                return true;
            }
        }
    }
    return false;
}

//
bool MapInfo::isCollision(vector<double> point) {
    if ((point[0] > 0) && (point[0] < width) &&
        (point[1] > 0) && (point[1] < length))
    {
        Vector2f p;
        p.x() = point[0];
        p.y() = point[1];
        for (auto obstacle: obstacles) {
            if (obstacle->isPointNearObstacle(p, 1.0)) return true;
        }
        return false;
    }
    return true;

}