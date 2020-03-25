#include "include/MapInfo.h"
#include "include/constants.h"

#include <eigen3/Eigen/Dense>
#include <utility>

using namespace Eigen;

// Arguments:
//      dimensions: length, width
//      start_: starting pose of car x, y, yaw
//      end_: desired ending pose of car x, y, yaw
//      obstacles: list of obstacles
MapInfo::MapInfo(vector<double> dimensions, Pose start_,
                 Pose end_, vector<Obstacle *> obstacles_):
                 start(move(start_)), end(move(end_)),
                 obstacles(move(obstacles_)) {
    width = dimensions[0];
    length = dimensions[1];
    vector<double> car_dimensions ({DEFAULT_CAR_LENGTH, DEFAULT_CAR_WIDTH});
    Pose car_pose (start);
    car = Car(car_dimensions, car_pose);
}

// Set the pose of the car
// Arguments:
//      p: pose vector consisting of global x, y, yaw
// Returns:
//      none
void MapInfo::setCarPose(Pose p) {
    car.setPose(p);
}

vector<Point> MapInfo::getCarOutline() {
    return car.getOutline();
}

// Determine whether the car outline intersects an obstacle
// Arguments:
//      car_outline: x, y outline of the car
// Returns:
//      bool indicating whether the car outline intersects an obstacle
bool MapInfo::isCollision(vector<Point> car_outline) {
    Vector2f p1, p2;
    for (size_t i = 0; i < car_outline.size(); i++) {
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

// Determine whether the point is near an obstacle
// Arguments:
//      point: x, y global location
// Returns:
//      bool indicating whether the point is near an obstacle
bool MapInfo::isCollision(Point point) {
    if ((point[0] > 0) && (point[0] < width) &&
        (point[1] > 0) && (point[1] < length))
    {
        Vector2f p;
        p.x() = point[0];
        p.y() = point[1];
        for (auto obstacle: obstacles) {
            if (obstacle->isPointNearObstacle(p, BOT_CLEARANCE)) return true;
        }
        return false;
    }
    return true;

}