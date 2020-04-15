#include "include/MapInfo.h"

#include <utility>

// Arguments:
//      dimensions: length, width
//      start_: starting pose of car x, y, yaw
//      end_: desired ending pose of car x, y, yaw
//      obstacles: list of obstacles
MapInfo::MapInfo(HybridAStarInitialConditions *hastar_ic_,
    HybridAStarHyperparameters *hastar_hp_) {
    hastar_ic = hastar_ic_;
    hastar_hp = hastar_hp_;
    setStateSpace();
    setObstacles();
    vector<double> car_dimensions ({hastar_hp->car_length, hastar_hp->car_width});
    Pose car_pose (start);
    car = Car(car_dimensions, car_pose);
}

MapInfo::~MapInfo()
{
    for (auto obstacle : obstacles) {
        delete obstacle;
    }
    obstacles.clear();
    obstacles.resize(0);
}

// Set the valid state space for search
// State space calculated as minimum bounding rectangle with buffer
// origin is lower left corner, bounds is width, height
void MapInfo::setStateSpace() {
    start.assign({hastar_ic->x_start, hastar_ic->y_start, hastar_ic->yaw_start});
    end.assign({hastar_ic->x_end, hastar_ic->y_end, hastar_ic->yaw_end});
    origin.push_back(min(start[0], end[0]) - hastar_hp->lane_width); // x
    origin.push_back(min(start[1], end[1]) - hastar_hp->lane_width); // y
    bounds.push_back(max(start[0], end[0]) - origin[0] + hastar_hp->lane_width);
    bounds.push_back(max(start[1], end[1]) - origin[1] + hastar_hp->lane_width);
}

void MapInfo::setObstacles() {
    // Construct obstacles
    vector<double> llx(hastar_ic->o_llx, hastar_ic->o_llx + hastar_ic->no);
    vector<double> lly(hastar_ic->o_lly, hastar_ic->o_lly + hastar_ic->no);
    vector<double> urx(hastar_ic->o_urx, hastar_ic->o_urx + hastar_ic->no);
    vector<double> ury(hastar_ic->o_ury, hastar_ic->o_ury + hastar_ic->no);

    for (int i = 0; i < hastar_ic->no; i++) {
        addObstacle(
            Vector2f(llx[i], lly[i]),
            Vector2f(urx[i], ury[i])
        );
    }
}

void MapInfo::addObstacle(Vector2f first_point, Vector2f second_point) {
    obstacles.push_back(new Obstacle(std::move(first_point),
                                     std::move(second_point),
                                     hastar_hp->obstacle_clearance));
}

// Set the pose of the car
// Arguments:
//      p: pose vector consisting of global x, y, yaw
// Returns:
//      none
void MapInfo::setCarPose(Pose p) {
    car.setPose(p);
}

// Return the outline of the car as a vector of x, y points
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
        if (isOutOfBounds(p1)) {
            return true; // p2 will be checked in loop
        }
        for (auto obstacle: obstacles) {
            if (obstacle->isSegmentInObstacle(p1, p2)) return true;
        }
    }
    return false;
}

// Determine if point is outside the bounds
bool MapInfo::isOutOfBounds(Vector2f p) {
    return (p.x() < origin[0]) ||
           (p.x() > origin[0] + bounds[0]) ||
           (p.y() < origin[1]) ||
           (p.y() > origin[1] + bounds[1]);
}

// Get the map area
double MapInfo::getMapArea() {
    return bounds[0] * bounds[1];
}