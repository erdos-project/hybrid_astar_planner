#include "include/Car.h"

#include <cmath>

using namespace std;

Car::Car() {}

Car::Car(vector<double> dimensions,
         Pose pose_): pose(pose_) {
    length = dimensions[0];
    width = dimensions[1];
}

// Set the pose of the car
void Car::setPose(Pose p) {
    pose = p;
}

// Compute the outline of the car given its current pose
vector<Point> Car::getOutline() {
    double x, y, yaw;
    double tail_x, tail_y, head_x, head_y;
    vector<double> tail_l, tail_r;
    vector<double> head_l, head_r;

    x = pose[0];
    y = pose[1];
    yaw = pose[2];

    tail_x = x - cos(yaw) * length / 4.0;
    tail_y = y - sin(yaw) * length / 4.0;
    tail_l.push_back(tail_x + cos(yaw + M_PI_2) * width / 2.0);
    tail_l.push_back(tail_y + sin(yaw + M_PI_2) * width / 2.0);
    tail_r.push_back(tail_x + cos(yaw - M_PI_2) * width / 2.0);
    tail_r.push_back(tail_y + sin(yaw - M_PI_2) * width / 2.0);

    head_x = x + cos(yaw) * length * 3.0 / 4.0;
    head_y = y + sin(yaw) * length * 3.0 / 4.0;
    head_l.push_back(head_x + cos(yaw + M_PI_2) * width / 2.0);
    head_l.push_back(head_y + sin(yaw + M_PI_2) * width / 2.0);
    head_r.push_back(head_x + cos(yaw - M_PI_2) * width / 2.0);
    head_r.push_back(head_y + sin(yaw - M_PI_2) * width / 2.0);

    vector<Point> outline;
    outline.push_back(tail_l);
    outline.push_back(tail_r);
    outline.push_back(head_r);
    outline.push_back(head_l);
    outline.push_back(tail_l);
    return outline;
}

