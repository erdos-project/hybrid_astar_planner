#include "Car.h"

#include <cmath>

using namespace std;

Car::Car(pair<double, double> dimensions,
         tuple<double, double, double> pose_): pose(pose_) {
    length = dimensions.first;
    width = dimensions.second;
}

vector<pair<double, double>> Car::getOutline() {
    double x, y, yaw;
    double tail_x, tail_y, head_x, head_y;
    pair<double, double> tail_l, tail_r;
    pair<double, double> head_l, head_r;

    x = get<0>(pose);
    y = get<1>(pose);
    yaw = get<2>(pose);

    tail_x = x - cos(yaw) * length / 4.0;
    tail_y = y - sin(yaw) * length / 4.0;
    tail_l = make_pair(
        tail_x + cos(yaw + M_PI_2) * width / 2.0,
        tail_y + sin(yaw + M_PI_2) * width / 2.0
    );
    tail_r = make_pair(
        tail_x + cos(yaw - M_PI_2) * width / 2.0,
        tail_y + sin(yaw - M_PI_2) * width / 2.0
    );

    head_x = x + cos(yaw) * length * 3.0 / 4.0;
    head_y = y + sin(yaw) * length * 3.0 / 4.0;
    head_l = make_pair(
        head_x + cos(yaw + M_PI_2) * width / 2.0,
        head_y + sin(yaw + M_PI_2) * width / 2.0
    );
    head_r = make_pair(
            head_x + cos(yaw - M_PI_2) * width / 2.0,
            head_y + sin(yaw - M_PI_2) * width / 2.0
    );

    vector<pair<double, double>> outline;
    outline.push_back(tail_l);
    outline.push_back(tail_r);
    outline.push_back(head_r);
    outline.push_back(head_l);
    outline.push_back(tail_l);
    return outline;
}
