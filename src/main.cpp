#include <cmath>
#include <iostream>
#include "src/Car/Car.h"
#include "src/DubinsPath/DubinsPath.h"

using namespace std;

int main() {
    // test car
//    pair<double, double> dimensions (2.0, 1.0);
//    tuple<double, double, double> pose(1.0, 1.0, M_PI / 3.0);
//    Car car (dimensions, pose);
//
//    vector<pair<double, double>> outline;
//    outline = car.getOutline();
//    for (pair<double, double> point : outline) {
//        cout << "X: " << point.first << " Y: " << point.second << endl;
//    }
//
    // test dubins
    vector<double> start, end;
    vector<vector<double>> xy;
    vector<pair<char, double>> path;
    double radius = 4.0;

    for (int i = 0; i < 12; i++) {
        start.clear();
        end.clear();
        start.push_back(1.0);
        start.push_back(1.0);
        start.push_back(300 / 180.0 * M_PI);
        end.push_back(-2.0);
        end.push_back(0.0);
        end.push_back(i * 30 / 180.0 * M_PI);
        DubinsPath dubins (start, end, radius);
        path = dubins.getShortestPath();
        xy = dubins.generatePath(start, path);
        vector<double> x = xy[0];
        vector<double> y = xy[1];
        vector<double> yaw = xy[2];
        cout << x.size() << endl;
        for (auto xp : x) {
            cout << xp << ", ";
        }
        cout << endl;
        for (auto yp : y) {
            cout << yp << ", ";
        }
    }
}