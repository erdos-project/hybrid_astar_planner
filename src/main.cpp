#include <cmath>
#include <iostream>
#include "src/Car/Car.h"

using namespace std;

int main() {
    // test car.py
    pair<double, double> dimensions (2.0, 1.0);
    tuple<double, double, double> pose(1.0, 1.0, M_PI / 3.0);
    Car car (dimensions, pose);

    vector<pair<double, double>> outline;
    outline = car.getOutline();
    for (pair<double, double> point : outline) {
        cout << "X: " << point.first << " Y: " << point.second << endl;
    }
}