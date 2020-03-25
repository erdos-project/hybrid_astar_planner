#include <iostream>
#include <chrono>

#include "include/utils.h"
#include "include/HybridAStar.h"
#include "include/MapInfo.h"
#include "include/Obstacle.h"

using namespace std;
using namespace Eigen;
using namespace chrono;

int main() {
    // test hybrid a star
    vector<double> dimensions ({60.0, 60.0});
    Pose start ({10.0, 15.0, 0.0});
    Pose end ({50.0, 15.0, 0.0});

    vector<Obstacle *> obs;
    Obstacle *o;
    Vector2f ol, ou;

    ol.x() = 28.0;
    ol.y() = 13.0;
    ou.x() = 32.0;
    ou.y() = 17.0;
    o = new Obstacle(ol, ou);
    obs.push_back(o);

    ol.x() = 28.0;
    ol.y() = 20.0;
    ou.x() = 32.0;
    ou.y() = 24.0;
    o = new Obstacle(ol, ou);
    obs.push_back(o);

    for (int i = 0; i < 60; i+=1) {
        ol.x() = i;
        ol.y() = 12;
        ou.x() = i;
        ou.y() = 12;
        o = new Obstacle(ol, ou);
        obs.push_back(o);
    }

    MapInfo *map_info = new MapInfo(dimensions, start, end, obs);

    HybridAStar hastar (map_info, RADIUS);
    auto st = high_resolution_clock::now();

    vector<Pose> path = hastar.runHybridAStar();
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - st);
    cout << "Duration (ms): " << duration.count() << endl;
    cout << "Expected destination: " << end[0] <<  " " << end[1] << " " <<
    end[2] << endl;
    cout << "Reached: " << path.back()[0] << " " << path.back()[1] <<  " "
    << path
    .back()[2] << endl;
    cout << "[";
    for (auto p : path) {
        cout << "[" << p[0] << ", " << p[1] << ", " << p[2] << "], ";
    }
    cout << "]";
}