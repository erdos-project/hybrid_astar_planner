#include <cmath>
#include <iostream>
#include <chrono>

#include "src/utils.h"
#include "src/Car/Car.h"
#include "src/DubinsPath/Dubins.h"
#include "src/HybridAStar/AStar.h"
#include "src/HybridAStar/HybridAStar.h"
#include "src/MapInfo/MapInfo.h"
#include "src/Obstacle/Obstacle.h"

using namespace std;
using namespace Eigen;
using namespace chrono;

int main() {
    // test car
//    vector<double> dimensions;
//    vector<double> pose;
//    dimensions.push_back(2.0);
//    dimensions.push_back(1.0);
//    pose.push_back(1.0);
//    pose.push_back(1.0);
//    pose.push_back(M_PI / 3.0);
//
//    Car car (dimensions, pose);
//
//    vector<vector<double>> outline;
//    outline = car.getOutline();
//    cout << "Car outline is: \n";
//    for (auto point : outline) {
//        cout << "X: " << point[0] << " Y: " << point[1] << endl;
//    }
//    cout << "Expected: \n";
//    cout << "X: 0.316987 Y: 0.816987\n"
//            "X: 1.18301 Y: 0.316987\n"
//            "X: 2.18301 Y: 2.04904\n"
//            "X: 1.31699 Y: 2.54904\n"
//            "X: 0.316987 Y: 0.816987\n";

    // test dubins
//    vector<double> start, end;
//    vector<vector<double>> xy;
//    vector<DubinsPoint> path;
//    double radius = 4.0;
//
//    for (int i = 0; i < 12; i++) {
//        start.clear();
//        end.clear();
//        start.push_back(1.0);
//        start.push_back(1.0);
//        start.push_back(300 / 180.0 * M_PI);
//        end.push_back(-2.0);
//        end.push_back(0.0);
//        end.push_back(i * 30 / 180.0 * M_PI);
//        Dubins dubins (start, end, radius);
//        path = dubins.getShortestPath();
//        if (i == 0) {
//            cout << "Shortest path is: \n";
//            for (auto p: path) {
//                cout << "Direction: " << p.first << " Distance: " << p.second
//                << endl;
//            }
//            cout << "Expected: \n";
//            cout << "Direction: r Distance: 0.370115\n"
//                    "Direction: s Distance: 3.03569\n"
//                    "Direction: r Distance: 4.86587\n";
//        }
//
//        xy = dubins.generatePath(start, path);
//        vector<double> x = xy[0];
//        vector<double> y = xy[1];
//        vector<double> yaw = xy[2];
//    }

    // test map info
//    vector<double> map_dimensions;
//    vector<Obstacle *> obstacles;
//    Obstacle * o;
//    Vector2f ol, ou;
//    ol.x() = 4.0;
//    ol.y() = 4.0;
//    ou.x() = 5.0;
//    ou.y() = 5.0;
//    o = new Obstacle(ol, ou);
//    obstacles.push_back(o);
//    map_dimensions.push_back(40.0);
//    map_dimensions.push_back(20.0);
//    MapInfo map1 (map_dimensions, start, end, obstacles);
//    bool collision = map1.isCollision(outline);
//    cout << "Expecting collision = 0\nCollision is actually: " << collision
//    << endl;

//    delete o;
//    obstacles.clear();
//    ol.x() = 0.5;
//    ol.y() = 0.5;
//    ou.x() = 1.5;
//    ou.y() = 1.5;
//    o = new Obstacle(ol, ou);
//    obstacles.push_back(o);
//    MapInfo map2 (map_dimensions, start, end, obstacles);
//    collision = map2.isCollision(outline);
//    cout << "Expecting collision = 1\nCollision is actually: " << collision
//    << endl;

    // test a star
//    vector<double> dims ({60.0, 40.0});
//    vector<double> s ({10.5, 10.1});
//    vector<double> e ({50.2, 30.7});
//    vector<Obstacle *> obs;
//    for (int i = 0; i < 30; i++) {
//        ol.x() = 20;
//        ol.y() = i;
//        ou.x() = 20;
//        ou.y() = i;
//        o = new Obstacle(ol, ou);
//        obs.push_back(o);
//    }
//    for (int i = 0; i < 30; i++) {
//        ol.x() = 40;
//        ol.y() = 40 - i;
//        ou.x() = 40;
//        ou.y() = 40 - i;
//        o = new Obstacle(ol, ou);
//        obs.push_back(o);
//    }
//    MapInfo *map_info = new MapInfo(dims, s, e, obs);
//    AStar planner (map_info);
//    vector<Point> a_star_path = planner.runAStar();
//    cout << "Expecting endpoint = " << e[0] << " " << e[1] << endl;
//    cout << "Actual endpoint = " << a_star_path.back()[0] << " " <<
//        a_star_path.back()[1] << endl;
//
//    obs.clear();
//    ol.x() = 50.2;
//    ol.y() = 30.7;
//    ou.x() = 50.2;
//    ou.y() = 30.7;
//    o = new Obstacle(ol, ou);
//    obs.push_back(o);
//    map_info = new MapInfo(dims, s, e, obs);
//    planner = AStar(map_info);
//    a_star_path = planner.runAStar();
//    cout << "Expecting path length of 0\n";
//    cout << "Actual path length: " << a_star_path.size() << endl;

    // test hybrid a star
    vector<double> dimensions ({60.0, 60.0});
    vector<double> car_dimensions ({4.8, 1.8});
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

    MapInfo *map_info = new MapInfo(dimensions, start, end, obs,
            car_dimensions, start);

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