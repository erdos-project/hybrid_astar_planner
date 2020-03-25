#include "include/HybridAStar.h"

#include <eigen3/Eigen/Dense>
#include <vector>

using namespace Eigen;

extern "C" {
    int ApplyRRTStar(double x_start, double y_start, double x_end, double y_end,
                     double *obstacles_llx, double *obstacles_lly,
                     double *obstacles_urx, double *obstacles_ury,
                     int numObstacles, double *x_path, double *y_path) {
        // Construct obstacles
        vector<Obstacle *> obs;
        Obstacle *o;
        vector<double> llx(obstacles_llx, obstacles_llx + numObstacles);
        vector<double> lly(obstacles_lly, obstacles_lly + numObstacles);
        vector<double> urx(obstacles_urx, obstacles_urx + numObstacles);
        vector<double> ury(obstacles_ury, obstacles_ury + numObstacles);
        for (int i = 0; i < numObstacles; i++) {
            o = new Obstacle(Vector2f(llx[i], lly[i]),
                             Vector2f(urx[i], ury[i]));
            obs.push_back(o);
        }

//        MapInfo *map_info = new MapInfo(dimensions, start, end, obs);



    }
}
