#include "include/HybridAStar.h"

#include <eigen3/Eigen/Dense>
#include <vector>

using namespace Eigen;

extern "C" {
    int ApplyHybridAStar(double x_start, double y_start, double yaw_start,
                         double x_end, double y_end, double yaw_end,
                         double *obstacles_llx, double *obstacles_lly,
                         double *obstacles_urx, double *obstacles_ury,
                         int numObstacles,
                         double *x_path, double *y_path, double *yaw_path) {
        Pose start({x_start, y_start, yaw_start});
        Pose end({x_end, y_end, yaw_end});


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

        MapInfo *map_info = new MapInfo(start, end, obs);
        HybridAStar hastar (map_info, RADIUS);
        vector<Pose> path = hastar.runHybridAStar();

        // Reconstruct Hybrid AStar path
        int index = 0;
        for (auto p : path) {
            x_path[index] = p[0];
            y_path[index] = p[1];
            yaw_path[index] = p[2];
            index += 1;
        }
        x_path[index] = NAN;
        y_path[index] = NAN;

        // Only map_info contains pointers that need to be freed
        delete map_info;
        return !path.empty();
    }
}
