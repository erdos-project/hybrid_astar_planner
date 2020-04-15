#ifndef HYBRID_ASTAR_PLANNER_PY_CPP_STRUCT_H
#define HYBRID_ASTAR_PLANNER_PY_CPP_STRUCT_H
const int MAX_PATH_LENGTH = 100;

struct HybridAStarInitialConditions {
    double x_start;
    double y_start;
    double yaw_start;
    double x_end;
    double y_end;
    double yaw_end;
    double *o_llx;
    double *o_lly;
    double *o_urx;
    double *o_ury;
    int no;
};

struct HybridAStarReturnValues {
    int success;
    double x_path[MAX_PATH_LENGTH];
    double y_path[MAX_PATH_LENGTH];
    double yaw_path[MAX_PATH_LENGTH];
};

struct HybridAStarHyperparameters {
    double step_size;
    int max_iterations;
    double completion_threshold;
    double angle_completion_threshold;
    double rad_step;
    double rad_upper_range;
    double rad_lower_range;
    double obstacle_clearance;
    double lane_width;
    double radius;
    double car_length;
    double car_width;
};
#endif //HYBRID_ASTAR_PLANNER_PY_CPP_STRUCT_H
