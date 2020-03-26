#ifndef CONSTANTS_H
#define CONSTANTS_H
// Hybrid AStar search constants
const float STEP_SIZE = 3.0;
const float COMPLETION_THRESHOLD = 1.0;
const float ANGLE_COMPLETION_THRESHOLD = 0.25; // radians ~ 15 degrees
const float RAD_STEP = 1;
const float RAD_UPPER_RANGE = 2;
const float RAD_LOWER_RANGE = 2;

// Map Info constants
const float BOT_CLEARANCE = 0.1;
const float LANE_WIDTH = 5.0;

// Car parameters
const float RADIUS = 6.0; // turning radius
const float DEFAULT_CAR_LENGTH = 4.8;
const float DEFAULT_CAR_WIDTH = 1.8;
#endif // CONSTANTS_H
