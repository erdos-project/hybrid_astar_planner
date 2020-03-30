#ifndef CONSTANTS_H
#define CONSTANTS_H
// Hybrid AStar search constants
const float STEP_SIZE = 3.0;                    // meters
const float COMPLETION_THRESHOLD = 1.0;         // meters
const float ANGLE_COMPLETION_THRESHOLD = 0.25;  // radians ~ 15 degrees
const float RAD_STEP = 1;                       // meters (turning radius)
const float RAD_UPPER_RANGE = 2;                // meters (turning radius)
const float RAD_LOWER_RANGE = 2;                // meters (turning radius)

// Map Info constants
const float BOT_CLEARANCE = 0.1;                // meters
const float LANE_WIDTH = 5.0;                   // meters

// Car parameters
const float RADIUS = 6.0;                       // meters (turning radius)
const float DEFAULT_CAR_LENGTH = 4.8;           // meters
const float DEFAULT_CAR_WIDTH = 1.8;            // meters
#endif // CONSTANTS_H
