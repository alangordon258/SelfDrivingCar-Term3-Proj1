#ifndef coefficients_h
#define coefficients_h
#define TRACK_LENGTH 6945.554
#define VEHICLE_SIZE 2.0

// Number of time steps to use for non-ego car predictions
#define N_PREDICTIONS 50

#define SPEED_LIMIT 21.85
#define LANE_CHANGE_BUFFER 6

// Weights for cost functions

#define PROXIMITY_COST_WEIGHT 100
#define LANE_PROXIMITY_COST_WEIGHT 6000
#define COLLISION_COST_WEIGHT 100000
#define LANE_SPEED_COST_WEIGHT 4000
#define NOT_MIDDLE_LANE_COST_WEIGHT 400
#define LANE_CHANGE_COST_WEIGHT 400
#endif
