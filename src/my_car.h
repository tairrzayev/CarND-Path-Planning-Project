#ifndef MY_CAR_H_
#define MY_CAR_H_

#include "car.h"

class MyCar {
  public:
    double x;
    double y;
    double s;
    double d;
    double yaw;
    double speed;
    int lane;
    double end_path_s;
    int prev_path_size;
    double ref_vel;


bool isInMyLaneAndTooClose(Car other);

/**
 * Returns true if it is safe to change the lane given there is
 * a car in that lane
 */
bool isSafeToChangeIntoCarsLane(Car carInTargetLane);

 /**
  * Returns true, if it is safe to overtake the carInFront, given that there
  * is also a car in the target lane.
  */
  bool isSafeToChangeTheLane(Car carInFront, Car carInTargetLane);
};

#endif