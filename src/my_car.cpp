#include "my_car.h"
#include <cmath>

bool MyCar::isInMyLaneAndTooClose(Car other) {
  if (lane != other.lane) {
    return false;
  }

  float car_s = other.s_projected;
  return car_s > s && car_s - s < 30;
}

bool MyCar::isSafeToChangeIntoCarsLane(Car other) {
  float s_diff = other.s_projected - s;
  // The car should either be far enough behind our car or far enough infront.
  return s_diff < -5 || s_diff > 30;
}

bool MyCar::isSafeToChangeTheLane(Car carInFront, Car carInTargetLane) {
  return isSafeToChangeIntoCarsLane(carInTargetLane) 
  // If the car in target lane is near the car in front (their S is close), it only makes sense to change
  // the lane if its velocity is larger.
    && (std::abs(carInTargetLane.s_projected - carInFront.s_projected) > 5 || carInTargetLane.speed > carInFront.speed);
}