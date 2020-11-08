  #include "car.h"
  /**
   * Returns true if this car is ahead of other car by at least dist after time
   **/ 
  bool Car::isAheadByAtLeast(Car other, float dist) {
    return s_projected - other.s_projected > dist;
  }