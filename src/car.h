#ifndef CAR_H_
#define CAR_H_

class Car {
  public:  
    float vx;
    float vy;
    float d;
    float s;
    float s_projected;
    float lane;
    float speed;
  
    /**
     * Returns true if this car is ahead of other car by at least dist after time
     **/ 
    bool isAheadByAtLeast(Car other, float dist);
};

#endif