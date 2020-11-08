## Path generation description

This model is based on the code and suggestions from the [Project Q&A section](https://classroom.udacity.com/nanodegrees/nd013/parts/01a340a5-39b5-4202-9f89-d96de8cf17be/modules/1dc566d7-03d4-40da-af2c-b8ec85f2e4dd/lessons/407a2efa-3383-480f-9266-5981440b09b3/concepts/3bdfeb8c-8dd6-49a7-9d08-beff6703792d)

- Spline library is used to build smooth trajectories by interpolating sparsely spaced points
- As per suggestion from the Q&A Section, the new path is not re-created on each iteration but extended instead, re-using the remaining points from the previous path
- The algorithm (described below) operates in the Frenet frame of reference which greatly simplifies the implementation.

## Algorithm description:

The model favours driving in the current lane unless it's necessary to overtake the car infront.

For each car in my lane (main.cpp, Line 132), check if the car is close enough to our car to consider the lane change, otherwise stay in our current lane.

When changing lane, we will always first check the left lane (which in real world is usually faster) before the right.

Before changing lane we have to consider the following:

- It's safe to change the lane (we will not run into any car when doing that) - my_car.cpp, Line 20.
- It's rational to change the lane. E.g. if there is a car in the target lane, driving with the same speed & at approx. the same S as the car in front of us, it does not make sense to change the lane as we will get stuck behind the car again, just in another lane - my_car.cpp, Line 23. 

If we have successfully changed the lane, stick to the max speed limit.

If it was not safe or rational to change lane, stay behind the car in our lane, adjusting our speed to match the speed of the car in front  (main.cpp, Line 175)