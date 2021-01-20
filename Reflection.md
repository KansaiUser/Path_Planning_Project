# Reflection file.

Note: It is clearly indicated in the Project Specification that
"The code model for generating paths is described in detail. **This can be part of the README** or a separate doc labeled "Model Documentation"."

The code model for generating paths is *already* described in detail in the README. For convenience I repeat the explanation here:


## Code Model for Generating Paths

In this project the code for generating paths is inside the function `avoid_collisions` in the `helpers.h` file.  

This function receives as input the sensor fusion data (where all other cars are), the lane where the ego_car is, the previous path size, the tip of the future ego_car and the actual s value of the ego_car. 
As output it returns a boolean which indicates if the car in front is "too close" and also modifies the lane that the car has to follow.

In order to do these modifications it works like this:

1. First, if the car is on the left lane, then it is not possible to move left
2. If the car is on the right lane it is not possible to move right

3. For information of all surrounding cars (other_car):

  * if the other_car is to the left check if it is ahead. 
  * if it is ahead and it is too close then the left lane is closed
  * if it is not too close get the left_space distance
  * if it is behind and its future is too close then the left lane is closed

  The same for the right
    
  * if the other_car is to theright check if it is ahead. 
  * if it is ahead and it is too close then the right lane is closed
  * if it is not too close get the right_space distance
  * if it is behind and its future is too close then the right lane is closed

  * If the other car is on the same lane and it is too close, set the too_close value to true

4. With this information, it is time to decide where to go

  * If the ego_car is on the left and the right lane is open go to the right
  * If the ego_car is on the right and the left lane is open go to the left
  * If the ego_car is in the center :
     * If both lanes are open, and the left_space distance is bigger than the right one go left otherwise go right
     * If only the left lane is open go left
     * If only the right lane is open go right

With this simple strategy, we have decided as an ego_car to go left, right or stay on the lane (reducing the speed)



