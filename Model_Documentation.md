# **Path Planning Project** 

## Reflection


---

**How to generate paths**

The strategy chosen for generating path was to use a spline tool for interpolating points, as suggested in the Project instructions and in the Project Q&A. 

The starting step was to use the telemetry data and get the 
car's x and y positions and the yaw angle. These values were used to calculate the first two points of the car trajectory, either using the last two points from the previous cycle or the car position and angle. 

The previous path is a list of points that the car hasn't already visited from the previous cycle. 
I decided to stick with a 50 points path, as in the project instructions, so all the remaining points from the previous cycle were added to the output list so I only had to compute and append new waypoints until the path had 50 in total. 

To generate new points, the car position in Frenét s coordinates was used to compute 3 anchor points in map coordinates that would be used to interpolate a spline. These 3 points would be equally distributed at 30, 60 and 90 meters in the s direction from the car's position. 
By using a provided helper function (getXY), an x and y coordinate was generated from each anchor point in map coordinates.

To make it easier to generate a path for the car, the x and y anchor points were shifted and rotated to be in car's local coordinates system. A spline was then generated from them, that would be able to associate a y value from a given x value and make them fit a desired line similar to the path the car should follow. 

To compute these points, by following the project Q&A discussion, a point in the horizon was chosen at 30 meters from the car and its corresponding y coordinate was generated from the spline (line 318). The distance of that target point from the car could be thought of as the hypotenuse of a hypothetical triangle where the opposite side is the x axis. The distance could then be calculated by using the Pythagorean problem and then split into a number of steps, each one delimited by points. 

The number of steps was calculated by dividing the target distance by the car velocity at each cycle. 
Then new path coordinates were generated and added to the points from the previous path, until the output list had a length of 50. 

The last part of the project was to develop the logic to change lanes. 
The first step was to declare a boolean value "is_too_close" that would be set to true if our car were closer than 30 meters to a vehicle in front of it on the same lane. 
This variable would be checked at every cycle and, when true, it would trigger a velocity decrease of about 10 meters/second. In case this flag variable were false, the car would keep its velocity close to 50 MPH, increasing it when needed. 

To set the "is_too_close" flag on and off, the sensor fusion data was used to retrieve the "d" coordinate for every other car in sensor range in the road. 
The "d" coordinate can be used to find which lane a car is in, as it measures a car's distance from the center of the road. By using that other car's speed and s position, it was possible to evaluate its distance from our car. 
The same logic was used to find if the lane to the right and to the left of our car was free from vehicles within 15 meters ahead or behind our car. 
