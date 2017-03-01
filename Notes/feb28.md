# Goal
Have a camera be able to pick out the row in a crop. From that create a guide for the robot to follow.

#Motivation
Using GPS as the only means for robot navigation in crops has problems. Firstly it assumes the user will enter every
single GPS coordinate for the robot to follow. This would be required for any rows that have a curve. Generally the user
would want to places only the GPS coordinates at the end of each row. Even if the user places every point this leads to 
the second problem GPS systems, even RTK, are not perfect and will have errors. Having some means to correct the small
deviations in the GPS and even the drifting of the robot is vital.


