# Arduino_SelfDriving_Robot
Software for Arduino self-driving-based RC-car. Participated and won the University competition (2019).

Rise in Costa is an annual competition where the LsMaker must make one of the rises of the park of the university, to try to arrive the first ones in the highest part.
We can carry out all kinds of software and hardware modifications, but always bearing in mind that it must be a fully autonomous robot, so we can only tell the robot when it will begin to move, nothing else.
It is also important to emphasize the fact that during the start of the race, we can have the robot placed in any direction, therefore already from the start it starts calculating where the highest point is to start the journey to the finish line.

How it works: (For a more detailed explanation, head to the memoria.pdf document)
At first, we look at the Y sensor of the accelerometer to determine whether we are flat / inclined upwards, or downwards. In case we are flat or looking up, the situation could still be that we were very deviated from the highest point, so you must look at the axes of x. The axes of X tell us if a wheel is higher than the other. Ideally, they should be matched, therefore 0. After testing we have decided to give a margin of 2 positive and negative, if between these two cases it is straight, and if it is less than -2 or higher at 2, rectify turning to the side that touches him.
If you are looking down, but both wheels are balanced (between -2 and 2) we will turn 180 degrees. In other cases, we will turn to look up to the right or the left, depending on how fast it is. For example, if the right wheel is higher than the left, it will be easier to move the left wheel just to balance it and that it is facing up as fast as possible.



Demostration YT Video:

Testing the car, example of how it's lookinf for the highest point it can go.
[![Watch the video](https://img.youtube.com/vi/5CkfrVYpJJM/maxresdefault.jpg)](https://youtu.be/5CkfrVYpJJM)

Video of the University LSMaker (the robot) finals where I won.
[![Watch the video](https://img.youtube.com/vi/yfT1XwsNTL4/maxresdefault.jpg)](https://youtu.be/yfT1XwsNTL4)
