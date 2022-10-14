# Robot Localization Writeup #

## What was the goal of your project? ##

The goal of this project was, given a limited set of data, estimate the location of our robot. In our simulation, the robot has no initial idea where it is. While we drive the robot around a predefined map, the program gets access to the robot’s LIDAR data as well as some relative location data (how far the robot moved since the previous position was published). This data is enough for us to make estimations of the robot’s location and over time, refine the accuracy of said estimation.

## How did you solve the problem? ##

We took the approach discussed in class to solve this problem (i.e. laying out a field of particles with varying positions and orientations around an initial pose to establish which one had the highest probability of being the robot). The first stage of which as illustrated in figure 1 is laying out the initial particle field. 

| ![space-1.jpg](https://blog-assets.thedyrt.com/uploads/2019/01/shutterstock_1033306540-1.jpg) |
|:--:|
| <b>Figure 1. Placing Initial Particle Cloud Around an Initial Pose.</b>|
 
Our code relies on the user providing an initial position and orientation they believe the robot to be at or otherwise the code just uses the current odometry of the Neato. The user chooses a number of particles placed according to a standard deviation around this initial guess. A standard deviation approach was chosen because it allows us to place most of our points around the area where we have the most confidence while providing some ability for checks of widely varying locations. This helps protect us if our initial guess is inaccurate. 
 
Once this was completed, we had to treat each point as if it was the Neato and examine how its position and orientation would match the lidar data we are constantly receiving from the Neato. However, this raises the fundamental question that if we do not know where the Neato is, how can we possibly evaluate the particles? Well, we know the 360 distance values we received from the Lidar are distances until the laser hit an obstacle. Therefore, we can take these distances and their angles and fit them as if they are emanating from each particle as presented in Figure 2.

| ![space-1.jpg](https://blog-assets.thedyrt.com/uploads/2019/01/shutterstock_1033306540-1.jpg) |
|:--:|
| <b>Figure 2. Lidar Scans in Green Fitted to Particles</b>|
 
Now, if the particle is exactly aligned with the Neato and if we draw the lidar vectors according to their distance and the particle’s orientation, the distance to an obstacle at the end of every one of these vectors should be zero. Again, this is because a lidar return means the laser encountered an obstacle it has bounced off of. Therefore, the end of each lidar vector should be right on an obstacle. Of course, it is highly improbable that we will receive a perfect match but adding all of the distances can give us a margin of error and probability of how close a particle is to the Neato.  
 
This brings us to the step where we evaluate the weight of every particle based on these distances with a total distance to obstacles per point closer to zero meaning a higher weight. We chose to take the weighted mean of every particle after they had been normalized to 1, so we multiplied the position and orientations by their weight and then took the overall average. An example of how this would look is shown in Figure 3.

| ![space-1.jpg](https://blog-assets.thedyrt.com/uploads/2019/01/shutterstock_1033306540-1.jpg) |
|:--:|
| <b>Figure 3. Weighting the Likelihood That Each Particle is the Neato</b>|
 
A weighted average approach was taken mainly because it made sense given the fact that all of our weights were already normalized to sum to 1. Therefore, they were already perfect to multiply against their matching position and orientation without additional math. This weighted average gives us a good idea of where the Neato is.
 
All that is left at this point is to resample our particles after the Neato has moved. To do this we examined the odometry frame of the Neato and the relative change in angle and distance before extrapolating this to every particle. This process is illustrated in Figure 4.

| ![space-1.jpg](https://blog-assets.thedyrt.com/uploads/2019/01/shutterstock_1033306540-1.jpg) |
|:--:|
| <b>Figure 4. Transformation of Particle Based on Odometry Information</b>|
 
We do not know how the odometry frame relates to the map frame, but since we only care about the relative change in orientation and distance, this does not matter. The basic concept is we know the neato is now at some position say C with a new X, Y, and orientation. With the change in odometry position, we know the Neato had to turn some angle Theta, move some distance D, and then turn again some angle Phi to arrive at this new position. It is really important we apply this relative process to every point. For example, if we just shifted every particle and then turned them, we would be assuming all of our particles had the same orientation instead of being the distinct Neato projections they are. 
 
Thus, we must turn every particle by Theta given its starting orientation. Then given this heading we must make it move along its distance D, and then change the heading again due to Phi. It is vital this is done as each particle is a representation that it is completely confident in its position and orientation and serves as an independent tester of its confidence. 
 
Now that we have shifted our guess of where the particle is. We resample all of our particles again around the weighted averages we calculated earlier. If the weighted sample was accurate, the points should continue to become closer and closer to the Neato’s true position. As we already have calculated the weighted averages, everything past this step is a rinse and repeat of the methods aforementioned.

## Describe a design decision you had to make when working on your project and what you ultimately did (and why)? ##

The only two major design decisions we made were to use a weighted average when deciding our position and orientation and to use standard deviation when placing our particle cloud. Other than this, we primarily followed the suggested methods explored in class. However, from meeting with Paul, we learned that these were not the best methods to approach this problem. Firstly, using the weighted average within itself is not problematic but it became less than ideal when we resampled all of our points around this average. Particle clouds excel when they have uncertainty calculated into each of the particles and then are sampled around each particle, according to its wait. By just resampling around the weighted average of all of the x, y, and theta of all of the points, we were effectively robbing our initial laser scan of this chance to double check our assumptions, placing all of our certainty into one point. We later changed our code to use the provided function to resample the particles around each previous particle, according to their weights.
 
Our second method of using standard deviation to generate a particle cloud was mostly unchanged over the course of our project except for the fact of the step size of our deviations. Our initial choice for the standard deviations was pi/3 for theta and 1 meter for the x and y coordinates. We soon realized this was far too large as the far end of the distribution c=values produced could be far off the map or facing the completely wrong direction. Therefore, we eventually tuned these values to pi/30 standard deviation for theta and .1 meter for the x and y coordinates. It was interesting to discover that theta was far less forgiving than the coordinates, so a much smaller distribution was required. Overall, these distribution sizes worked well with the Neato, allowing it place particles that had a decent amount of uncertainty while still being in the realm of possibility.

## What if any challenges did you face along the way? ##

The group faced a lot of challenges along the way. One of the first problems we faced early on was writing our function to update the particle based on odometry. It took my group a long time to understand all the math that related the movement of the particles in the odometry frame and the map frame. The next major challenge was that our code was producing a lot of nan values which was causing our quaternion to be nan and our code to error out. It took a lot of time to figure out where the nan values were coming from. In the end it came from one of our functions returning a nan value when the input was out of bounds on the map.

Probably our biggest struggle though was with testing and visualizing our code. The primary method of testing our code was using rviz2. However, because of the way rviz2 works, our code had to be mostly complete to interface with rviz2. This means that we wrote the majority of our code before testing it. Once we did begin testing it, things immediately went wrong and it took us a really long time to track down the problems. We had to use a lot of print based debugging to get our code to a point where it would work in rviz2.

## What would you do to improve your project if you had more time? ##

Some improvements we would like to look into given more time revolve around optimization. Right now our program utilizes several loops that iterate through and perform actions on each element in an array instead of utilizing array operations more frequently. This is likely something that could save us compute time and make our program run faster. We would also like to look into optimizing the randomness of how our points are initialized/redistributed to see what effect, if any, it has on the runtime and confidence of the resulting location. Last but not least, we want to make our code more robust when it comes to handling edge cases. For example, we ran into a lot of issues handling cases where a generated point was initialized outside of the map.

## Did you learn any interesting lessons for future robotic programming projects? ##

One lesson we learned was the importance of pseudocode. When we were trying to solve problems like updating the particles with odometry, it was really helpful to break the problems out into smaller pieces and then walk through those pieces step by step to write the code. Another lesson was the importance of visualizing and testing the results of the code early and often. We did not do that on this problem and it was really overwhelming to get to the end of the project and have a whole bunch of errors stacked on top of errors that we didn’t know how to fix. If we had tested in smaller chunks, we likely would have been able to catch a lot of the errors earlier on. 

More generally, 3 people was a lot for this project. Our meeting times were not always the most effective. It may have been more effective to have less people, do more asynchronous work, or meet in smaller groups. 3 people is a lot to get around 1 laptop screen and still have everyone contribute. 
