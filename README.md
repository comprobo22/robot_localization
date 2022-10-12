# Robot Localization
## Project Overview
The primary goal of this project was to implement a particle filter algorithm in order to localize a robot within a given map. At a base level, the initial position of the robot is known and the particle filter is used to track its motion over time. Taking it one step further, the initial position of the robot is not given. This is known as the kidnapped robot problem, which was the primary target of this project to solve on large-scale maps.

In order to accomplish these tasks, we needed to integrate a variety of data from the TurtleBot4. This data primarily consisted of the laser scan data and the odometry of the robot. The full implementation, described below, can be found in this [github repository](https://github.com/ayushchakra/robot_localization).

Note: Parts of the initial implementation for this project were provided as starter code. This consists of large parts of the ROS Node workflow, the original occupancy field implementation, and the helper functions. This was to help guide the project's focus towards implementing and debugging the particle filter at a conceptual level, rather than spending extensive time on accessories to the workflow.

## Problem-Solving Approach
In order to solve our end goal of the kidnapped robot problem, we chose to implement an iterative approach that started with solving the MVP particle filter problem and slowly incremented until it was able to be tested on the map of the full MAC with no initial position given. 

### MVP Particle Filter
The MVP for the particle filter was to use a particle cloud to track the position of a robot when given its starting position. This form of tracking the robot's position helps reduce the impact of odometry drift while tracking the position of the robot over time. An example implementation of this section is shown below:

### Optimizing Code
Once the MVP particle filter was consistently working, we decided to optimize our code in order to improve its overall runtime. This was motivated by us realizing that we would need to drastically scale up the number of particles used when trying to solve the kidnapped robot problem and that inefficient code would hinder the particle filter's ability to converge on the actual position of the robot. To do this, we converted the majority of our for loops into matrix operations. This is because python's numpy library is very effecient at handling matrix operations as compared to the inefficiencies of an iterative for loop in python. 

### Kidnapped Robot Problem
Once the code had been sufficiently optimized, we shifted our focus to tackle the kidnapped robot problem. The primary change that this entailed was removing code for extracting the initial position of the robot. To compenstate for this loss of data, we drastically increased the number of particles being generated to form the particle cloud and employed a more sophisticated particle cloud intialization strategy (explained in more detail below). Along with careful tuning of odometry variance, we were able to relatively consistently localize the robot within a map of half of a floor of Olin's MAC.

## Particle Filter Workflow
The main workflow of the particle filter is defined in the `run_loop` function of the `ParticleFilter` Node. The main operations executed by `run_loop` are as following:
1. Initialize particle cloud
    - Creates a list of `n_particles` that represents potential "global" coordinates for the actual robot.
2. Process the current scan
    - Converts the current laser scan data from the LiDAR reference frame to the odometry reference frame.
3. Check if the robot has moved past a certain threshold to trigger a particle update
    - Since particle updates are computationally intensive, they can not be continuously called, which is why the robot must move and/or turn by a certain amount to trigger an update.
4. If it has, update each particle's position based on the odometry change.
    - Move each particle by the change in odometry as if it was the actual robot, yielding its updated position.
5. Update the weight of the particle based on the simulated laser scan.
    - Project the obtained laser scan onto each particle and compute how likely the particle is to be the actual pose of the robot based on how well its projected scan correlates to the features of the map.
6. Resample particles
    - Resample particles in favor of more-likely particles to remove unlikely particles and add noise. This makes it more likely that the actual position of the robot will be captured by a particle.
7. Repeat steps 2-6
    - Repeat each step until the robot has completed its path.

## Design Decisions
particle initialization (adding nans to unscanned points), laser scan likelihood function, best particle determination
## Results
gifs: mvp particle filter gauntlet, mac, then full tests of gauntlet mac mac full


## Reflection
challenges/lessons learned

## Next Steps