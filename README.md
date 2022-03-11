# FYP_Optimization
FYP - Distributed Optimization for Multi-Agent Systems

Problem: Optimization of drone swarm to maximise area coverage over time, while minimizing trajectory energy cost

## Modules
### Polygonal_Method.jl (Module)
Function to obtain the union of the area of N intersecting circles
Algorithm breaks up the circles into polygons and circular sectors, based on the StackOverflow post:
https://stackoverflow.com/questions/1667310/combined-area-of-overlapping-circles, Ants Aasma & Timothy Shields

### MonteCarlo_Method.jl (Module)
Calculation of the area of N intersection circles
Approximation method using MonteCarlo method

### Functions.jl (Module)
Contains all the base functions used

### Plotter.jl (Module)
Plots our described problem for visualisation

### Position_Optimization.jl (Module)
Positional optimization using DirectSearch.jl

### Trajectory_Optimization.jl (Module)
Trajectory optimization using RobotDynamics.jl, TrajectoryOptimization.jl and Altro.jl (model defined with RobotZoo.jl)

## Scripts
### Script_MADS.jl
Script to run the positional optimization
1) Generate N random circles in our defined domain
2) Use MADS (objective function defined by Polygonal_Method.jl) to find optimum position of circles that maximise area

### Script_Altro.jl
Script to run the trajectory optimization
Given the initial and final positions from Script_MADS.jl, find the optimal trajectories from initial to final positions

## How to run problem
Run Script_MADS.jl, then without clearing the workspace, run Script_Altro.jl

Work is being done on integrating the 2 scripts into one single script, which loops until the entire domain has been covered