# FYP_Optimization
FYP - Distributed Optimization for Multi-Agent Systems

Problem: Optimization of drone swarm to maximise area coverage over time, while minimizing trajectory energy cost

## Polygonal_Method.jl
Algorithm that breaks up the circles into polygons and circular sectors, based on the StackOverflow post:
https://stackoverflow.com/questions/1667310/combined-area-of-overlapping-circles, Ants Aasma & Timothy Shields

## MonteCarlo_Method.jl
Approximation method using the MonteCarlo method

## Functions.jl
Contains all the base functions used

## Plotter.jl
Plots our described problem for visualisation

## Optimization.jl
Perform MADS optimization on our problem to maximise area