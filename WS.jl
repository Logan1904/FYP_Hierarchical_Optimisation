import MonteCarlo_Method
#import Polygonal_Method
import Position_Optimization
import Plotter
import Functions

# Parameters
N_drones = 10
domain_x = 50
domain_y = 50
R_lim = 10

# MADS Parameters
N_iter = 100

# Generate circles
x_arr = Vector{Float64}(undef,0)
y_arr = Vector{Float64}(undef,0)
R_arr = Vector{Float64}(undef,0)

for i in range(1,stop=N_drones)
    x = rand(R_lim:domain_x-R_lim)[1]
    y = rand(R_lim:domain_y-R_lim)[1]
    R = rand(1:R_lim)[1]


    push!(x_arr,x)
    push!(y_arr,y)
    push!(R_arr,R)
end

z = [x_arr;y_arr;R_arr]


# Define objective
function obj(z)
    try
        global Area = Polygonal_Method.Area(z)
    catch e
        global Area = 0.0
    end
    return -Area
end

# Define extreme and progressive constraints
include("Constraints.jl")
cons_ext = [cons1,cons2]
cons_prog = []

# Position optimization
MADS_problem = Position_Optimization.define_problem(z, obj, cons_ext, cons_prog, N_iter)
MADS_output = Position_Optimization.optimize(MADS_problem)
#MADS_output = Position_Optimization.check_local_minima(MADS_output, obj, cons_ext, cons_prog, N_iter)

#z = MADS_output

# Print areas and plot images
a = Polygonal_Method.Area(z)
b = MonteCarlo_Method.Area(z,100000)
println("Polygonal: ",a)
println("MonteCarlo: ",b)
println("Error: ",abs(a-b))

circles = Functions.make_circles(z)

Plotter.plot_all(circles, [], [], [], [domain_x,domain_y], "debug")

Polygonal_Method.check_coincident!(circles)

circles = Polygonal_Method.check_contained!(circles)

Polygonal_Method.check_partially_contained!(circles)

Plotter.plot_all(circles, [], [], [], [domain_x,domain_y], "debug2")

intersections = Polygonal_Method.intersection_points(circles)

Polygonal_Method.boundary_ID!(circles,intersections)

intersections = Polygonal_Method.form_contour_points!(circles,intersections)

contour = Polygonal_Method.form_contour(circles)

sectors = Polygonal_Method.form_sectors(circles,intersections,contour)

g, f, polygons, nodes = Polygonal_Method.form_polygons(circles,intersections,contour)

Plotter.plot_all(circles, intersections, polygons, sectors, [domain_x,domain_y], "debug3")

using Graphs, GraphRecipes
label = [i for i in 1:nv(g)]

if ne(f) > 0
    graphplot(f, curves=false, names=label, nodesize=0.25, nodeshape=:hexagon)
end