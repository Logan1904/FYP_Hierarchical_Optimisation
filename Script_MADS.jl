import Polygonal_Method, MonteCarlo_Method
import Position_Optimization
import Functions
import Plotter

# Parameters
N_drones = 8
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

MADS_input = [x_arr;y_arr;R_arr]

objective(z) = Polygonal_Method.Area(z)

# Define objective
function obj(z)
    try
        global Area = objective(z)
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
MADS_problem = Position_Optimization.define_problem(MADS_input, obj, cons_ext, cons_prog, N_iter)
MADS_output = Position_Optimization.optimize(MADS_problem)
MADS_output = Position_Optimization.check_local_minima(MADS_output, obj, cons_ext, cons_prog, N_iter)

# Print areas and plot images
println("First iteration: ",objective(MADS_input))
println("After optimization: ", objective(MADS_output))

Plotter.plot_domain(Functions.make_circles(MADS_input),[domain_x,domain_y],"First_Iteration")
Plotter.plot_domain(Functions.make_circles(MADS_output),[domain_x,domain_y],"Last_Iteration")