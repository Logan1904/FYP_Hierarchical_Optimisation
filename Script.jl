import Polygonal_Method
import Position_Optimization, Trajectory_Optimization
import Functions
import Plotter

# Parameters
N_drones = 12
domain_x = 50
domain_y = 50
R_lim = 10

# MADS Parameters
N_iter = 50

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
MADS_problem = Position_Optimization.define_problem(MADS_input, obj, cons_ext, cons_prog, N_iter)
MADS_output = Position_Optimization.optimize(MADS_problem)
MADS_output = Position_Optimization.check_local_minima(MADS_output, obj, cons_ext, cons_prog, N_iter)


# Print areas and plot images
println("First iteration: ",Polygonal_Method.Area(MADS_input))
println("After optimization: ", Polygonal_Method.Area(MADS_output))

Plotter.plot_domain(Functions.make_circles(MADS_input),[domain_x,domain_y],"First_Iteration")
Plotter.plot_domain(Functions.make_circles(MADS_output),[domain_x,domain_y],"Last_Iteration")

# Trajectory optimization
# ALTRO Parameters
tf = 15.0
N_t = 151

x_initial = Trajectory_Optimization.MADS_to_ALTRO(MADS_input)
x_final = Trajectory_Optimization.MADS_to_ALTRO(MADS_output)

sphere_constraint = []
collision = true
while collision
    global X, U = Trajectory_Optimization.optimize(x_initial,x_final,tf,N_t,sphere_constraint);
    global sphere_constraint, collision = Trajectory_Optimization.collide(X, sphere_constraint);
    println("Collision detected. Re-optimizing trajectory...")
end

# Plot trajectory as gif
using Plots
gr()

drones = []
for i in 1:N_drones
    push!(drones,i)
end
plt = ()
@gif for i in 1:size(X)[3]
    plot(X[drones,1,1:i]', X[drones,2,1:i]', X[drones,3,1:i]', label="", xlims=(0,domain_x), ylims=(0,domain_y), zlims=(0,15), xlabel="x (m)", ylabel="y (m)", zlabel="z (m)")
    for j in 1:length(sphere_constraint)
        plot!([sphere_constraint[j][2]], [sphere_constraint[j][3]], [sphere_constraint[j][4]], label="", seriestype=:scatter)
    end
end


fig = plotly()
plot(X[drones,1,:]', X[drones,2,:]', X[drones,3,:]', label="", xlims=(0,domain_x), ylims=(0,domain_y), zlims=(0,15), xlabel="x (m)", ylabel="y (m)", zlabel="z (m)")
for j in 1:length(sphere_constraint)
    plot!([sphere_constraint[j][2]], [sphere_constraint[j][3]], [sphere_constraint[j][4]], label="", seriestype=:scatter)
end
current()
