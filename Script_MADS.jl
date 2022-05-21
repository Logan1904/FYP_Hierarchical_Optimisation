using Greens_Method
using Position_Optimization

# Parameters
N = 15
domain_x = 50
domain_y = 50
R_lim = 7

# MADS Parameters
N_iter = 100

# Generate circles
x_arr = Vector{Float64}(undef,0)
y_arr = Vector{Float64}(undef,0)
R_arr = Vector{Float64}(undef,0)

for i in 1:N
    x = rand(R_lim:domain_x-R_lim)[1]
    y = rand(R_lim:domain_y-R_lim)[1]
    R = rand(1:R_lim)[1]


    push!(x_arr,x)
    push!(y_arr,y)
    push!(R_arr,R)
end

MADS_input = [x_arr;y_arr;R_arr]

# Define objective
function obj(x)
    my_problem = Greens_Problem(x)

    distance = sqrt(sum((MADS_input - x).^2))
    
    return -my_problem.area + distance
end

# Define extreme and progressive constraints
include("Constraints.jl")
cons_ext = [cons1,cons2]
cons_prog = []

# Position optimization
MADS_output = Position_Optimization.optimize(MADS_input, obj, cons_ext, cons_prog, N_iter, R_lim, domain_x, domain_y)

input_problem = Greens_Problem(MADS_input)
output_problem = Greens_Problem(MADS_output)

# Plot input and output figures
using Plotter

Plot(input_problem,[domain_x,domain_y],"Input")
Plot(output_problem,[domain_x,domain_y],"Output")

# Print areas
println("-----------INPUT----------")
println("Greens Method: ",input_problem.area)
println("")
println("-----------OUTPUT----------")
println("Greens Method: ",output_problem.area)