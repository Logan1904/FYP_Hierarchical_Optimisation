using DirectSearch
import Polygonal_Method
import Plotter
import Functions

# parameters
N_circles = 10
N_Dimensions = N_circles*3
domain_x = 50
domain_y = 50
R_lim = 10

# generate circles
x_arr = Vector{Float64}(undef,0)
y_arr = Vector{Float64}(undef,0)
R_arr = Vector{Float64}(undef,0)

for i in range(1,stop=N_circles)
    x = rand(R_lim:domain_x-R_lim)[1]
    y = rand(R_lim:domain_y-R_lim)[1]
    R = rand(1:R_lim)[1]

    push!(x_arr,x)
    push!(y_arr,y)
    push!(R_arr,R)
end

z = [x_arr;y_arr;R_arr]

initial_circles = Functions.make_circles(z)

# define optimization problem
p = DSProblem(N_Dimensions)
SetInitialPoint(p, z)

# set objective function
function obj(z)
    Area = Polygonal_Method.Area(z)
    return -Area
end

SetObjective(p, obj)
SetIterationLimit(p, 100) # number of iterations for convergence depends on problem size

include("Constraints.jl") # constraints defined

# Add constraints to problem
AddExtremeConstraint(p, cons1)          # domain constraint on x,y
AddExtremeConstraint(p, cons2)          # radius constraint (R between 1 and R_lim)
AddExtremeConstraint(p, cons3)          # radius constraint (R cannot change)
#AddProgressiveConstraint(p, cons4)     # radius constraint based on x,y location

Optimize!(p)

if p.x === nothing
    output = p.i
else
    output = p.x
end

# check if
# 1) Any circles completely contained by another circle
# 2) Any circles have ONLY non-contour intersection points

while true
    _, circles, intersections = Polygonal_Method.Area(output, return_objects=true)

    contained = [i.Contained for i in circles]

    points_index = [i.Points for i in circles]
    points = [intersections[i] for i in points_index]
    underneath = []
    for i in 1:length(points)
        dum = [j.ID for j in points[i]]
        if length(dum) > 0
            push!(underneath, dum)
        else
            push!(underneath, [true])
        end
    end

    global circle_index_tochange = findall(x -> x == true, contained)

    for i in 1:length(underneath)
        if all(!,underneath[i])
            push!(circle_index_tochange, i)
        end
    end

    unique!(circle_index_tochange)

    if length(circle_index_tochange) == 0
        println("Final configuration ok")
        break
    end

    println("Some circles contained by others. Re-generating circle and re-optimizing.")

    for i in circle_index_tochange
        x = rand(R_lim:domain_x-R_lim)[1]
        y = rand(R_lim:domain_y-R_lim)[1]

        output[i] = x
        output[N_circles + i] = y
    end

    # Defining new MADS problem as it performs better
    global p = DSProblem(N_Dimensions)
    SetInitialPoint(p, output)
    SetObjective(p, obj)
    SetIterationLimit(p, 100)

    AddExtremeConstraint(p, cons1)
    AddExtremeConstraint(p, cons2)
    AddExtremeConstraint(p, cons3)
    #AddProgressiveConstraint(p, cons4)

    Optimize!(p);

    if p.x === nothing
        global output = p.i
    else
        global output = p.x
    end

end

# print areas and plot images
println("First iteration: ",Polygonal_Method.Area(z))
println("After optimization: ", Polygonal_Method.Area(output))

Plotter.plot_domain(initial_circles,[domain_x,domain_y],"First_Iteration")
Plotter.plot_domain(Functions.make_circles(output),[domain_x,domain_y],"Last_Iteration")