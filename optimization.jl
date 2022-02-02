using DirectSearch
import Polygonal_Method

# parameters
N_circles = 10
domain_x = 100
domain_y = 100
radius_limit = 20

#generate circles
x_arr = Vector{Float64}(undef,0)
y_arr = Vector{Float64}(undef,0)
R_arr = Vector{Float64}(undef,0)

for i in range(1,stop=N_circles)
    x = rand(radius_limit:domain_x-radius_limit)[1]
    y = rand(radius_limit:domain_y-radius_limit)[1]
    R = rand(1:radius_limit)[1]

    push!(x_arr,x)
    push!(y_arr,y)
    push!(R_arr,R)
end

arr = [x_arr;y_arr;R_arr]
circles = Polygonal_Method.make_circles(arr)

z = [x_arr;y_arr]

N_Dimensions = N_circles*2

# Objective function
function obj(z)
    obj_arr = [z;R_arr]
    Area = -1 * Polygonal_Method.Area(obj_arr)
    return Area
end

p = DSProblem(N_Dimensions, search=RandomSearch(10), poll=LTMADS());
SetInitialPoint(p, z);
SetObjective(p, obj);
SetIterationLimit(p, 100); #number of iterations depends on problem size to converge

function cons(x)
    for i in range(1,stop=length(x))
        val = x[i] <= domain_x-radius_limit && x[i] >= radius_limit; #change constraint so independant of x and y
        if !val
            return false
        end
    end

    return true
end

AddExtremeConstraint(p, cons);

Optimize!(p)

println("First iteration: ",Polygonal_Method.Area(arr,print=false))
println("After optimization: ", Polygonal_Method.Area([p.x;R_arr],print=false))

import Plotter
Plotter.plot_domain(circles,[domain_x,domain_y],"First_Iteration")
Plotter.plot_domain(Polygonal_Method.make_circles([p.x;R_arr]),[domain_x,domain_y],"Last_Iteration")