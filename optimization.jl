using DirectSearch
import Polygonal_Method

# parameters
N_circles = 10
N_Dimensions = N_circles*2
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

arr = [x_arr;y_arr;R_arr]
z = [x_arr;y_arr]

initial_circles = Polygonal_Method.make_circles(arr)

# define optimization problem
p = DSProblem(N_Dimensions, search=RandomSearch(10), poll=LTMADS())
SetInitialPoint(p, z)

# set objective function
function obj(z)
    obj_arr = [z;R_arr]
    Area = -1 * Polygonal_Method.Area(obj_arr)
    return Area
end

SetObjective(p, obj)
SetIterationLimit(p, 100) #number of iterations for convergence depends on problem size

function cons(x)
    for i in range(1,stop=length(x))
        val = x[i] <= domain_x-R_lim && x[i] >= R_lim #change constraint so independant of x and y
        if !val
            return false
        end
    end

    return true
end

AddExtremeConstraint(p, cons)
Optimize!(p)

# check if any circle completely contained by another circle, ie stuck in local minima
_,final_circles = Polygonal_Method.Area([p.x;R_arr], return_objects=true)
global contained = [final_circles[i].Contained for i in 1:length(final_circles)]

while any(contained)
    println("Some circles contained in others. Replacing with random circles")
    index_contained = findall(x -> x == true, contained)
    for i in index_contained
        x = rand(R_lim:domain_x-R_lim)[1]
        y = rand(R_lim:domain_y-R_lim)[1]

        p.x[i] = x
        p.x[N_circles + i] = y
    end

    global _,final_circles = Polygonal_Method.Area([p.x;R_arr], return_objects=true)
    global contained = [final_circles[i].Contained for i in 1:length(final_circles)]

    BumpIterationLimit(p)
    Optimize!(p)

end

println("First iteration: ",Polygonal_Method.Area(arr))
println("After optimization: ", Polygonal_Method.Area([p.x;R_arr]))

import Plotter
Plotter.plot_domain(initial_circles,[domain_x,domain_y],"First_Iteration")
Plotter.plot_domain(Polygonal_Method.make_circles([p.x;R_arr]),[domain_x,domain_y],"Last_Iteration")