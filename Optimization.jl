using DirectSearch
import Polygonal_Method
import Plotter

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
    x = rand(31.0:domain_x-R_lim)[1]
    y = rand(31.0:domain_y-R_lim)[1]
    R = rand(1:R_lim)[1]

    push!(x_arr,x)
    push!(y_arr,y)
    push!(R_arr,R)
end

z = [x_arr;y_arr;R_arr]

initial_circles = Polygonal_Method.make_circles(z)

# define optimization problem
p = DSProblem(N_Dimensions)
SetInitialPoint(p, z)

# set objective function
function obj(z)
    try
        global Area = Polygonal_Method.Area(z)
    catch e
        println("Error when evaluating area")
        global Area = 0.0
    end
    return -Area
end

SetObjective(p, obj)
SetIterationLimit(p, 100) #number of iterations for convergence depends on problem size

# x domain constraint
function cons1(x)
    for i in range(1,stop=Int(length(x)/3))
        x_val = x[i]
        R_val = x[N_circles*2 + i]
        val = x_val <= domain_x-R_val && x_val >= R_val #change constraint so independant of x and y
        if !val
            return false
        end
    end

    return true
end

# y domain constraint
function cons2(x)
    for i in range(1,stop=Int(length(x)/3))
        y_val = x[N_circles + i]
        R_val = x[N_circles*2 + i]
        val = y_val <= domain_y-R_val && y_val >= R_val #change constraint so independant of x and y
        if !val
            return false
        end
    end

    return true
end

# R domain constraint
function cons3(x)
    for i in range(1,stop=Int(length(x)/3))
        R_val = x[N_circles*2 + i]
        val = R_val >= 1 && R_val <= R_lim
        if !val
            return false
        end
    end

    return true
end

function cons4(x)
    for i in range(1,stop=Int(length(x)/3))
        R_val = x[N_circles*2 + i]
        val = R_val == z[N_circles*2 + i]
        if !val
            return false
        end
    end

    return true
end

function cons5(x)
    return sum(x[2*N_circles+1 : N_Dimensions]) - 50
end

function cons6(x)
    for i in range(1,stop=Int(length(x)/3))
        x_val = x[i]
        y_val = x[N_circles + i]
        R_val = x[N_circles*2 + i]

        if x_val <= 30
            if R_val > 5
                return false
            end
        end
    end
    return true
end

# add constraints to problem
AddExtremeConstraint(p, cons1)          # domain constraint on x
AddExtremeConstraint(p, cons2)          # domain constraint on y
AddExtremeConstraint(p, cons3)         # radius constraint (R between 1 and R_lim)
#AddExtremeConstraint(p, cons4)          # radius constraint (R cannot change)
#AddProgressiveConstraint(p, cons5)     # radius constraint (Sum of radii < someval)
AddExtremeConstraint(p, cons6)      # radius constraint based on x,y location

Optimize!(p)

if p.x === nothing
    output = p.i
else
    output = p.x
end

# check if any circle completely contained by another circle, ie stuck in local minima
_,final_circles = Polygonal_Method.Area(output, return_objects=true)
global contained = [final_circles[i].Contained for i in 1:length(final_circles)]

while any(contained)
    println("Some circles contained in others. Replacing with random circles")
    index_contained = findall(x -> x == true, contained)
    for i in index_contained
        x = rand(R_lim:domain_x-R_lim)[1]
        y = rand(R_lim:domain_y-R_lim)[1]

        output[i] = x
        output[N_circles + i] = y
    end

    global _,final_circles = Polygonal_Method.Area(output, return_objects=true)
    global contained = [final_circles[i].Contained for i in 1:length(final_circles)]

    if any(contained) == false
        BumpIterationLimit(p)
        Optimize!(p)
    end
end

if p.x === nothing
    output = p.i
else
    output = p.x
end

# print areas and plot images
println("First iteration: ",Polygonal_Method.Area(z))
println("After optimization: ", Polygonal_Method.Area(output))

Plotter.plot_domain(initial_circles,[domain_x,domain_y],"First_Iteration")
Plotter.plot_domain(Polygonal_Method.make_circles(output),[domain_x,domain_y],"Last_Iteration")

area, circles, intersections, polygons, sectors = Polygonal_Method.Area(output, return_objects = true)
Plotter.plot_all(circles, intersections, polygons, sectors, "tmp")