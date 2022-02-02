using DirectSearch
import Polygonal_Method
import Functions

N_circles = 5
domain_x = 20
domain_y = 20
radius_limit = 5
Radius = [1,2,3,4,5]

circles = []
z = Vector{Float64}(undef, 0)
for i in range(1,stop=N_circles)
    x = rand(radius_limit:domain_x-radius_limit)[1]
    y = rand(radius_limit:domain_y-radius_limit)[1]
    R = Radius[i]

    push!(circles,Functions.Circle(x,y,R,[],[],false))
    push!(z,x,y)
end

N_Dimensions = N_circles*2

# Initial transformation from circles to z and radius
function obj(z)
    temp = []
    for i in range(1,stop=size(z)[1],step=2)
        x = z[i]
        y = z[i+1]

        push!(temp,Functions.Circle(x,y,1,[],[],false))
    end

    for i in range(1,stop=length(circles))
        temp[i].R = Radius[i]
    end

    Area = -Polygonal_Method.Area(temp)

    return Area
end

#transform from z back to circle vector
function transform(z)
    temp = []
    for i in range(1,stop=length(z),step=2)
        x = z[i]
        y = z[i+1]

        push!(temp,Functions.Circle(x,y,1,[],[],false))
    end

    for i in range(1,stop=length(circles))
        temp[i].R = Radius[i]
    end

    return temp
end

p = DSProblem(N_Dimensions, search=RandomSearch(10), poll=LTMADS());
SetInitialPoint(p, z);
SetObjective(p, obj);
SetIterationLimit(p, 100);

function cons(x)
    for i in range(1,stop=length(x))
        val = x[i] <= 15 && x[i] >= 5;
        if !val
            return false
        end
    end

    return true
end

AddExtremeConstraint(p, cons);

Optimize!(p)

println("First iteration: ",Polygonal_Method.Area(circles,print=false))
println("After optimization: ", Polygonal_Method.Area(transform(p.x),print=false))

import Plotter
Plotter.plot_domain(circles,[domain_x,domain_y],"First_Iteration")
Plotter.plot_domain(transform(p.x),[domain_x,domain_y],"Last_Iteration")