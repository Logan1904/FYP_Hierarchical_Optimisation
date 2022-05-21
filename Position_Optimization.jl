module Position_Optimization

using DirectSearch
using Greens_Method
using Base_Functions

"""
    optimize(input, obj, cons_ext, cons_prog, N_iter, R_lim, domain_x, domain_y)

Optimize the problem using  the Mesh Adaptive Direct Search solver

# Arguments:

    - 'input': Initial point
    - 'obj': Objective function
    - 'cons_ext': Extreme constraints
    - 'cons_prog': Progressive constraints
    - 'N_iter': Number of iterations
    - 'R_lim': Radius limit of circles
    - 'domain_x': x domain size
    - 'domain_y': y domain size
"""
function optimize(input, obj, cons_ext, cons_prog, N_iter, R_lim, domain_x, domain_y)

    # Define optimization problem
    global p = DSProblem(length(input))
    SetInitialPoint(p, input)
    SetObjective(p, obj)
    SetIterationLimit(p, N_iter)

    # Add constraints to problem
    for i in cons_ext
        AddExtremeConstraint(p, i)
    end
    for i in cons_prog
        AddProgressiveConstraint(p, i)
    end

    iter = 0
    while true
        Optimize!(p)

        # After optimization, return feasible or infeasible solution
        if p.x === nothing
            global result = p.i
        else
            global result = p.x
        end
        
        # check if any circles are contained
        var,new_input = check(input,result, R_lim, domain_x, domain_y)

        if var
            SetInitialPoint(p, new_input)
            BumpIterationLimit(p, i=N_iter)

            println("Non-optimal solution (a circle is contained). REINITIALIZING...")
            iter += 1
        else
            break
        end

        if iter > 10 # prevent infinite recusion if solution can't be found in 10 reinitializations
            break
        end
    end

    return result

end

"""
    check(input,output,R_lim,domain_x,domain_y)

Checks if any Circle objects are contained by other circles, and regenerates them (randomly)
"""
function check(input,output,R_lim,domain_x,domain_y)
    output_circles = make_circles(output)

    N = length(output_circles)

    new_input = copy(input)

    is_contained = Vector{Bool}([false for i in 1:length(output_circles)])

    for i in 1:length(output_circles)
        for j in i+1:length(output_circles)
            if contained(output_circles[i],output_circles[j]) == output_circles[i]
                is_contained[i] = true
            elseif contained(output_circles[i],output_circles[j]) == output_circles[j]
                is_contained[j] = true
            end
        end
    end

    if any(is_contained)

        for i in 1:length(output_circles)
            # if circle is contained, regenerate it (randomly)
            if is_contained[i] == true
                new_input[i] = rand(R_lim:domain_x-R_lim)[1]
                new_input[N + i] = rand(R_lim:domain_y-R_lim)[1]
                new_input[2*N + i] = rand(1:R_lim)[1]
            end
        end

        return true, new_input
    else
        return false, []
    end
end

end # module end