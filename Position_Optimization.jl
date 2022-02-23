module Position_Optimization

using DirectSearch
import Polygonal_Method
import Functions

function define_problem(z, obj, cons_ext, cons_prog, N_iter)
    # Define optimization problem
    p = DSProblem(length(z))
    SetInitialPoint(p, z)
    SetObjective(p, obj)
    SetIterationLimit(p, N_iter)

    # Add constraints to problem
    for i in cons_ext
        AddExtremeConstraint(p, i)
    end
    for i in cons_prog
        AddProgressiveConstraint(p, i)
    end

    return p
end

function optimize(p)
    # Optimize
    Optimize!(p)

    # After optimization, return feasible or infeasible solution
    if p.x === nothing
        global result = p.i
    else
        global result = p.x
    end

    return result
end

function check_local_minima(z, obj, cons_ext, cons_prog, N_iter)
    # Check: 
    # 1) Any circles completely contained by another circle
    # 2) Any circles have ONLY non-contour intersection points

    N_drones = Int(length(z)/3)

    while true
        _, circles, intersections = Polygonal_Method.Area(z, return_objects=true)

        # 'contained' vector is Bool vector for each circle, if it's contained
        contained = [i.Contained for i in circles]

        # 'underneath' vector is vector of vector of point IDs for each circle
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

        # if every point on a circle is not a contour point, it is underneath other circles
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
            # Generate new circle within its radius
            x_old = z[i]
            y_old = z[N_drones + i]
            R_old = z[N_drones*2 + i]
            x_new = rand(x_old-R_old:x_old+R_old)[1]
            y_new = rand(y_old-R_old:y_old+R_old)[1]

            z[i] = x_new
            z[N_drones + i] = y_new
        end

        # Defining new MADS problem as it performs better
        define_problem(z, obj, cons_ext, cons_prog, N_iter)
        z = optimize(p)
    end

    return z

end

end # module end