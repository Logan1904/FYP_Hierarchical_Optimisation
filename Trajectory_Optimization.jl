module Trajectory_Optimization

using TrajectoryOptimization
using RobotDynamics
using RobotZoo: Quadrotor
using StaticArrays, Rotations, LinearAlgebra
using Altro

# Function to change MADS vector structure to ALTRO vector structure
function MADS_to_ALTRO(z)
    N_drones = Int(length(z)/3)
    FOV = 90/180*pi # FOV of the sensor
    x = []
    for i in range(1,stop=N_drones)
        x_val = z[i]
        y_val = z[N_drones + i]
        R_val = z[N_drones*2 + i]

        z_val = 2*R_val/FOV

        push!(x, [x_val, y_val, z_val, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    end

    return x
end

# Collision Avoidance
function collide(X, sphere_constraint)
    for i in 1:size(X)[1]
        for j in i+1:size(X)[1]
            for k in 1:size(X)[3]
                x1,y1,z1 = X[i,1:3,k]
                x2,y2,z2 = X[j,1:3,k]

                distance = sqrt((x2-x1)^2 + (y2-y1)^2 + (z2-z1)^2)
                
                if distance < 2
                    #println("Drone ",i," and Drone ",j," collide at timestep ",k)
                    push!(sphere_constraint, [i, x2, y2, z2]) # constraint on drone 1 at location of drone 2
                    return sphere_constraint, true
                end

            end
        end
    end
    return sphere_constraint, false
end

function optimize(x_initial, x_final, tf, N_t, sphere_constraint)
    N_drones = Int(length(x_initial))

    model = Quadrotor()
    n,m = size(model)

    # Matrix of states; 1st dim = drone, 2nd dim = states, 3rd dim = time
    X = zeros(Float64, (N_drones, 13, N_t))
    # Matrix of controls; 1st dim = drone, 2nd dim = control input, 3rd dim = time
    U = zeros(Float64, (N_drones, 4, N_t-1))

    for i in 1:N_drones # for the i'th drone

        x0 = SVector{13, Float64}(x_initial[i])
        xf = SVector{13, Float64}(x_final[i])

        Q = Diagonal(@SVector fill(0.1, n))
        R = Diagonal(@SVector fill(0.01, m))
        Qf = Diagonal(@SVector fill(100.0, n))

        objective = LQRObjective(Q,R,Qf,xf,N_t)

        state_lower_bound = SVector{13, Float64}([0.0,0.0,0.0,-1,-1,-1,-1,-5,-5,-5,-5,-5,-5])
        state_upper_bound = SVector{13, Float64}([50.0,50,50,1,1,1,1,5,5,5,5,5,5])
        control_lower_bound = SVector{4, Float64}([0,0,0,0])
        control_upper_bound = SVector{4, Float64}([1.5,1.5,1.5,1.5])

        cons = ConstraintList(n,m,N_t)
        add_constraint!(cons, BoundConstraint(n,m,x_min=state_lower_bound,x_max=state_upper_bound,u_min=control_lower_bound,u_max=control_upper_bound),1:N_t-1)
        #add_constraint!(cons, GoalConstraint(xf), N_t)

        for j in 1:length(sphere_constraint)
            if sphere_constraint[j][1] == i
                xpos = sphere_constraint[j][2]
                ypos = sphere_constraint[j][3]
                zpos = sphere_constraint[j][4]
                add_constraint!(cons, SphereConstraint(n,[xpos],[ypos],[zpos],[2.1],1,2,3),1:N_t)
            end
        end

        prob = Problem(model, objective, xf, tf, x0=x0, constraints=cons)

        u0 = zeros(model)[2]
        initial_controls!(prob, u0)

        opts = SolverOptions(
            cost_tolerance_intermediate=1e-2,
            penalty_scaling=10.,
            penalty_initial=1.0
        )

        altro = ALTROSolver(prob, opts)
        solve!(altro);
        
        # Store states
        for j in 1:N_t
            X[i,:,j] = states(altro)[j]
        end
        # Store controls
        for j in 1:N_t-1
            U[i,:,j] = controls(altro)[j]
        end
    end

    return X, U

end



end # module end