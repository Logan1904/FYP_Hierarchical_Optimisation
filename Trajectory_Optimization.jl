module Trajectory_Optimization

using TrajectoryOptimization
using RobotDynamics
using RobotZoo: Quadrotor
using StaticArrays, Rotations, LinearAlgebra
using Altro

# Function to change MADS vector structure to ALTRO vector structure
function MADS_to_ALTRO(z)
    N_drones = Int(length(z)/3)
    FOV = 90/180*pi    
                 # FOV of the sensor
    x = []
    for i in range(1,stop=N_drones)
        x_val = z[i]
        y_val = z[N_drones + i]
        R_val = z[N_drones*2 + i]

        z_val = 2*R_val/FOV

        push!(x, RBState([x_val, y_val, z_val], UnitQuaternion(I), zeros(3), zeros(3)));
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

function optimize(mass, J, gravity, motor_dist, kf, km, x_initial, x_final, tf, Nt, u0)
    N_drones = length(x_initial)

    model = Quadrotor(mass=mass, J=J, gravity=gravity, motor_dist=motor_dist, kf=kf, km=km)
    n,m = size(model)

    # Matrix of states; 1st dim = drone, 2nd dim = states, 3rd dim = time
    X = zeros(Float64, (N_drones, 13, Nt))
    # Matrix of controls; 1st dim = drone, 2nd dim = control input, 3rd dim = time
    U = zeros(Float64, (N_drones, 4, Nt-1))

    for i in 1:N_drones # for the i'th drone
        # initial and final conditions
        x0 = SVector{13, Float64}(x_initial[i])
        xf = SVector{13, Float64}(x_final[i])

        # objective
        Q = Diagonal(@SVector fill(0.1, n))
        R = Diagonal(@SVector fill(0.1, m))
        Qf = Diagonal(@SVector fill(1000.0, n))
        objective = LQRObjective(Q,R,Qf,xf,Nt)

        # constraints
        cons = ConstraintList(n,m,Nt)

        u_min = zeros(4)
        u_max = fill(5.0,4)
        x_min = [-10.0,-10.0,0.0,-1.0,-1.0,-1.0,-1.0,-5.0,-5.0,-5.0,-5.0,-5.0,-5.0]
        x_max = [60.0,60.0,15.0,1.0,1.0,1.0,1.0,5.0,5.0,5.0,5.0,5.0,5.0]

        add_constraint!(cons, BoundConstraint(n,m, x_min=x_min, x_max=x_max), 1:Nt-1)
        #add_constraint!(cons, NormConstraint(n,m,5,Equality(),:control), 1:Nt-1)

        # problem
        prob = Problem(model, objective, xf, tf, x0=x0, constraints=cons)
        
        initial_controls!(prob, u0)
        initial_states!(prob,)


        altro = ALTROSolver(prob)
        solve!(altro);

        # Store states
        for k in 1:Nt
            X[i,:,k] = states(altro)[k]
        end
        # Store controls
        for k in 1:Nt-1
            U[i,:,k] = controls(altro)[k]
        end
    end

    return X, U

end



end # module end