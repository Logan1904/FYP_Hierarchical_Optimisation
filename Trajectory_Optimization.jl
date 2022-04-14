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
function collide(x,i,j)
    
    for k in 3:size(x)[3]
        x1,y1,z1 = x[i,1:3,k]
        x2,y2,z2 = x[j,1:3,k]

        distance = sqrt((x2-x1)^2 + (y2-y1)^2 + (z2-z1)^2)

        if distance < 2.0
            return true, [x1,y1,z1], [x2,y2,z2]
            break
        end

    end

    return false, [], []

end

function optimize(mass, J, gravity, motor_dist, kf, km, x_initial, x_final, tf, Nt, collision)
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
        Q = Diagonal(@SVector fill(10., n))
        R = Diagonal(@SVector fill(10., m))
        H = @SMatrix zeros(m,n)
        q = -Q*xf
        r = @SVector zeros(m)
        c = xf'*Q*xf/2

        cost1 = QuadraticCost(Q, R, H, q, r, c)

        objective = Objective(cost1, Nt)

        # constraints
        cons = ConstraintList(n,m,Nt)

        u_min = zeros(4)
        u_max = fill(5.0,4)
        x_min = [-10.0,-10.0,0.0,-2.0,-2.0,-2.0,-2.0,-5.0,-5.0,-5.0,-5.0,-5.0,-5.0]
        x_max = [60.0,60.0,15.0,2.0,2.0,2.0,2.0,5.0,5.0,5.0,5.0,5.0,5.0]

        add_constraint!(cons, BoundConstraint(n,m, x_min=x_min, x_max=x_max), 1:Nt-1)

        if collision[i][1] == true
            x,y,z = collision[i][2]
            add_constraint!(cons, SphereConstraint(n, [x], [y], [z], [1.5]), 1:Nt)
        end

        prob = Problem(model, objective, xf, tf, x0=x0, constraints=cons)

        # initialization
        # State initialization: linear trajectory from start to end
        # Control initialization: hover
        X0 = zeros(Float64, (n,Nt))
        U0 = zeros(Float64, (m,Nt-1))

        hover = zeros(model)[2]

        for i in 1:Nt-1
            X0[:,i] = x0 + (xf-x0)*(i-1)/(Nt-1)
            U0[:,i] = hover
        end

        X0[:,Nt] = xf

        initial_states!(prob, X0)
        initial_controls!(prob, U0)
        
        altro = ALTROSolver(prob)
        set_options!(altro, max_cost_value = 1e30, penalty_initial=100)

        solve!(altro);

        global stats = altro.stats

        # Store states
        for k in 1:Nt
            X[i,:,k] = states(altro)[k]
        end
        # Store controls
        for k in 1:Nt-1
            U[i,:,k] = controls(altro)[k]
        end
    end

    return X, U, stats

end



end # module end