#using Trajectory_Optimization
using StaticArrays, Rotations, LinearAlgebra
using RobotZoo: Quadrotor
using RobotDynamics

# Drone Parameters
mass = 0.4                                       # mass of quadrotor
J = Diagonal(@SVector[0.0023, 0.0023, 0.004])    # inertia matrix
gravity = SVector(0,0,-3.721)                    # gravity vector
motor_dist = 0.1750                              # distance between motors
kf = 1.0                                         # motor force constant (motor force = kf*u)
km = 0.0245                                      # motor torque constant (motor torque = km*u)

# Obtain initial and final states for all drones
x_start = Trajectory_Optimization.MADS_to_ALTRO(MADS_input)
x_final = Trajectory_Optimization.MADS_to_ALTRO(MADS_output)

MAVs = Vector{Trajectory_Optimization.Trajectory_Problem}()
for i in 1:N
    push!(MAVs,Trajectory_Optimization.Trajectory_Problem(mass,J,gravity,motor_dist,kf,km,x_start[i],x_final[i]))
end

# MPC optimization
hor = 3.0           # Prediction horizon length
dt = 0.1            # Time-step length per horizon
Nt = Int(hor/dt)+1  # Number of timesteps per horizon
R_D = 10.0           # Danger radius
R_C = 1.0           # Collision radius
Nm = 5              # Number of applied time-steps


total_converge = false # Check if all MAVs have converged
collision = Vector{Any}(undef,N) # Vector describing collision constraints
for i in 1:N
    collision[i] = [false,[]]
end

# Initialise vector for solution times
my_time = Vector{Vector{Any}}(undef,N)
for i in 1:N
    my_time[i] = []
end

# Optimize
while total_converge == false
    global total_converge = true
    
    # Optimize if MAV has not converged to final position
    for i in 1:N
        MAV = MAVs[i]
        if Trajectory_Optimization.converge(MAV) > 0.1
            global total_converge = false
            t = Trajectory_Optimization.optimize(MAV,hor,Nt,Nm,collision[i])
            push!(my_time[i],t)
        end
    end

    # Check for collision
    for i in 1:N
        global collision[i] = [false,[]]
    end
    for i in 1:N
        for j in i+1:N
            MAV1 = MAVs[i]
            MAV2 = MAVs[j]
            distance = sqrt(sum((MAV1.StateHistory[end][1:3] - MAV2.StateHistory[end][1:3]).^2))
            if distance <= R_D
                a,b,c = Trajectory_Optimization.collide(MAV1,MAV2,R_C,Nt)
                collision[i] = [a,c]
                collision[j] = [a,b]
            end
        end
    end
end

# Normalise state histories
longest = maximum([length(MAVs[i].StateHistory) for i in 1:N])
for i in 1:N
    MAV = MAVs[i]
    size = length(MAV.StateHistory)
    if size < longest
        for j in 1:longest-size
            push!(MAV.StateHistory,MAV.StateHistory[end])
        end
    end
end

# Extract trajectories
X = []
for i in 1:N
    MAV = MAVs[i]
    x = zeros(Float64, (length(MAV.StateHistory),13))
    for j in 1:length(MAV.StateHistory)
        x[j,:] = MAV.StateHistory[j]
    end
    push!(X,x)
end

# Plot trajectories
using Plots

# Static 3D plot
p = plot(legend=:outertopright,minorgrid=true, minorgridalpha=0.25)
for i in 1:N
    plot!(X[i][:,1],X[i][:,2],X[i][:,3],  markershape=:none, label="MAV $i", xlims=(0,50), ylims=(0,50), zlims=(0,15), xlabel="x (m)", ylabel="y (m)", zlabel="z (m)")
end

# Animation plot
anim = @animate for i in 1:longest
    for j in 1:N
        if j == 1
            plot(X[j][1:i,1], X[j][1:i,2],X[j][1:i,3], label="", xlims=(0,50), ylims=(0,50), zlims=(0,15), xlabel="x (m)", ylabel="y (m)", zlabel="z (m)")
        else
            plot!(X[j][1:i,1], X[j][1:i,2],X[j][1:i,3], label="", xlims=(0,50), ylims=(0,50), zlims=(0,15), xlabel="x (m)", ylabel="y (m)", zlabel="z (m)")
        end
    end
end

gif(anim,fps=10)