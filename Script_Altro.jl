import Trajectory_Optimization
using StaticArrays, Rotations, LinearAlgebra

# Drone Parameters
mass = 0.5                                       # mass of quadrotor
J = Diagonal(@SVector[0.0023, 0.0023, 0.004])    # inertia matrix
gravity = SVector(0,0,-3.721)                    # gravity vector
motor_dist = 0.1750                              # distance between motors
kf = 1.0                                         # motor force constant (motor force = kf*u)
km = 0.0245                                      # motor torque constant (motor torque = km*u)

# Obtain initial and final states for all drones
x_initial = Trajectory_Optimization.MADS_to_ALTRO(MADS_input)
x_final = Trajectory_Optimization.MADS_to_ALTRO(MADS_output)

# Initialise state and control history matrices
X = zeros(Float64, (N_drones, 13, 1))

for i in 1:N_drones
    X[i,:,1] = x_initial[i]
end

U = zeros(Float64, (N_drones, 4, 0))

# MPC optimization
tf = 15.0           # Total time
hor = 1.0           # Prediction horizon length
dt = 0.05           # Time-step length per horizon
Nt = Int(hor/dt)+1  # Number of timesteps per horizon

time = 0.0
while time < tf

    x,u = Trajectory_Optimization.optimize(mass, J, gravity, motor_dist, kf, km, x_initial, x_final, hor, Nt)
    
    # update x_initial to first timestep
    for i in 1:N_drones
        global x_initial[i] = x[i,:,2]
    end

    # store state and control
    global X = cat(X,x[:,:,2],dims=3)
    global U = cat(U,u[:,:,2],dims=3)

    global time += 1*dt
end


# Plot trajectory as gif
using Plots
drones = []
for i in 1:N_drones
    push!(drones,i)
end

fig = plotly()
plot(X[drones,1,:]', X[drones,2,:]', X[drones,3,:]', label="", xlims=(0,domain_x), ylims=(0,domain_y), zlims=(0,15), xlabel="x (m)", ylabel="y (m)", zlabel="z (m)")
current()

gr()
plt = ()
@gif for i in 1:size(X)[3]
    plot(X[drones,1,1:i]', X[drones,2,1:i]', X[drones,3,1:i]', label="", xlims=(0,domain_x), ylims=(0,domain_y), zlims=(0,15), xlabel="x (m)", ylabel="y (m)", zlabel="z (m)")
end