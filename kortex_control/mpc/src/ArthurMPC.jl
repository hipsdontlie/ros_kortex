include("Arthur.jl")
using RigidBodyDynamics
using StaticArrays
using RobotDynamics
using Rotations
using LinearAlgebra
using ForwardDiff, FiniteDiff
using Altro
using TrajectoryOptimization

model = Arthur()
n,m = size(model)

N = 61
tf = 3.
dt = tf/(N-1)

x0 = @SVector zeros(n)
xf = @SVector zeros(n);  # i.e. swing up
#Traj = # Input from trajectory planner

# Set up
Q = 1.0e-2*Diagonal(@SVector ones(n))
Qf = 100.0*Diagonal(@SVector ones(n))
R = 1.0e-1*Diagonal(@SVector ones(m))
obj = TrackingObjective(Q, R, Traj, Qf=Qf)
# obj = LQRObjective(Q,R,Qf,xf,N);

# Create Empty ConstraintList
conSet = ConstraintList(n,m,N)

# Control Bounds based on Robot Specs (Joint torque limits)
u_bnd = [39.0, 39.0, 39.0, 39.0, 9.0, 9.0, 9.0]
control_bnd = BoundConstraint(n,m, u_min=-u_bnd, u_max=u_bnd)
add_constraint!(conSet, control_bnd, 1:N-1)

# State Bounds based on Robot Specs (Joint velocity and speed limits)
x_bnd = zeros(26)
x_bnd[1:7] = [Inf, deg2rad(128.9), Inf, deg2rad(147.8), Inf, deg2rad(120.3), Inf] # rad
x_bnd[8:14] = [1.39, 1.39, 1.39, 1.39, 1.22, 1.22, 1.22] # rad/sec
x_bnd[15:end] = [Inf, Inf, Inf, Inf, Inf, Inf, Inf, Inf, Inf, Inf, Inf, Inf] # Constraints on force elsewhere
state_bnd = BoundConstraint(n,m, x_min=-x_bnd, x_max=x_bnd)
add_constraint!(conSet, state_bnd, 1:N)

# Cartesian Velocity Bound
ẋ_max = 0.0005 # m/s
vel_bnd = NormConstraint(n, m, ẋ_max, Inequality(), 15:20)
add_constraint!(conSet, vel_bnd, 1:N)

# Force Bound
F_max = 20 # Newtons
F_bnd = NormConstraint(n, m, F_max, Inequality(), 21:26)
add_constraint!(conSet, F_bnd, 1:N)

# Goal Constraint - only if you want the final state to be the desired state
# goal = GoalConstraint(xf)
# add_constraint!(conSet, goal, N)


