import Pkg; Pkg.status()
include("../src/Arthur.jl")
using RigidBodyDynamics
using StaticArrays
using RobotDynamics
using Altro
using TrajectoryOptimization

struct MPC_Params
    model::Arthur
    n::Int64
    m::Int64
    N::Int64
    tf::Float64
    dt::Float64
    state::MechanismState
    v̇::SegmentedVector{JointID, Float64, Base.OneTo{JointID}, Vector{Float64}}
    conSet::ConstraintList
    opts::SolverOptions{Float64}
    Q::Diagonal{Float64, SVector{14, Float64}}
    Qf::Diagonal{Float64, SVector{14, Float64}}
    R::Diagonal{Float64, SVector{7, Float64}}

    function MPC_Params()
        model = Arthur()
        n,m = size(model)

        tf = 3.0 # Time Horizon (seconds)
        dt = 0.05 # Time step (seconds)
        N = Int(round(tf/dt) + 1) # Time Horizon (discrete steps)

        state = MechanismState(model.mechanism)
        zero!(state)
        v̇ = similar(velocity(state))
        for joint in joints(state.mechanism)
            if joint == joints(state.mechanism)[2] || joint == joints(state.mechanism)[3] || joint == joints(state.mechanism)[4] || joint == joints(state.mechanism)[5] || joint == joints(state.mechanism)[6] || joint == joints(state.mechanism)[7] || joint == joints(state.mechanism)[8]
                v̇[joint][1] = 0.0
            end
        end
        
        # Create Empty ConstraintList
        conSet = ConstraintList(n,m,N)

        # Control Bounds based on Robot Specs (Joint torque limits)
        u_bnd = [39.0, 39.0, 39.0, 39.0, 9.0, 9.0, 9.0]
        control_bnd = BoundConstraint(n,m, u_min=-u_bnd, u_max=u_bnd)
        add_constraint!(conSet, control_bnd, 1:N-1)

        # State Bounds based on Robot Specs (Joint velocity and speed limits)
        x_bnd = zeros(n)
        x_bnd[1:7] = [Inf, deg2rad(128.9), Inf, deg2rad(147.8), Inf, deg2rad(120.3), Inf] # rad
        x_bnd[8:14] = [1.39, 1.39, 1.39, 1.39, 1.22, 1.22, 1.22] # rad/sec
        # x_bnd[8:14] = [0.25, 0.05, 0.05, 0.05, 0.25, 0.15, 0.15] # rad/sec
        # x_bnd[15:end] = [Inf for k=1:(n-14)] # Constraints on force elsewhere
        state_bnd = BoundConstraint(n,m, x_min=-x_bnd, x_max=x_bnd)
        add_constraint!(conSet, state_bnd, 1:N)

        # # Cartesian Velocity Bound
        # ẋ_max = 0.0005 # m/s
        # vel_bnd = NormConstraint(n, m, ẋ_max, Inequality(), 21:23)
        # add_constraint!(conSet, vel_bnd, 1:N)

        # # Force Bound (Fx Fy Fz)
        # F_max = 20 # Newtons
        # F_bnd = NormConstraint(n, m, F_max, Inequality(), 27:29)
        # add_constraint!(conSet, F_bnd, 1:N)

        opts = SolverOptions(
            cost_tolerance_intermediate=1e-2,
            penalty_scaling=10.,
            penalty_initial=1.0,
        )

        Q = 100.0*Diagonal(@SVector ones(n))
        Qf = 100*Diagonal(@SVector ones(n))
        R = 1.0e-1*Diagonal(@SVector ones(m))
        new(model,n,m,N,tf,dt,state,v̇,conSet,opts,Q,Qf,R)
    end
end


function initialize_solver(params::MPC_Params)
    traj = RobotDynamics.Traj(params.n, params.m, params.dt, params.N)
    obj = TrajectoryOptimization.TrackingObjective(params.Q, params.R, traj, Qf=params.Qf)
    prob = Problem(params.model, obj, Xref[end], tf, x0=Xref[1], constraints=params.conSet, X0=Xref, U0=Uref)
    altro = ALTROSolver(prob, params.opts)
end