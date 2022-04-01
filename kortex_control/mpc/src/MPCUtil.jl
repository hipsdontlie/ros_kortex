import Pkg; Pkg.status()
include("../src/Arthur.jl")
using RigidBodyDynamics
using StaticArrays
using RobotDynamics
using Altro
using TrajectoryOptimization

# function mpc_update(altro, prob_mpc, Z_track, t0, k_mpc)
#     TrajectoryOptimization.set_initial_time!(prob_mpc, t0)

#     # Propagate the system forward w/ noise
#     # x0 = discrete_dynamics(TrajectoryOptimization.integration(prob_mpc),
#     #                             prob_mpc.model, prob_mpc.Z[1])      
#     x0 = rk4(prob_mpc.model, state(prob_mpc.Z[1]), control(prob_mpc.Z[1]), prob_mpc.Z[1].dt)

#     TrajectoryOptimization.set_initial_state!(prob_mpc, x0)

#     # Update tracking cost
#     TrajectoryOptimization.update_trajectory!(prob_mpc.obj, Z_track, k_mpc)

#     # Shift the initial trajectory
#     RobotDynamics.shift_fill!(prob_mpc.Z)

#     # Shift the multipliers and penalties
#     Altro.shift_fill!(TrajectoryOptimization.get_constraints(altro.solver_al))
# end

function update_trajectory_fill!(obj::Objective{<:TrajectoryOptimization.QuadraticCostFunction}, Z::TrajectoryOptimization.AbstractTrajectory, start=1)
    inds = (start-1) .+ (1:length(obj))
    if (start + length(obj) - 1) <= length(Z)
        for (i,k) in enumerate(inds)
            TrajectoryOptimization.set_LQR_goal!(obj[i], state(Z[k]), control(Z[k]))
        end
    else
        for (i,k) in enumerate(inds)
            TrajectoryOptimization.set_LQR_goal!(obj[i], state(Z[min(k,length(Z))]), control(Z[min(k,length(Z))]))
        end
    end
end

function mpc_update(altro, prob_mpc, Z_track, x0, t0)
    TrajectoryOptimization.set_initial_time!(prob_mpc, t0)

    # Propagate the system forward w/ noise
    # x0 = discrete_dynamics(TrajectoryOptimization.integration(prob_mpc),
    #                             prob_mpc.model, prob_mpc.Z[1])      
    # x0 = rk4(prob_mpc.model, state(prob_mpc.Z[1]), control(prob_mpc.Z[1]), prob_mpc.Z[1].dt)
    k_mpc = argmin(norm.([(states(Z_track)[k] - x0) for k=1:length(Z_track)]))

    TrajectoryOptimization.set_initial_state!(prob_mpc, x0)

    # Update tracking cost
    update_trajectory_fill!(prob_mpc.obj, Z_track, k_mpc)
    
    # Shift the initial trajectory
    RobotDynamics.shift_fill!(prob_mpc.Z)
    
    # Shift the multipliers and penalties
    Altro.shift_fill!(TrajectoryOptimization.get_constraints(altro.solver_al))
end

function ArthurProblem(Xref; params=MPC_Params())
    # Create initial guess of U trajectory
    Qref = Vector{Float64}[]
    Uref = Vector{Float64}[]
    N = length(Xref)
    n = params.n
    m = params.m
    tf = round(params.dt*(N-1), digits = 3)
    push!(Qref, copy(Xref[1][1:14]))
    for k = 1:N-1
        set_configuration!(params.state, Qref[k][1:7])
        set_velocity!(params.state, Qref[k][8:end])
        push!(Uref, inverse_dynamics(params.state, params.v̇))
        push!(Qref, Qref[k] + dynamics(params.model, Qref[k], Uref[k])*params.dt)
    end

    # Gather trajectory information
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
    add_constraint!(conSet, state_bnd, 1:N-1)

    # # Cartesian Velocity Bound
    # ẋ_max = 0.0005 # m/s
    # vel_bnd = NormConstraint(n, m, ẋ_max, Inequality(), 21:23)
    # add_constraint!(conSet, vel_bnd, 1:N)

    # # Force Bound (Fx Fy Fz)
    # F_max = 20 # Newtons
    # F_bnd = NormConstraint(n, m, F_max, Inequality(), 27:29)
    # add_constraint!(conSet, F_bnd, 1:N)

    obj = TrajectoryOptimization.LQRObjective(params.Q, params.R, params.Qf, Xref[end], N)
    prob = Problem(params.model, obj, Xref[end], tf, x0=Xref[1], constraints=conSet, X0=Xref, U0=Uref, integration=RK4)
    return prob
end

function ArthurHorizonProblem(prob::TrajectoryOptimization.Problem, N; params=MPC_Params())
    n,m = size(prob)
    dt = prob.Z[1].dt
    tf = (N-1)*dt

    # Get sub-trajectory
    Z = Traj(prob.Z[1:N])

    # Generate a cost that tracks the trajectory
    obj = TrajectoryOptimization.TrackingObjective(params.Q, params.R, Z, Qf=params.Qf)

    # Use the same constraints, except the Goal constraint
    cons = ConstraintList(n,m,N)
    for (inds, con) in zip(prob.constraints)
        if !(con isa GoalConstraint)
            if inds.stop > N
                inds = inds.start:N-(prob.N - inds.stop)
            end
            length(inds) > 0 && TrajectoryOptimization.add_constraint!(cons, con, inds)
        end
    end

    prob = TrajectoryOptimization.Problem(prob.model, obj, state(Z[N]), tf, x0=state(Z[1]), constraints=cons,
        integration=TrajectoryOptimization.integration(prob)
    )
    initial_trajectory!(prob, Z)
    return prob
end

struct MPC_Params
    model::Arthur
    n::Int64
    m::Int64
    H::Int64
    tf::Float64
    dt::Float64
    state::MechanismState
    v̇::SegmentedVector{JointID, Float64, Base.OneTo{JointID}, Vector{Float64}}
    opts::SolverOptions{Float64}
    Q::Diagonal{Float64, SVector{14, Float64}}
    Qf::Diagonal{Float64, SVector{14, Float64}}
    R::Diagonal{Float64, SVector{7, Float64}}

    function MPC_Params()
        model = Arthur()
        n,m = size(model)

        tf = 0.2 # Time Horizon (seconds)
        dt = 0.1 # Time step (seconds)
        H = Int(round(tf/dt) + 1) # Time Horizon (discrete steps)

        state = MechanismState(model.mechanism)
        zero!(state)
        v̇ = similar(velocity(state))
        for joint in joints(state.mechanism)
            if joint == joints(state.mechanism)[2] || joint == joints(state.mechanism)[3] || joint == joints(state.mechanism)[4] || joint == joints(state.mechanism)[5] || joint == joints(state.mechanism)[6] || joint == joints(state.mechanism)[7] || joint == joints(state.mechanism)[8]
                v̇[joint][1] = 0.0
            end
        end

        opts = SolverOptions(
            cost_tolerance_intermediate=1e-2,
            penalty_scaling=10.,
            penalty_initial=1.0,
            show_summary=true,
            projected_newton=true,
        )

        Q = 100.0*Diagonal(@SVector ones(n))
        Qf = 100*Diagonal(@SVector ones(n))
        R = 1.0e-1*Diagonal(@SVector ones(m))
        new(model,n,m,H,tf,dt,state,v̇,opts,Q,Qf,R)
    end
end


# function initialize_solver(params::MPC_Params)
#     traj = RobotDynamics.Traj(params.n, params.m, params.dt, params.H)
#     obj = TrajectoryOptimization.TrackingObjective(params.Q, params.R, traj, Qf=params.Qf)
#     prob = Problem(params.model, obj, Xref[end], tf, x0=Xref[1], constraints=params.conSet, X0=Xref, U0=Uref)
#     altro = ALTROSolver(prob, params.opts)
# end