using RigidBodyDynamics
using StaticArrays
using Parameters
using RobotDynamics
using Rotations
using LinearAlgebra
using ForwardDiff

# Defining Arthur model using RigidBodyDynamics
struct Arthur{T} <: AbstractModel
    mechanism::T
    function Arthur(mechanism)
        T = eltype(RigidBodyDynamics.Mechanism)
        new{T}(mechanism)
    end
end

#TODO: Change Path
Arthur(; mechanism=RigidBodyDynamics.URDF.parse_urdf("../../../kortex_description/arms/gen3/7dof/urdf/GEN3_URDF_V12.urdf", remove_fixed_tree_joints = false)) = Arthur(mechanism)

# State, sx, is [q q̇ p ṗ F]
# p will be input from the camera
# q, q̇, ṗ will be taken or derived from the arm
# F will be input from the F/T Sensor
# Input, u, is Torque (τ)
function RobotDynamics.dynamics(model::Arthur, x::AbstractVector{T}, u) where T
    # Create a state of the mechanism model and a result struct for the dynamics
    dynamicsResult = RigidBodyDynamics.DynamicsResult{T}(model.mechanism)
    mechanismState = RigidBodyDynamics.MechanismState{T}(model.mechanism)
    print("Check1\n")
    # Get states and constants of system not dependent on model state
    M = RigidBodyDynamics.mass_matrix(mechanismState)
    num_q = RigidBodyDynamics.num_positions(model.mechanism)
    print("Check2\n")
    q = x[1:num_q]
    q̇ = x[num_q+1:2*num_q]
    p = x[2*num_q + 1:2*num_q + 6]
    ṗ = x[2*num_q + 7:2*num_q + 12]
    F = x[2*num_q + 13:2*num_q + 18]
    print("Check2.1\n")
    Be = zeros(eltype(x), 6, 6)
    if (norm(ṗ) > 1e-5)
        for k = 1:6
            print(k, "Check2.2\n")
            normF = norm(F)
            print(k, "Check2.3\n")
            normpd = norm(ṗ)
            print(k, "Check2.4\n")
            n = normF / normpd
            print(k, "Check2.5\n")
            Be[k,k] = n
            print(k, "Check2.6\n")
        end
    end
    print("Check3\n")
    # Set mechanism state to current state
    RigidBodyDynamics.set_configuration!(mechanismState, q)
    print("Check4\n")
    RigidBodyDynamics.set_velocity!(mechanismState, q̇)
    print("Check5\n")
    w = Wrench{T}(default_frame(bodies(model.mechanism)[end-1]), F[4:6], F[1:3])
    print("Check5.1\n")
    wrenches = BodyDict{Wrench{T}}(b => zero(Wrench{T}, root_frame(model.mechanism)) for b in bodies(model.mechanism))
    print("Check5.2\n")
    wrenches[bodies(model.mechanism)[end-1].id] = transform(w, transform_to_root(mechanismState, bodies(model.mechanism)[end-1]))
    print("Check6\n")
    dynamics!(dynamicsResult, mechanismState, u, wrenches)
    print("Check7\n")
    q̈ = dynamicsResult.v̇
    p̈ = [dynamicsResult.accelerations[bodies(model.mechanism)[end].id].linear; dynamicsResult.accelerations[bodies(model.mechanism)[end].id].angular]
    Ḟ = Be*p̈
    print("Check8\n")
    return SVector{32}([q̇; q̈; ṗ; p̈; Ḟ])
end

RobotDynamics.state_dim(::Arthur) = 32
RobotDynamics.control_dim(::Arthur) = 7

# function Base.convert(::Type{Float64}, x::ForwardDiff.Dual{ForwardDiff.Tag{RobotDynamics.var"#fd_aug#16"{RK3, Arthur{Any}, Float64, Float64, SVector{7, Int64}, SVector{32, Int64}}, Float64}, Float64, 39})
#     print(x)
#     print("\n\n")
#     print(typeof(x.partials.values))
#     print("\n\n")
#     print(x.value)
#     print("\n\n")
#     print([x.value; x.partials.values])
# end