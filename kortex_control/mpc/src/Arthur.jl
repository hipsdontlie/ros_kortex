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

# State, s, is [q q̇ x ẋ F]
# x will be input from the camera
# q, q̇, ẋ will be taken or derived from the arm
# F will be input from the F/T Sensor
# Input, u, is Torque (τ)
function RobotDynamics.dynamics(model::Arthur, s, u)
    # Create a state of the mechanism model and a result struct for the dynamics
    dynamicsResult = RigidBodyDynamics.DynamicsResult(model.mechanism)
    mechanismState = RigidBodyDynamics.MechanismState(model.mechanism)
    
    # Get states and constants of system not dependent on model state
    M = RigidBodyDynamics.mass_matrix(mechanismState)
    num_q = RigidBodyDynamics.num_positions(model.mechanism)
    q = s[1:num_q]
    q̇ = s[num_q+1:2*num_q]
    x = s[2*num_q + 1:2*num_q + 6]
    ẋ = s[2*num_q + 7:2*num_q + 12]
    F = s[2*num_q + 13:2*num_q + 18]
    Be = zeros(6, 6)
    if (norm(ẋ) > 1e-5)
        for k = 1:6
            Be[k,k] = norm(F) / norm(ẋ)
        end
    end
    
    # Set mechanism state to current state
    RigidBodyDynamics.set_configuration!(mechanismState, q)
    RigidBodyDynamics.set_velocity!(mechanismState, q̇)
    
    w = Wrench(default_frame(bodies(model.mechanism)[end-1]), F[4:6], F[1:3])
    wrenches = BodyDict{Wrench{Float64}}(b => zero(Wrench{Float64}, root_frame(model.mechanism)) for b in bodies(model.mechanism))
    wrenches[bodies(model.mechanism)[end-1].id] = transform(w, transform_to_root(mechanismState, bodies(model.mechanism)[end-1]))
    dynamics!(dynamicsResult, mechanismState, u, wrenches)

    q̈ = dynamicsResult.v̇
    ẍ = [dynamicsResult.accelerations[bodies(model.mechanism)[end].id].linear; dynamicsResult.accelerations[bodies(model.mechanism)[end].id].angular]
    Ḟ = Be*ẍ
    return SVector{32}([q̇; q̈; ẋ; ẍ; Ḟ])
end

RobotDynamics.state_dim(::Arthur) = 32
RobotDynamics.control_dim(::Arthur) = 7
