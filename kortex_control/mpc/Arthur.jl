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
Arthur(; mechanism=RigidBodyDynamics.URDF.parse_urdf("/home/amkyu/catkin_workspace/src/ros_kortex/kortex_description/arms/gen3/7dof/urdf/GEN3_URDF_V12.urdf")) = Arthur(mechanism)

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
        for k = 1:3
            Be[k,k] = norm(F) / norm(ẋ)
        end
    end
    
    # Set mechanism state to current state
    RigidBodyDynamics.set_configuration!(mechanismState, q)
    RigidBodyDynamics.set_velocity!(mechanismState, q̇)
    
    # Get variables dependent on state
    J = getJacobian(model, q, q̇)
    τ_ext = transpose(J)*Be*ẋ
    
    # Calculate dynamics
    RigidBodyDynamics.dynamics!(dynamicsResult, mechanismState, u)
    # Add the effects of external forces/torques into dynamics
    q̈ = M\((M * dynamicsResult.v̇) - τ_ext)
    ẍ = getJ̇(model, J, q, q̇)*q̇ + J*q̈
    Ḟ = Be*ẍ
    return [q̇; q̈; ẋ; ẍ; Ḟ]
end

function getJacobian(model::Arthur, q,  q̇)
    mechanismState = RigidBodyDynamics.MechanismState(model.mechanism)
    RigidBodyDynamics.set_configuration!(mechanismState, q)
    RigidBodyDynamics.set_velocity!(mechanismState, q̇)
    p = RigidBodyDynamics.path(model.mechanism, RigidBodyDynamics.root_body(model.mechanism), RigidBodyDynamics.bodies(model.mechanism)[end])
    J_data = RigidBodyDynamics.geometric_jacobian(mechanismState, p)
#     print(typeof([J_data.linear; J_data.angular]))
    return [J_data.linear; J_data.angular]
end

function getẊ(model::Arthur, J, q, q̇)
#     J = getJacobian(model, q, q̇)
    ẋ = J*q̇
#     print(typeof(ẋ))
    return ẋ
end

function getJ̇(model::Arthur, J, q, q̇)
    return ForwardDiff.jacobian(dq -> getẊ(model, J, dq, q̇), q)
end

RobotDynamics.state_dim(::Arthur) = 32
RobotDynamics.control_dim(::Arthur) = 7

# model = Arthur()
# getJacobian, model, zeros(7), 0.1*ones(7)
# print(RobotDynamics.dynamics(model, zeros(32), 0.1*ones(7)))




