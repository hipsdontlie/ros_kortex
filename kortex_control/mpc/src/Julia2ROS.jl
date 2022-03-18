#!/usr/bin/env julia

#Import Statements for MPC
include("Arthur.jl")
using RigidBodyDynamics
using StaticArrays
using RobotDynamics
using Rotations
using LinearAlgebra
using ForwardDiff, FiniteDiff
using Altro
using TrajectoryOptimization


# Black magic to make Julia and Python happy
using Distributed

using Pkg
Distributed.@everywhere using Pkg

Distributed.@everywhere begin
  ENV["PYTHON"] = "/usr/bin/python3"
  Pkg.build("PyCall")
end

using PyCall
Distributed.@everywhere using PyCall


#Import Statements for RobotOS
using RobotOS

#For Testing Purposes
@rosimport geometry_msgs.msg: Point
rostypegen()
using .geometry_msgs.msg

# function callback(msg::Point,pub_obj::Publisher{Point})
#     pt_msg = Point(10,10,10)
#     publish(pub_obj,pt_msg)
# end

# function loop(pub_obj)
#     loop_rate = Rate(5.0)
#     while ! is_shutdown()
#         pt_msg = Point(rand(),rand(),rand())
#         publish(pub_obj,pt_msg)
#         rossleep(loop_rate)
#     end
# end

# function main()
#     init_node("julia2ros")
#     pub = Publisher{Point}("julia_out",queue_size=10)
#     sub = Subscriber{Point}("julia_in",callback,(pub,),queue_size=10)
#     loop(pub)
# end

# if !isinteractive()
#     main()
# end


@rosimport trajectory_msgs.msg: JointTrajectoryPoint
rostypegen()
using .trajectory_msgs.msg


JointTrajectoryOutput = JointTrajectoryPoint([1,1,1],[1,1,1],[1,1,1],[1,1,1],Duration(1))

function callback(msg::Point,pub_obj::Publisher{JointTrajectoryPoint})
    # JointTrajectory = msg.positions
    #Do magic here

    JointTrajectoryOutput = JointTrajectoryPoint([0,0,0],[0,0,0],[0,0,0],[0,0,0],Duration(0))

    publish(pub_obj,JointTrajectoryOutput)
end

function loop(pub_obj)
    loop_rate = Rate(5.0)
    while ! is_shutdown()
        publish(pub_obj,JointTrajectoryOutput)
        rossleep(loop_rate)
    end
end

function main()
    init_node("julia2ros")
    pub = Publisher{JointTrajectoryPoint}("julia_out",queue_size=10)
    sub = Subscriber{Point}("julia_in",callback,(pub,),queue_size=10)
    loop(pub)
end

if !isinteractive()
    main()
end
