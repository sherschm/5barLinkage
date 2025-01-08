println("Importing packages...")
using FiveBarLinkage
using Symbolics
using DifferentialEquations
using Plots
using Interpolations
import LinearAlgebra as LA
using DelimitedFiles

println("done!")

# Define symbolic variables for joint angles, velocities, accelerations
@variables t θ(t)[1:4] θd(t)[1:4] θdd(t)[1:4]

#define the system's constant parameters
l=[0.2;0.2;0.2;0.2] #link lengths
c=[0.5;0.5;0.5;0.5] #centre of mass of links as percentage of link lengths (0.5 means CoM is at midpoint of the link)
m=[0.1;0.1;0.1;0.1] #mass of each link
I=[0.01;0.01;0.01;0.01]  #moment of inertia of each link
g=9.81; #acceleration due to gravity

#import necessary functions from other scripts
include("Kinematic_functions.jl")
include("Dynamics_functions.jl")
include("Plot_functions.jl")

initial_config=[pi/3;0.0;0;-pi/3] #choose some initial set of joint angles to calculate a base distance, L
L=forward_kinematics(initial_config, l)[end][1] #calc the base separation distance, L

# Parameters for symbolic computation
n = 4 # Number of links
θ = [θ[i] for i in 1:n] #Symbolic joint angles
θd= [θd[i] for i in 1:n] #Symbolic joint velocities
θdd= [θdd[i] for i in 1:n] #Symbolic joint accelerations

# Compute Symbolic equations of motion of the 4 bar linkage
eom = equations_of_motion(θ, θd, θdd, l, c, m, I, g)

# Compute and display the mass matrix, and nonlinear terms (Coriolis & gravity forces)
mass_matrix = extract_mass_matrix(eom, θdd)
nonlinear_vector=expand.(expand.(eom)-expand.(mass_matrix)*θdd)

#turn mass matrix and nonlinear vector into Julia functions.
M_f=eval(build_function(mass_matrix,θ)[1])
N_f=eval(build_function(nonlinear_vector,[θ;θd])[1])

#choose control laws for joints 
torq1(t)=2*sin(2*t)
torq2(t)=2*sin(2*t)

function calc_lagrange_multiplier(x,t,J,Jd)
    #compute Lagrange multipliers (see eq 6.6 from "A Mathematical Introduction to Robotic Manipulation" (Murray, Li & Sastry))
    #the lagrange multipliers will actually equal the constraint forces!
    θ=x[1:4]
    θd=x[5:8]

    τ =[torq1(t);0;0;torq2(t)]

    return inv(J*inv(M_f(θ))*J')*(J*inv(M_f(θ))*(τ-N_f(x))+Jd*θd)
end

function dynamics_uncoupled(x,p,t)
    ## This basically describes the dynamics of a 4 bar linkage.
    ## this is quite chaotic since I have not included any damping terms in N_f.
    θ=x[1:4]
    θd=x[5:8]

    #u=[torq1(t);0;0;torq2(t)]
    u=[0;0;0;0] # no need to simulate control torques for this.. pendulum would go wild
    
    xd=[θd;inv(M_f(θ))*(u-N_f(x))]
    return xd
end

function dynamics_constrained(x,p,t)

    θ=x[1:4]
    θd=x[5:8]

    J=jacobian_f(θ)
    Jd=jacobian_derivative_f(x)

    u=[torq1(t);0;0;torq2(t)] # This is only correct if when using abolute angle coordinates. (thankfully, we are!)
    #if using relative joint coordinates, need to compute tool angle jacobian: u=[torq1;0;0;0]+J_angle'*torq2. 

    #compute Lagrange multipliers (see eq 6.6 from "A Mathematical Introduction to Robotic Manipulation" (Murray, Li & Sastry))
    LagMult=calc_lagrange_multiplier(x,t,J,Jd)

    xd=[θd;inv(M_f(θ))*(u-N_f(x)-J'*LagMult)]
    
    return xd
end

#Define simulation parameters
tf=10
Δt=0.05 #you may want to reduce this! If you do, note Julia animations can only go up to a maximum fps of 
q0=[initial_config;0;0;0;0]
plot_robot(initial_config, l, c)

#Simulate & animate unconstrained 4-link manipulator, just for example
tvec_out, θ_array, θdot_array=simulate_system(dynamics_uncoupled,q0,tf,Δt)
animate_q(θ_array,1/Δt,"unconstrained_system")

#Simulate & animate constrained linkage
tvec_out, θ_array_constrained, θdot_array_constrained=simulate_system(dynamics_constrained,q0,tf,Δt)
animate_q(θ_array_constrained,1/Δt,"constrained_system")

#build output data arrays for SINDy-Pi
X=[θ_array_constrained θdot_array_constrained]
#Xd can be calculated from X by acausal differentiation TBC!!

#calculate the constraint forces and position throughout the simulation:
constr_forces=Array{Float64}(undef,length(tvec_out),2)
constr_position=Array{Float64}(undef,length(tvec_out),2) #this should be constant throughout, if there is drift then try increasing ODE solver precision.
for i in 1:length(tvec_out)
    constr_position[i,:]=collect(forward_kinematics(X[i,1:4], l)[end])
    constr_forces[i,:]=calc_lagrange_multiplier(X[i,:],tvec_out[i],jacobian_f(X[i,1:4]),jacobian_derivative_f(X[i,:]))
end

# One solution to avoid "inverse M" appearing in SINDy-PI is to include the constraint forces in the control input array.
# For experimental SysID with SINDy-PI, this would require force sensors to measure the constraint forces...
U=[torq1.(tvec_out) torq2.(tvec_out) constr_forces]

#save data matrices to CSV files for SINDy-PI
writedlm("SINDyPI_data/U.csv",  U, ',')
writedlm("SINDyPI_data/X.csv",  X, ',')
writedlm("SINDyPI_data/tvec.csv",  tvec_out, ',')

#plot(constr_forces)

#plot(constr_position[:,1]) #should be constant!!

#plot(θ_array_constrained)