
function energy_terms(θ, θd, l, c, m, I, g)

    #This is an interative algorithm for calculating kinetic & potential energy of the system

    n = length(θ)
    total_angle = 0.0
    v_prev = [0.0, 0.0] # Base velocity is zero
    p_prev = [0.0, 0.0] # Base position is zero

    T = 0.0 # Total kinetic energy
    U = 0.0 # Total potential energy

    for i in 1:n
        #total_angle += θ[i] #for relative joint coordinates
        total_angle = θ[i] #for absolute joint coordinates

        # Compute joint position
        p_curr = p_prev + [l[i] * cos(total_angle), l[i] * sin(total_angle)]
        # Compute center of mass position
        p_COM = p_prev + c[i] * [l[i] * cos(total_angle), l[i] * sin(total_angle)]
        # Compute joint velocity
        v_curr = v_prev + [-l[i] * sin(total_angle) * θd[i], l[i] * cos(total_angle) * θd[i]]
        # Compute center of mass velocity
        v_COM = v_prev + c[i] * [-l[i] * sin(total_angle) * θd[i], l[i] * cos(total_angle) * θd[i]]

        # Translational kinetic energy (COM velocity)
        T += 0.5 * m[i] *v_COM'*v_COM
        # Rotational kinetic energy (about COM)
        T += 0.5 * I[i] * θd[i]^2

        # Potential energy (COM height)
        U += m[i] * g * p_COM[2]

        # Update for next iteration
        v_prev = v_curr
        p_prev = p_curr
    end

    return T, U
end

# Lagrangian
function lagrangian(θ, θd, l, c, m, I, g)
    T, V= energy_terms(θ, θd, l, c, m, I, g)
    return T - V
end

# Derive equations of motion
function equations_of_motion(θ, θd, θdd, l, c, m, I, g)
    L = lagrangian(θ, θd, l, c, m, I, g)
    Dt = Differential(t)
    eqns = []
    subs=Dict{Num, Num}()
    for i in 1:length(θ)
        dLdθd= Differential(θd[i])(L)
        dLdθ = Differential(θ[i])(L)
        ddt_dLdθd= Dt(dLdθd)
        eqn = ddt_dLdθd-dLdθ
        push!(eqns, simplify(eqn))
        push!(subs,Dt(θ[i],) => θd[i], Dt(θd[i],) => θdd[i], Dt(Dt(θ[i],),) => θdd[i])
    end
    
    #eqns=simplify.(expand.(expand_derivatives.(eqns)))
    eqns=simplify.(expand_derivatives.(eqns))

    for i in 1:length(θ)
        eqns[i]=substitute(eqns[i], (subs))
    end

    return simplify.(eqns)
end

function extract_mass_matrix(eom, θdd)
    n = length(θdd)
    D = zeros(Num, n, n)
    for i in 1:n
        for j in 1:n
            # Extract coefficient of θdd[j] from the i-th equation
            D[i, j] = Symbolics.coeff(expand(simplify(expand(eom[i]))), θdd[j])
        end
    end
    return D
end

function simulate_system(dynamics,q0,tf,Δt)
    #simulate using Julia's ODE solver (from DifferentialEquations.jl)

    tspan = (0.0, tf)
    prob = ODEProblem(dynamics, q0, tspan)
    println("Simulating...")

    #tolerance precision is important due to constraint drift. for more info on this issue, check out Baumgarte Stabilisation
    sol = solve(prob, reltol = 1e-16, abstol = 1e-16)
    
    ##unpack solution
    tvec=sol.t

    x_sol=Array{Float64}(undef,length(tvec),4)  #joint position array
    xdot_sol=Array{Float64}(undef,length(tvec),4) #joint velocity array
    for i in 1:size(sol.u)[1]
        x_sol[i,:]=sol.u[i][1:4]
        xdot_sol[i,:]=sol.u[i][4+1:2*4]
    end

    #interpolate solution for making animation
    tvec_out=Δt:Δt:tf
    x_out=general_lin_interp(x_sol,tvec,tvec_out)
    xdot_out=general_lin_interp(xdot_sol,tvec,tvec_out)
    
    return tvec_out, x_out, xdot_out#, F_e
end

#=
Baumgarte stabilisation not necessary
function calc_lagrange_multiplier_Baumgarte(x,J,Jd)
    #compute Lagrange multipliers (see eq 6.6 from "A Mathematical Introduction to Robotic Manipulation" (Murray, Li & Sastry))
    #the lagrange multipliers will actually equal the constraint forces!
    θ=x[1:4]
    θd=x[5:8]

    α=0.01 #parameter for tuning Baumgarte stabilisation
    β=0.01 #parameter for tuning Baumgarte stabilisation

    θdd=LA.pinv(J)*(-(Jd*θd+2*α*J*θ+β^2*(forward_kinematics(θ, l)-[0;L])))

    LagMult=LA.pinv(J')*()

    return inv(J*inv(M_f(θ))*J')*(J*inv(M_f(θ))*(-N_f(x))+Jd*θd)
end
=#