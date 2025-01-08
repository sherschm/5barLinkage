
# Forward kinematics for position of all joints
function forward_kinematics(θ, l)
    x, y = 0.0, 0.0
    total_angle = 0.0
    positions = []
    push!(positions, (x, y))
    for i in 1:length(θ)
        #total_angle += θ[i] #for relative joint coordinates
        total_angle = θ[i] #for absolute joint coordinates

        x += l[i] * cos(total_angle)
        y += l[i] * sin(total_angle)
        push!(positions, (x, y))
    end
    return positions
end

# Compute the Jacobian
function jacobian(θ, l)
    (x, y) = forward_kinematics(θ, l)[5]
    J = [Differential(θi)(x) for θi in θ] # Row for x
    J = hcat(J, [Differential(θi)(y) for θi in θ]) # Add row for y
    return expand_derivatives.(J)'
end

# Compute the time derivative of the Jacobian
function jacobian_derivative(θ, θd, l)
    J=jacobian(θ, l)
    n = length(θ)
    J_dot = zeros(Num, size(J))
    for i in 1:size(J, 1)
        for j in 1:size(J, 2)
            J_dot[i, j] = sum(θd[k]*Differential(θ[k])(J[i, j]) for k in 1:n)
        end
    end
    return expand_derivatives.(J_dot)
end

jacobian_f=eval(build_function(jacobian(θ, l),θ)[1]) #turn symbolic jacobian into a Julia Function
jacobian_derivative_f=eval(build_function(jacobian_derivative(θ, θd, l),[θ;θd])[1]) #turn symbolic jacobian derivative into a Julia Function
