
# Function to plot the robot
function plot_robot(θ, l, c)
    # Compute joint positions
    positions = forward_kinematics(θ, l)
    xs, ys = [p[1] for p in positions], [p[2] for p in positions]

    # Plot
    wdth=sum(l)
    plot(xs, ys, label="Manipulator", lw=2, color=:blue, marker=:circle, markersize=6,aspect_ratio=:equal,xlims=(-wdth,wdth),ylims=(-wdth,wdth))
    scatter!(xs, ys, label="Joints", color=:red)
    title!("Planar 4-Link Manipulator")
    xlabel!("X")
    ylabel!("Y")
end

function animate_q(θ_array,fps,name="anim")

    if fps>25
        N=size(θ_array,1)
        frame_rate=25
        tvec_in=(1/fps):(1/fps):(1/fps)*N
        tvec_reduced=(1/frame_rate):(1/frame_rate):(1/fps)*N
        θ_array=general_lin_interp(θ_array,tvec_in,tvec_reduced)
    else
        frame_rate=fps
    end

    anim = @animate for i in 1:size(θ_array,1)
        plot()
        p=plot_robot(θ_array[i,:], l, c)
        plot(p)
    end
    gif(anim,"plots/"*name*".gif",fps=frame_rate);
end

##Function for interpolating a data matrix.
function general_lin_interp_old(dataset::AbstractMatrix{<:Real}, 
    tvec::AbstractVector{<:Real}, 
    tvec_new::AbstractVector{<:Real})     
    # Check for input consistency
    @assert size(dataset, 1) == length(tvec) "Time dimension mismatch between dataset and tvec"

    # Preallocate output matrix
    interped_data = similar(dataset, length(tvec_new), size(dataset, 2))

    # Perform interpolation for each column
    for i in 1:size(dataset, 2)
    interp_fn = LinearInterpolation(tvec, dataset[:, i], extrapolation_bc=Line())
    interped_data[:, i] .= interp_fn(tvec_new)
    end

    return interped_data
end

function general_lin_interp(dataset::AbstractMatrix{<:Real}, 
    tvec::AbstractVector{<:Real}, 
    tvec_new::AbstractVector{<:Real})
    @assert size(dataset, 1) == length(tvec) "Time dimension mismatch between dataset and tvec"

    interped_data = similar(dataset, length(tvec_new), size(dataset, 2))

    # Precompute interpolation objects for each column
    interp_fns = [LinearInterpolation(tvec, dataset[:, i], extrapolation_bc=Line()) for i in 1:size(dataset, 2)]
    # Interpolate each column
    for i in 1:size(dataset, 2)
        interped_data[:, i] .= interp_fns[i](tvec_new)
    end

    return interped_data
end