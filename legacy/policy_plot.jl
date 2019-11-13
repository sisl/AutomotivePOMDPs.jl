using PGFPlots, GridInterpolations, Colors, ColorBrewer
using AutomotivePOMDPs
using AutomotiveDrivingModels
using POMDPPolicies
using SARSOP
using LinearAlgebra
using POMDPs
using POMDPModelTools
using POMDPPolicies
using ProgressMeter

function get_XY_grid(pomdp::SingleOCPOMDP)
    env = pomdp.env
    rl = env.params.roadway_length
    cw = env.params.crosswalk_width
    lw = env.params.lane_width
    cl = env.params.crosswalk_length
    x_ped = rl/2
    Y = LinRange(-cl/4, cl/4, Int(floor(cl/2/pomdp.pos_res)) + 1)
    X = LinRange(5., rl/2 + 2*cw , Int(floor((rl/2 + 2*cw -5)/pomdp.pos_res)) + 1)
    return X, Y
end

function get_grid_data(policy::AlphaVectorPolicy, grid::RectangleGrid, pomdp::SingleOCPOMDP, v_ego::Float64, v_ped::Float64)
    values = zeros(n_actions(pomdp),length(grid))
    for i=1:length(grid)
        x, y = ind2x(grid, i)
        ego = AutomotivePOMDPs.xv_to_state(pomdp, x, v_ego)
        ped = AutomotivePOMDPs.yv_to_state(pomdp, y, v_ped)
        s = SingleOCState(collision_checker(ego, ped, pomdp.ego_type, pomdp.ped_type), ego, ped)
        # si = stateindex(pomdp, s)
        values[:,i] =  actionvalues(policy, Deterministic(s))
    end
    return values
end

function get_belief(y::Float64, sig::Float64, n_bins::Int64)
    y_noise = LinRange(y-2*sig, y+2*sig, n_bins)
    weights = zeros(n_bins)
    for i=1:n_bins
        weights[i] = 1/(sig*sqrt(2*pi))*exp(-(y_noise[i] - y)^2/(2*sig^2))
    end
    normalize!(weights, 1)
    return y_noise, weights
end

function get_belief(pomdp::SingleOCPOMDP, x::Float64, y::Float64, sig::Float64, n_bins::Int64, v_ego::Float64, v_ped::Float64)
    y_noise = LinRange(y-2*sig, y+2*sig, n_bins)
    belief = SparseCat(Vector{SingleOCState}(undef, n_bins), Vector{Float64}(undef, n_bins))
    for i=1:n_bins
        ego = AutomotivePOMDPs.xv_to_state(pomdp, x, v_ego)
        ped = AutomotivePOMDPs.yv_to_state(pomdp, y_noise[i], v_ped)
        s = SingleOCState(collision_checker(ego, ped, pomdp.ego_type, pomdp.ped_type), ego, ped)
        weight = 1/(sig*sqrt(2*pi))*exp(-(y_noise[i] - y)^2/(2*sig^2))
        belief.vals[i] = s
        belief.probs[i] = weight
    end
    normalize!(belief.probs, 1)
    return belief
end

function get_deterministic_belief(pomdp::SingleOCPOMDP, x::Float64, y::Float64, sig::Float64, n_bins::Int64, v_ego::Float64, v_ped::Float64)
    ego = AutomotivePOMDPs.xv_to_state(pomdp, x, v_ego)
    ped = AutomotivePOMDPs.yv_to_state(pomdp, y, v_ped)
    s = SingleOCState(collision_checker(ego, ped, pomdp.ego_type, pomdp.ped_type), ego, ped)
    return Deterministic(s)
end

function interpolate_belief(pomdp::SingleOCPOMDP, x::Float64, y::Float64, sig::Float64, n_bins::Int64, v_ego::Float64, v_ped::Float64)
    y_noise = LinRange(y-2*sig, y+2sig, n_bins)
    vals = SingleOCState[]
    probs = Float64[]
    X, Y = get_XY_grid(pomdp::SingleOCPOMDP)
    X = X[1:21]
    grid = RectangleGrid(X, Y)
    for i=1:n_bins 
        itp_xy, itp_weights = interpolants(grid, [x, y_noise[i]])
        for (j, xy_ind) in enumerate(itp_xy)
            xg, yg = ind2x(grid, xy_ind)
            ego = AutomotivePOMDPs.xv_to_state(pomdp, xg, v_ego)
            ped = AutomotivePOMDPs.yv_to_state(pomdp, yg, v_ped)
            s = SingleOCState(collision_checker(ego, ped, pomdp.ego_type, pomdp.ped_type), ego, ped)
            weight = itp_weights[j] * 1/(sig*sqrt(2*pi))*exp(-(y_noise[i] - y)^2/(2*sig^2))
            if !(s in vals)
                push!(vals, s)
                push!(probs, weight)
            else
                si = findfirst(isequal(s), vals)
                probs[si] += weight
            end
        end
    end
    normalize!(probs, 1)
    return SparseCat(vals, probs)
end

function compute_acts(pomdp::SingleOCPOMDP, policy::AlphaVectorPolicy, sig::Float64 = 0.01, v_ego::Float64 = 5., v_ped::Float64 = 1., n_bins::Int64 = 20, n_pts::Int64 = 100)
    X, Y = get_XY_grid(pomdp::SingleOCPOMDP)
    X = X[1:21]
    grid = RectangleGrid(X, Y)
    values = get_grid_data(policy, grid, pomdp, v_ego, v_ped)

    xrange = LinRange(X[1], X[end], n_pts)
    yrange = LinRange(Y[1], Y[end], n_pts)
    action_map = zeros(n_pts, n_pts)
    n_acts = n_actions(pomdp)

    for (i,x) in enumerate(xrange)
        for (j,y) in enumerate(yrange)
            y_noise, weights = get_belief(y, sig, n_bins)
            acts_xy = zeros(n_acts)
            for k=1:n_acts
                for l=1:n_bins
                    val = interpolate(grid, values[k,:], [x,y_noise[l]])
                    acts_xy[k] += weights[l]*val
                end
            end
            max, indmax = findmax(acts_xy)
            action_map[i,j] = policy.action_map[indmax].acc
        end
    end
    acts = action_map
    acts = acts'
    acts = acts[end:-1:1,1:end]
    return acts
end

function compute_acts2(pomdp::SingleOCPOMDP, policy::AlphaVectorPolicy, sig::Float64 = 0.01, v_ego::Float64 = 5., v_ped::Float64 = 1., n_bins::Int64 = 20, n_pts::Int64 = 100)
    X, Y = get_XY_grid(pomdp)
    X = X[1:21]
    grid = RectangleGrid(X, Y)

    xrange = LinRange(X[1], X[end], n_pts)
    yrange = LinRange(Y[1], Y[end], n_pts)
    action_map = zeros(n_pts, n_pts)
    n_acts = n_actions(pomdp)

    @showprogress for (i,x) in enumerate(xrange)
        for (j,y) in enumerate(yrange)
            belief = get_deterministic_belief(pomdp, x, y, sig, n_bins, v_ego, v_ped)
            # belief = interpolate_belief(pomdp, x, y, sig, n_bins, v_ego, v_ped)
            max_val, ind_max = findmax(itp_values(policy, belief))
            action_map[i,j] = policy.action_map[ind_max].acc
            # action_map[i, j] = action(policy, belief).acc
        end
    end
    acts = action_map
    acts = acts'
    acts = acts[end:-1:1,1:end]
    return acts
end

function itp_values(policy::AlphaVectorPolicy, b)
    num_vectors = length(policy.alphas)
    max_values = -Inf * ones(length(actions(policy.pomdp)))
    for i=1:num_vectors
        ai = actionindex(policy.pomdp, policy.action_map[i])
        temp_value = itp_value(policy.alphas[i], b, policy.pomdp)
        # for (j, s) in enumerate(b.vals)
        #     s_itps, w_itps = interpolate_state(s, policy.pomdp)
        #     for (k, s_itp) in enumerate(s_itps)
        #         si = stateindex(policy.pomdp, s_itp)
        #         temp_value += policy.alphas[i][si]*b.probs[j]*w_itps[k]
        #     end
        # end
        if temp_value > max_values[ai]
            max_values[ai] = temp_value
        end
    end
    return max_values
end

function itp_value(alpha::AbstractArray, b::SparseCat, pomdp::POMDP)
    val = 0.0
    for (j, s) in enumerate(b.vals)
        s_itps, w_itps = interpolate_state(s, pomdp)
        for (k, s_itp) in enumerate(s_itps)
            si = stateindex(pomdp, s_itp)
            val += alpha[si]*b.probs[j]*w_itps[k]
        end
    end
    return val
end

function itp_value(alpha::AbstractArray, b::Deterministic, pomdp::POMDP)
    val = 0.0
    s_itps, w_itps = interpolate_state(b.val, pomdp)
    for (k, s_itp) in enumerate(s_itps)
        si = stateindex(pomdp, s_itp)
        val += alpha[si]*w_itps[k]
    end
    return val
end


function compute_acts3(pomdp::SingleOCPOMDP, policy::AlphaVectorPolicy, sig::Float64 = 0.01, v_ego::Float64 = 5., v_ped::Float64 = 1., n_bins::Int64 = 20, n_pts::Int64 = 100)
    X, Y = get_XY_grid(pomdp)
    X = X[1:21]
    action_map = zeros(length(X), length(Y))

    for (j, y) in enumerate(Y)
        for (i, x) in enumerate(X)
            ego = AutomotivePOMDPs.xv_to_state(pomdp, x, v_ego)
            ped = AutomotivePOMDPs.yv_to_state(pomdp, y, v_ped)
            s = SingleOCState(collision_checker(ego, ped, pomdp.ego_type, pomdp.ped_type), ego, ped)            
            a = action(policy, Deterministic(s))
            action_map[i, j] = a.acc
        end
    end
    acts = action_map
    acts = acts'
    acts = acts[end:-1:1,1:end]
    return acts
end


function policy_plot(pomdp::SingleOCPOMDP, policy::Policy, ;
                     sig::Float64 = 0.01, v_ego::Float64 = 5., v_ped::Float64 = 1., n_bins::Int64 = 20, n_pts::Int64 = 100)
    acts = compute_acts2(pomdp, policy, sig, v_ego, v_ped, n_bins, n_pts)
    r = RGB{Float64}(1., 0., 0.) # red
    o = RGB{Float64}(1., 69/255.0, 0.) # orange
    y = RGB{Float64}(1., 1., 0.) # yellow
    g = RGB{Float64}(0., 1., 0.) # green
    colors = [r;o;y;g]
    cmap = ColorMaps.RGBArrayMap(colormap("RdBu"))
    X, Y = get_XY_grid(pomdp::SingleOCPOMDP)
    X = X[1:21]
    ax =Axis(Plots.Image(acts, (X[1], X[end]), (Y[1], Y[end]), colormap = cmap),
    xlabel = "Ego car position on the road",
    ylabel = "Pedestrian position along the crosswalk",
    title = "Policy Plot for a fixed ego velocity at $v_ego m/s and fixed pedestrian speed at $v_ped m/s")
    return ax
end

# function compute_acts(pomdp::SingleOCPOMDP, policy::NNPolicy, sig::Float64 = 0.01, v_ego::Float64 = 5., v_ped::Float64 = 1., n_bins::Int64 = 20, n_pts::Int64 = 100)
#     X, Y = get_XY_grid(pomdp::SingleOCPOMDP)
#
#     xrange = LinRange(X[1], X[end], n_pts)
#     yrange = LinRange(Y[1], Y[end], n_pts)
# action_map = zeros(n_pts, n_pts)
# n_acts = n_actions(pomdp)
#     for (i,x) in enumerate(xrange)
#         for (j,y) in enumerate(yrange)
#             acts_xy = zeros(1, n_acts)
#             o = normalize_o(x, v_ego, y, v_ped, pomdp)
#             acts_xy = policy.nn[:value](o)
#             max, indmax = findmax(acts_xy)
#             action_map[i,j] = policy.action_map[indmax].acc
#         end
#     end
