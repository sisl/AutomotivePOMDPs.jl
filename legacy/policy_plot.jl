using PGFPlots, GridInterpolations, Colors, ColorBrewer
using AutomotivePOMDPs
using POMDPPolicies
using SARSOP
using LinearAlgebra
using POMDPs
using POMDPModelTools

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
        ego = xv_to_state(pomdp, x, v_ego)
        ped = yv_to_state(pomdp, y, v_ped)
        s = SingleOCState(collision_checker(ego, ped, pomdp.ego_type, pomdp.ped_type), ego, ped)
        si = state_index(pomdp, s)
        values[:,i] =  policy.alphas[si, :]
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
    belief = SparseCat(Vector{SingleOCState}(n_bins), Vector{Float64}(n_bins))
    for i=1:n_bins
        ego = xv_to_state(pomdp, x, v_ego)
        ped = yv_to_state(pomdp, y_noise[i], v_ped)
        s = SingleOCState(collision_checker(ego, ped, pomdp.ego_type, pomdp.ped_type), ego, ped)
        weight = 1/(sig*sqrt(2*pi))*exp(-(y_noise[i] - y)^2/(2*sig^2))
        belief.it[i] = s
        belief.p[i] = weight
    end
    normalize!(belief.p, 1)
    return belief
end

# function compute_acts(pomdp::SingleOCPOMDP, policy::AlphaVectorPolicy, sig::Float64 = 0.01, v_ego::Float64 = 5., v_ped::Float64 = 1., n_bins::Int64 = 20, n_pts::Int64 = 100)
#     X, Y = get_XY_grid(pomdp::SingleOCPOMDP)
#     X = X[1:21]
#     grid = RectangleGrid(X, Y)
#     values = get_grid_data(policy, grid, pomdp, v_ego, v_ped)

#     xrange = LinRange(X[1], X[end], n_pts)
#     yrange = LinRange(Y[1], Y[end], n_pts)
#     action_map = zeros(n_pts, n_pts)
#     n_acts = n_actions(pomdp)

#     for (i,x) in enumerate(xrange)
#         for (j,y) in enumerate(yrange)
#             y_noise, weights = get_belief(y, sig, n_bins)
#             acts_xy = zeros(1, n_acts)
#             for k=1:n_acts
#                 for l=1:n_bins
#                     val = interpolate(grid, values[k,:], [x,y_noise[l]])
#                     acts_xy[k] += weights[l]*val
#                 end
#             end
#             max, indmax = findmax(acts_xy)
#             action_map[i,j] = policy.action_map[indmax].acc
#         end
#     end
#     acts = action_map
#     acts = acts'
#     acts = acts[end:-1:1,1:end]
#     return acts
# end

function compute_acts(pomdp::SingleOCPOMDP, policy::AlphaVectorPolicy, sig::Float64 = 0.01, v_ego::Float64 = 5., v_ped::Float64 = 1., n_bins::Int64 = 20, n_pts::Int64 = 100)
    X, Y = get_XY_grid(pomdp::SingleOCPOMDP)
    X = X[1:21]
    grid = RectangleGrid(X, Y)

    xrange = LinRange(X[1], X[end], n_pts)
    yrange = LinRange(Y[1], Y[end], n_pts)
    action_map = zeros(n_pts, n_pts)
    n_acts = n_actions(pomdp)

    for (i,x) in enumerate(xrange)
        for (j,y) in enumerate(yrange)
            belief = get_belief(x, y, sig, n_bins, v_ego, v_ped)
            max_val, ind_max = findmax(actionvalues(policy, belief))
            action_map[i,j] = policy.action_map[ind_max].acc
        end
    end
    acts = action_map
    acts = acts'
    acts = acts[end:-1:1,1:end]
    return acts
end

function policy_plot(pomdp::SingleOCPOMDP, policy::Policy, ;
                     sig::Float64 = 0.01, v_ego::Float64 = 5., v_ped::Float64 = 1., n_bins::Int64 = 20, n_pts::Int64 = 100)
    acts = compute_acts(pomdp, policy, sig, v_ego, v_ped, n_bins, n_pts)
    r = RGB{Float64}(1., 0., 0.) # red
    o = RGB{Float64}(1., 69/255.0, 0.) # orange
    y = RGB{Float64}(1., 1., 0.) # yellow
    g = RGB{Float64}(0., 1., 0.) # green
    colors = [r;o;y;g]
    cmap = ColorMaps.RGBArrayMap(colors)
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
