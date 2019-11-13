using PGFPlots
using GridInterpolations
using Colors
using ColorBrewer
using POMDPs 
using AutomotivePOMDPs
using AutomotivePOMDPs: xv_to_state, yv_to_state
using POMDPModelTools
using LinearAlgebra
using ProgressMeter
using Profile


struct SARSOPPolicy{P<:Policy} 
    p::P
end

struct QMDPPolicy{P<:Policy}
    p::P
end


function POMDPs.value(policy::QMDPPolicy, b::SparseCat)
    val = zeros(n_actions(policy.pomdp))
    for (i,s) in enumerate(b.vals)
        val += value(policy, s)*b.p[i]
    end
    return val
end

function POMDPs.value(p::SARSOPPolicy, b::SparseCat)
    policy = p.p
    pomdp = policy.pomdp
    vectors = policy.alphas
    n = length(vectors)
    utilities = zeros(n_actions(pomdp), n)
    for i=1:n
        act = actionindex(pomdp, policy.action_map[i])
        for (j, s) in enumerate(b.vals)
            s_itps, w_itps = interpolate_state(s, pomdp)
            for (k, s_itp) in enumerate(s_itps)
                si = stateindex(pomdp, s_itp)
                utilities[act, i] += vectors[i][si]*b.probs[j]*w_itps[k]
            end
        end
    end
    return maximum(utilities, dims=2)
end

function POMDPs.value(p::QMDPPolicy, s::SingleOCState)
    policy = p.p
    pomdp = policy.pomdp
    val = zeros(n_actions(pomdp))
    alphas = hcat(policy.alphas...)

    ego_grid = RectangleGrid(get_X_grid(pomdp), get_V_grid(pomdp)) #XXX must not allocate the grid at each function call, find better implementation
    ego_indices, ego_weights = interpolants(ego_grid, [s.ego.posG.x, s.ego.v])

    if !isinf(s.ped.v)
        ped_grid = RectangleGrid(get_Y_grid(pomdp), get_V_ped_grid(pomdp))
        ped_indices, ped_weights = interpolants(ped_grid, [s.ped.posG.y, s.ped.v])
    else
        ped_indices = Int64[0]
        ped_weights = Float64[1.0]
    end

    sum_weight = 0.
    for i=1:length(ego_indices)
        for j=1:length(ped_indices)
            xg, vg = ind2x(ego_grid, ego_indices[i])
            ego = xv_to_state(pomdp, xg, vg)
            if ped_indices[1] == 0
                ped = get_off_the_grid(pomdp)
            else
                yg, v_pedg = ind2x(ped_grid, ped_indices[j])
                ped = yv_to_state(pomdp, yg, v_pedg)
            end
            state = SingleOCState(collision_checker(ego, ped, pomdp.ego_type, pomdp.ped_type),ego,ped)
            si = stateindex(pomdp, state)
            sum_weight += ego_weights[i]*ped_weights[j]
            val .+= view(alphas, si, :)*ego_weights[i]*ped_weights[j]
        end
    end
    val ./= sum_weight
    return val
end


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

function get_grid_data(p::QMDPPolicy, grid::RectangleGrid, pomdp::SingleOCPOMDP, v_ego::Float64, v_ped::Float64)
    policy = p.p
    values = zeros(n_actions(pomdp),length(grid))
    alphas = hcat(policy.alphas...)
    for i=1:length(grid)
        x, y = ind2x(grid, i)
        ego = xv_to_state(pomdp, x, v_ego)
        ped = yv_to_state(pomdp, y, v_ped)
        s = SingleOCState(collision_checker(ego, ped, pomdp.ego_type, pomdp.ped_type), ego, ped)
        si = stateindex(pomdp, s)
        values[:,i] =  alphas[si, :]
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

function get_belief(x::Float64, y::Float64, sig::Float64, n_bins::Int64, v_ego::Float64, v_ped::Float64)
    y_noise = LinRange(y-2*sig, y+2*sig, n_bins)
    belief = SparseCat(Vector{SingleOCState}(undef, n_bins),Vector{Float64}(undef, n_bins))
    for i=1:n_bins
        ego = xv_to_state(pomdp, x, v_ego)
        ped = yv_to_state(pomdp, y_noise[i], v_ped)
        s = SingleOCState(collision_checker(ego, ped, pomdp.ego_type, pomdp.ped_type), ego, ped)
        weight = 1/(sig*sqrt(2*pi))*exp(-(y_noise[i] - y)^2/(2*sig^2))
        belief.vals[i] = s
        belief.probs[i] = weight
    end
    normalize!(belief.probs, 1)
    return belief
end

function compute_acts(pomdp::SingleOCPOMDP, p::QMDPPolicy, sig::Float64 = 0.01, v_ego::Float64 = 5., v_ped::Float64 = 1., n_bins::Int64 = 20, n_pts::Int64 = 100)
    X, Y = get_XY_grid(pomdp)
    X = X[1:21]
    grid = RectangleGrid(X, Y)
    values = get_grid_data(p, grid, pomdp, v_ego, v_ped)

    policy = p.p
    xrange = LinRange(X[1], X[end], n_pts)
    yrange = LinRange(Y[1], Y[end], n_pts)
    action_map = zeros(n_pts, n_pts)
    n_acts = n_actions(pomdp)
    pomdp_amap = ordered_actions(pomdp)

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
            action_map[i,j] = pomdp_amap[indmax].acc
        end
    end
    acts = action_map
    acts = acts'
    acts = acts[end:-1:1,1:end]
    return acts
end

function compute_acts(pomdp::SingleOCPOMDP, policy::SARSOPPolicy, sig::Float64 = 0.01, v_ego::Float64 = 5., v_ped::Float64 = 1., n_bins::Int64 = 20, n_pts::Int64 = 100)
    X, Y = get_XY_grid(pomdp)
    X = X[1:21]
    grid = RectangleGrid(X, Y)

    xrange = LinRange(X[1], X[end], n_pts)
    yrange = LinRange(Y[1], Y[end], n_pts)
    action_map = zeros(n_pts, n_pts)
    n_acts = n_actions(pomdp)
    pomdp_amap = ordered_actions(pomdp)

    @showprogress for (i,x) in enumerate(xrange)
        for (j,y) in enumerate(yrange)
            belief = get_belief(x, y, sig, n_bins, v_ego, v_ped)
            max_val, ind_max = findmax(value(policy, belief))
            action_map[i,j] = pomdp_amap[ind_max].acc
        end
    end
    acts = action_map
    acts = acts'
    acts = acts[end:-1:1,1:end]
    return acts
end

function policy_plot(pomdp::SingleOCPOMDP, policy::Union{QMDPPolicy, SARSOPPolicy}, ;
                     sig::Float64 = 0.01, v_ego::Float64 = 5., v_ped::Float64 = 1., n_bins::Int64 = 20, n_pts::Int64 = 100)
    acts = compute_acts(pomdp, policy, sig, v_ego, v_ped, n_bins, n_pts)
    cmap = ColorMaps.RGBArrayMap(colormap("RdBu"))
    X, Y = get_XY_grid(pomdp)
    X = X[1:21]
    ax =Axis(Plots.Image(acts, (X[1], X[end]), (Y[1], Y[end]), colormap = cmap),
    xlabel = "Ego car position on the road",
    ylabel = "Pedestrian position along the crosswalk",
    title = "Policy Plot for a fixed ego velocity at $v_ego m/s and fixed pedestrian speed at $v_ped m/s")
    return ax
end
