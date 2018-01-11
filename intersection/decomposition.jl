# Functions relating to the decomposition method

# Aliases
const DecBelief = Dict{Int64, OIDistribution}
const DecState = Dict{Int64, OIState}
const DecObs = Dict{Int64, OIObs}

"""
    POMDPs.value(policy::QMDPPolicy, b::OIDistribution)
Return the value of the belief according to the QMDP policy
"""
function POMDPs.value(policy::QMDPPolicy, b::OIDistribution)
    val = zeros(n_actions(policy.pomdp))
    for (i,s) in enumerate(b.it)
        val += value(policy, s)*b.p[i]
    end
    return val
end

"""
    POMDPs.value(policy::QMDPPolicy, s::OIState)
return the value of a given state using grid interpolations
"""
function POMDPs.value(policy::QMDPPolicy, s::OIState)
    # println("COUCOU")
    pomdp = policy.pomdp
    val = zeros(n_actions(pomdp))

    ego_grid = RectangleGrid(get_ego_s_grid(pomdp), get_v_grid(pomdp))
    ego_indices, ego_weights = interpolants(ego_grid, [s.ego.posF.s, s.ego.v])
    lane = pomdp.env.roadway[s.car.posF.roadind.tag]
    if !off_the_grid(pomdp, s.car)
        car_grid = RectangleGrid(get_lane_s(pomdp, lane), get_v_grid(pomdp))
        car_indices, car_weights = interpolants(car_grid, [s.car.posF.s, s.car.v])
    else
        car_indices = Int64[0]
        car_weights = Float64[1.0]
    end

    sum_weight = 0.
    for i=1:length(ego_indices)
        for j=1:length(car_indices)
            sg, vg = ind2x(ego_grid, ego_indices[i])
            ego = ego_state(pomdp, sg, vg)
            if car_indices[1] == 0
                car = get_off_the_grid(pomdp)
            else
                sg, v_carg = ind2x(car_grid, car_indices[j])
                car = car_state(pomdp, lane, sg, v_carg)
            end
            state = OIState(is_crash(pomdp,ego,car),ego,car)
            si = state_index(pomdp, state)
            sum_weight += ego_weights[i]*car_weights[j]
            val += policy.alphas[si, :]*ego_weights[i]*car_weights[j]
        end
    end
    val /= sum_weight
    return val
end


function POMDPs.action(policy::QMDPPolicy, b::OIDistribution)
    alphas = policy.alphas' #size |A|x|S|
    util = zeros(n_actions(pomdp)) # size |A|
    for i=1:n_actions(pomdp)
        res = 0.0
        for (j,s) in enumerate(b.it)
            si = state_index(pomdp, s)
            res += alphas[i, si]*b.p[j]
        end
        util[i] = res
    end
    ihi = indmax(util)
    return policy.action_map[ihi]
end

function POMDPs.action(policy::Policy, b::Dict{Int64, OIDistribution})
    val = fuse_value_min(policy, b)
    max_ind = indmax(val)
    return policy.action_map[max_ind]
end

"""
    fuse_value{B}(policy::Policy, b::Dict{Int64,B})
Fuse the value of different belief by summing them
"""
function fuse_value(policy::Policy, b::Dict{Int64,OIDistribution})
    val = zeros(n_actions(policy.pomdp))
    for id in keys(b)
        val += value(policy, b[id])
    end
    return val
end

"""
    fuse_value_min{B}(policy::Policy, b::Dict{Int64, B})
Fuse the value of different belief by taking the min of the two for each action
"""
function fuse_value_min(policy::Policy, b::Dict{Int64, OIDistribution})
    val = zeros(n_actions(policy.pomdp))
    for i=1:n_actions(policy.pomdp)
        min_val = +Inf
        for id in keys(b)
            v_id = value(policy, b[id])[i]
            if v_id < min_val
                min_val = v_id
            end
        end
        val[i] = min_val
    end
    return val
end

function POMDPs.update(up::Updater, bold::Dict{Int64, OIDistribution}, a::Float64, o::Dict{Int64, OIObs})
    bnew = Dict{Int64, OIDistribution}()
    for id in keys(bold)
        bnew[id] = update(up, bold[id], OIAction(a), o[id])
    end
    return bnew
end

function POMDPs.initial_state_distribution(pomdp::OIPOMDP, n::Int64)
    init_dis = DecBelief()
    for i=1:n
        init_dis[i+1] = initial_state_distribution(pomdp)
    end
    return init_dis
end

### HELPERS

"""
return the closest states and the associated weights using multi linear interpolation
"""
function interpolate_state(s::OIState, pomdp::OIPOMDP)
    ego_grid = RectangleGrid(get_ego_s_grid(pomdp), get_v_grid(pomdp)) #XXX must not allocate the grid at each function call, find better implementation
    ego_indices, ego_weights = interpolants(ego_grid, [s.ego.posG.x, s.ego.v])
    lane = env.roadway[s.car.posF.roadind.tag]
    if !off_the_grid(pomdp, s.car)
        car_grid = RectangleGrid(get_lane_s(pomdp, lane), get_v_grid(pomdp))
        car_indices, car_weights = interpolants(car_grid, [s.car.posG.y, s.car.v])
    else
        car_indices = Int64[0]
        car_weights = Float64[1.0]
    end

    states = OIState[] #todo preallocate
    weights = Float64[]
    for i=1:length(ego_indices)
        for j=1:length(car_indices)
            sg, vg = ind2x(ego_grid, ego_indices[i])
            ego = ego_state(pomdp, sg, vg)
            if car_indices[1] == 0
                car = get_off_the_grid(pomdp)
            else
                sg, v_carg = ind2x(car_grid, car_indices[j])
                car = car_state(pomdp, lane, sg, v_carg)
            end
            state = OIState(is_crash(pomdp,ego,car),ego,car)
            push!(states, state)
            push!(weights, ego_weights[i]*car_weights[j])
        end
    end
    normalize!(weights, 1)
    return states, weights
end

"""
    scene_to_states(scene::Scene, pomdp::OIPOMDP)
Convert a scene to a state representation, for each pedestrian present in the scene, a state object is created
"""
function scene_to_states(scene::Scene, pomdp::OIPOMDP)
    ego = scene[findfirst(scene, 1)]
    @assert scene.entities[1].id == 1
    states = Dict{Int64, OIState}()
    if scene.n == 1
        states[2] = OIState(false, ego, get_off_the_grid(pomdp))
    end
    for i=1:scene.n - 1
        car = scene[i+1]
        states[car.id] = OIState(is_crash(pomdp, ego, car.state), ego, car.state)
    end
    return states
end

"""
    states_to_scene(states::Dict{Int64, OIState}, pomdp::OIPOMDP)
"""
function states_to_scene(states::Dict{Int64, OIState}, pomdp::OIPOMDP)
    scene = Scene()
    for id in keys(states)
        if scene.n == 0
            ego = Vehicle(states[id].ego, pomdp.ego_type, 1)
            push!(scene, ego)
        end
        ped = Vehicle(states[id].car, pomdp.car_type, id)
        push!(scene, ped)
    end
    return scene
end
