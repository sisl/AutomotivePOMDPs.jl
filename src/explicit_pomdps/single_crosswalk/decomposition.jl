# Functions relating to the decomposition method

# Aliases
const DecBelief = Dict{Int64, OCDistribution}
const DecState = Dict{Int64, OCState}
const DecObs = Dict{Int64, OCObs}

"""
    POMDPs.value(policy::QMDPPolicy, b::OCDistribution)
Return the value of the belief according to the QMDP policy
"""
function POMDPs.value(policy::QMDPPolicy, b::OCDistribution)
    val = zeros(n_actions(policy.pomdp))
    for (i,s) in enumerate(b.it)
        val += value(policy, s)*b.p[i]
        # si = state_index(pomdp, s)
        # val += policy.alphas[si, :]*b.p[i]
    end
    return val
end

function POMDPs.value(policy::SARSOPPolicy, b::OCDistribution)
    pomdp = policy.pomdp
    actions = policy.alphas.alpha_actions
    vectors = alphas(policy)
    n = size(vectors, 2)
    utilities = zeros(n_actions(pomdp), n)
    for i=1:n
        act = actions[i] + 1
        for (j, s) in enumerate(b.it)
            s_itps, w_itps = interpolate_state(s, pomdp)
            for (k, s_itp) in enumerate(s_itps)
                si = state_index(pomdp, s_itp)
                utilities[act, i] += vectors[si,i]*b.p[j]*w_itps[k]
            end
        end
    end
    return maximum(utilities,2)
end

"""
    POMDPs.value(policy::QMDPPolicy, s::OCState)
return the value of a given state using grid interpolations
"""
function POMDPs.value(policy::QMDPPolicy, s::OCState)
    # println("COUCOU")
    pomdp = policy.pomdp
    val = zeros(n_actions(pomdp))

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
            state = OCState(is_crash(pomdp,ego,ped),ego,ped)
            si = state_index(pomdp, state)
            sum_weight += ego_weights[i]*ped_weights[j]
            val += policy.alphas[si, :]*ego_weights[i]*ped_weights[j]
        end
    end
    val /= sum_weight
    return val
end

function POMDPs.action(policy::QMDPPolicy, b::OCDistribution)
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

# function POMDPs.action(policy::SARSOPPolicy, b::OCDistribution)
#     pomdp = policy.pomdp
#     val = zeros(n_actions(pomdp))
#     actions = policy.alphas.alpha_actions
#     vectors = alphas(policy)
#     n = size(vectors, 2)
#     utilities = zeros(n)
#     for i=1:n
#         res = 0.
#         for (j, s) in enumerate(b.it)
#             si = state_index(pomdp, s)
#             res += vectors[si, i]*b.p[j]
#         end
#         utilities[i] = res
#     end
#     a = actions[indmax(utilities)] + 1
#     return policy.action_map[a]
# end

function POMDPs.action(policy::SARSOPPolicy, b::OCDistribution)
    max_val, ind_max = findmax(value(policy, b))
    return policy.action_map[ind_max]
end


function POMDPs.action{B}(policy::Policy, b::Dict{Int64, B})
    val = fuse_value_min(policy, b)
    max_ind = indmax(val)
    return policy.action_map[max_ind]
end



"""
    fuse_value(policy::Policy, b::Dict{Int64,OCDistribution})
Fuse the value of different belief by summing them
"""
function fuse_value{B}(policy::Policy, b::Dict{Int64,B})
    val = zeros(n_actions(policy.pomdp))
    for id in keys(b)
        val_inc = value(policy, b[id])
        val += val_inc
        # println("For pedestrian ", id, "value is", val_inc)
    end
    return val
end

"""
    fuse_value_min(policy::Policy, b::Dict{Int64,OCDistribution})
Fuse the value of different belief by taking the min of the two for each action
"""
function fuse_value_min{B}(policy::Policy, b::Dict{Int64, B})
    val = zeros(n_actions(policy.pomdp))
    for i=1:n_actions(policy.pomdp)
        min_val = +Inf
        for id in keys(b)
            v_id = value(policy, b[id])
            # println("For pedestrian ", id, "value is", v_id)
            v_id = v_id[i]
            if v_id < min_val
                min_val = v_id
            end
        end
        val[i] = min_val
    end
    return val
end

"""
    generate_scene(pomdp::OCPOMDP, scene::Scene, a::OCAction, rng::AbstractRNG)
generate the next scene using the pomdp's transition model
"""
function generate_scene(pomdp::OCPOMDP, scene::Scene, a::OCAction, rng::AbstractRNG)
    states = scene_to_states(scene, pomdp)
    statesp = Dict{Int64, OCState}()
    for id in keys(states)
        statesp[id] = generate_s(pomdp, states[id], a, rng)
    end
    return states_to_scene(statesp, pomdp)
end

"""
    generate_scene(pomdp::OCPOMDP, scene::Scene, a::OCAction, rng::AbstractRNG)
generate a vector of states
"""
function generate_states(pomdp::OCPOMDP, states::Dict{Int64, OCState}, a::OCAction, rng::AbstractRNG)
    statesp = Dict{Int64, OCState}()
    for id in keys(states)
        statesp[id] = generate_s(pomdp, states[id], a, rng)
    end
    return statesp
end

"""
POMDPs.generate_o(pomdp::OCPOMDP, states::Dict{Int64, OCState}, a::OCAction, statesp::Dict{Int64, OCState}, rng::AbstractRNG)
generate an observation from a vector of states
"""
function POMDPs.generate_o(pomdp::OCPOMDP, states::Dict{Int64, OCState}, a::OCAction, statesp::Dict{Int64, OCState}, rng::AbstractRNG)
    o = Dict{Int64, OCObs}()
    for id in keys(statesp)
        o[id] = generate_o(pomdp, states[id], a, statesp[id], rng)
    end
    return o
end


# function POMDPs.update(up::OCUpdater, bold::Dict{Int64, OCDistribution}, a::OCAction, o::Dict{Int64, OCObs})
#     bnew = Dict{Int64, OCDistribution}()
#     #TODO check for new comers
#     for id in keys(bold)
#         bnew[id] = update(up, bold[id], a, o[id])
#     end
#     return bnew
# end
#
# function POMDPs.update(up::Updater, bold::Dict{Int64, OCDistribution}, a::Float64, o::Dict{Int64, OCObs})
#     bnew = Dict{Int64, OCDistribution}()
#     #TODO check for new comers
#     for id in keys(bold)
#         bnew[id] = update(up, bold[id], OCAction(a), o[id])
#     end
#     return bnew
# end

# function POMDPs.update{U, B}(up::U, bold::Dict{Int64, B}, a::Float64, o::Dict{Int64, OCObs})
#     bnew = Dict{Int64, B}()
#     #TODO check for new comers
#     for id in keys(bold)
#         bnew[id] = update(up, bold[id], OCAction(a), o[id])
#     end
#     return bnew
# end

#### HELPERS ######################################################################################



"""
return the closest states and the associated weights using multi linear interpolation
"""
function interpolate_state(s::OCState, pomdp::OCPOMDP)
    ego_grid = RectangleGrid(get_X_grid(pomdp), get_V_grid(pomdp)) #XXX must not allocate the grid at each function call, find better implementation
    ego_indices, ego_weights = interpolants(ego_grid, [s.ego.posG.x, s.ego.v])

    if !isinf(s.ped.v)
        ped_grid = RectangleGrid(get_Y_grid(pomdp), get_V_ped_grid(pomdp))
        ped_indices, ped_weights = interpolants(ped_grid, [s.ped.posG.y, s.ped.v])
    else
        ped_indices = Int64[0]
        ped_weights = Float64[1.0]
    end

    states = OCState[] #todo preallocate
    weights = Float64[]
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
            state = OCState(is_crash(pomdp,ego,ped),ego,ped)
            push!(states, state)
            push!(weights, ego_weights[i]*ped_weights[j])
        end
    end
    normalize!(weights, 1)
    return states, weights
end

"""
    scene_to_states(scene::Scene, pomdp::OCPOMDP)
Convert a scene to a state representation, for each pedestrian present in the scene, a state object is created
"""
function scene_to_states(scene::Scene, pomdp::OCPOMDP)
    ego = scene.entities[1].state
    @assert scene.entities[1].id == 1
    states = Dict{Int64, OCState}()
    if scene.n == 1
        states[2] = OCState(false, ego, get_off_the_grid(pomdp))
    end
    for i=1:scene.n - 1
        ped = scene.entities[i+1]
        states[ped.id] = OCState(is_crash(pomdp, ego, ped.state), ego, ped.state)
    end
    return states
end

"""
    states_to_scene(states::Dict{Int64, OCState}, pomdp::OCPOMDP)
"""
function states_to_scene(states::Dict{Int64, OCState}, pomdp::OCPOMDP)
    scene = Scene()
    for id in keys(states)
        if scene.n == 0
            ego = Vehicle(states[id].ego, pomdp.ego_type, 1)
            push!(scene, ego)
        end
        ped = Vehicle(states[id].ped, pomdp.ped_type, id)
        push!(scene, ped)
    end
    return scene
end

"""
    rand_scene(rng::AbstractRNG, b::Dict{Int64, OCDistribution}, env::CrosswalkEnv)
Sample a scene from a belief representation
"""
function rand_scene(rng::AbstractRNG, b::Dict{Int64, OCDistribution}, pomdp::OCPOMDP)
    scene = Scene()
    for id in keys(b)
        s = rand(rng, b[id])
        if scene.n == 0
            ego = Vehicle(s.ego, pomdp.ego_type, 1)
            push!(scene, ego)
        end
        ped = Vehicle(s.ped, pomdp.ped_type, id)
        push!(scene, ped)
    end
    return scene
end

"""
    initialize_two_beliefs(pomdp::OCPOMDP)
Initialize a belief considering two pedestrians
"""
function initialize_two_beliefs(pomdp::OCPOMDP)
    b = Dict{Int64, OCDistribution}()
    b[2] = initial_state_distribution(pomdp)
    b[3] = initial_state_distribution(pomdp)
    return b
end