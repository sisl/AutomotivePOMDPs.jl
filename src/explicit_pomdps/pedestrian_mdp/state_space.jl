#=
Define a discrete state space for the pedestrian MDP problem
=#

function POMDPs.n_states(mdp::PedMDP)
    n_ego = n_ego_states(mdp.env, mdp.pos_res, mdp.vel_res)
    n_ped = n_ped_states(mdp.env, mdp.pos_res, mdp.vel_ped_res)
    return n_ego*(n_ped + 1) # do not forget absent state
end

function POMDPs.states(mdp::PedMDP)
    state_space = PedMDPState[]
    ego_states = get_ego_states(mdp.env, mdp.pos_res, mdp.vel_res)
    ped_states = get_ped_states(mdp.env, mdp.pos_res, mdp.vel_ped_res)
    for ego in ego_states
        for ped in ped_states
            crash =  is_colliding(Vehicle(ego, mdp.ego_type, EGO_ID), Vehicle(ped, mdp.ped_type, PED_ID))
            push!(state_space, PedMDPState(crash, ego, ped))
        end
        # add absent states
        push!(state_space, PedMDPState(false, ego, get_off_the_grid(mdp)))
    end
    return state_space
end

function POMDPs.stateindex(mdp::PedMDP, s::PedMDPState)
    n_ego = n_ego_states(mdp.env, mdp.pos_res, mdp.vel_res)
    n_ped = n_ped_states(mdp.env, mdp.pos_res, mdp.vel_ped_res)
    # step 1: find ego_index
    ego_i = ego_stateindex(mdp.env, s.ego, mdp.pos_res, mdp.vel_res)
    # step 2: find ped_index
    if s.ped.posG == mdp.off_grid
        ped_i = n_ped + 1
    else
        ped_i = ped_stateindex(mdp.env, s.ped, mdp.pos_res, mdp.vel_ped_res)
    end
    # step 3: find global index
    si = LinearIndices((n_ped+1, n_ego))[ped_i, ego_i]
end

function POMDPs.initialstate_distribution(mdp::PedMDP)
    ego = initial_ego_state(mdp)
    init_ped_dist = initial_ped_state_distribution(mdp)
    init_ped_states = init_ped_dist.vals
    init_states = Vector{PedMDPState}(length(init_ped_states))
    for i=1:length(init_ped_states)
        ped = init_ped_states[i]
        crash = is_colliding(Vehicle(ego, mdp.ego_type, EGO_ID), Vehicle(ped, mdp.ped_type, PED_ID))
        init_states[i] = PedMDPState(crash, ego, ped)
    end
    return SparseCat(init_states, init_ped_dist.probs)
end

function initial_ego_state(mdp::PedMDP)
    lanes = get_ego_route(mdp.env)
    posF = Frenet(mdp.env.roadway[lanes[1]], mdp.ego_start)
    v0 = 0.
    return VehicleState(posF, mdp.env.roadway, v0)
end

function initial_ped_state_distribution(mdp::PedMDP)
    init_ped_states = get_ped_states(mdp.env, mdp.pos_res, mdp.vel_ped_res)
    push!(init_ped_states, get_off_the_grid(mdp))
    # uniform (maybe add more weights to the states when pedestrians are not there?)
    probs = ones(length(init_ped_states))
    normalize!(probs, 1)
    return SparseCat(init_ped_states, probs)
end

function initial_ped_state(mdp::PedMDP, rng::AbstractRNG)
    init_dist = initial_ped_state_distribution(mdp)
    return rand(rng, init_dist)
end

function POMDPs.initialstate(mdp::PedMDP, rng::AbstractRNG)
    return rand(rng, initialstate_distribution(mdp))
end

function pedestrian_starting_states(mdp::PedMDP)
    # list of pedestrian starting states
    n_headings = 2
    lanes = get_ped_lanes(mdp.env)
    v_space = get_ped_vspace(mdp.env, mdp.vel_ped_res)
    ped_states = Vector{VehicleState}(length(lanes)*length(v_space)*n_headings)
    i = 1
    for lane in lanes
        for v in v_space
            ped_states[i] = VehicleState(Frenet(mdp.env.roadway[lane], 0., 0., 0.), mdp.env.roadway, v)
            i += 1
            ped_states[i] = VehicleState(Frenet(mdp.env.roadway[lane], get_end(mdp.env.roadway[lane]), 0., float(pi)), mdp.env.roadway, v)
            i += 1
        end
    end
    return ped_states
end
