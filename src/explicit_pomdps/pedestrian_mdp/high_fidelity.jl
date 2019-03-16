function interpolate_state(mdp::PedMDP, s::PedMDPState)
    # interpolate s in the grid
    ped_v = get_ped_vspace(mdp.env, mdp.vel_ped_res) 
    itp_ped, itp_ped_w = interpolate_pedestrian(mdp, s.ped, ped_v)
    ego_v = get_car_vspace(mdp.env, mdp.vel_res)
    itp_ego, itp_ego_w = conservative_interpolation(mdp, s.ego, ego_v)
    itp_states = Vector{PedMDPState}(length(itp_ego)*length(itp_ped))
    itp_w = Vector{Float64}(length(itp_states))
    k = 1
    for (i, p) in enumerate(itp_ped)
        for (j, e) in enumerate(itp_ego)
            crash = is_colliding(Vehicle(p, mdp.ped_type, 0), Vehicle(e, mdp.ego_type, 1))
            itp_states[k] = PedMDPState(crash, e, p)
            itp_w[k] = itp_ped_w[i]*itp_ego_w[j]
            k += 1
        end
    end
    @assert sum(itp_w) ≈ 1.
    return itp_states, itp_w
end

function get_mdp_state(mdp::PedMDP, s::Scene, ped_id::Int64)
    ped_i = findfirst(ped_id, s)
    ped = Vehicle(get_off_the_grid(mdp), mdp.ped_type, ped_id)
    if ped_i != nothing
        ped = s[ped_i]
    end
    ego = get_ego(s)
    e_state = VehicleState(ego.state.posG, car_roadway(mdp.env), ego.state.v)
    p_lane = get_lane(mdp.env.roadway, ped.state)
    if p_lane.tag ∈ [LaneTag(17,1), LaneTag(18,1), LaneTag(19,1)]
        if ped.state.posG.θ == 0. ||ped.state.posG.θ == float(pi) # solve ambiguity in projection
            p_lane = mdp.env.ped_roadway[LaneTag(19,1)]
        end
        p_state = VehicleState(ped.state.posG, p_lane, mdp.env.ped_roadway, ped.state.v)
    else
        p_state = VehicleState(ped.state.posG, mdp.env.ped_roadway, ped.state.v)
    end
    return PedMDPState(is_colliding(ego, ped), e_state, p_state)
end
function get_mdp_state(mdp::PedMDP, pomdp::UrbanPOMDP, s::Scene, ped_id::Int64)
    return get_mdp_state(mdp, s, ped_id)
end
