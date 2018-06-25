function interpolate_state(mdp::PedCarMDP, s::PedCarMDPState)
    # interpolate s in the grid
    vspace = get_car_vspace(mdp.env, mdp.vel_res)
    v_ped_space = get_ped_vspace(mdp.env, mdp.vel_ped_res)
    itp_ped, itp_ped_w = interpolate_pedestrian(get_ped_mdp(mdp), s.ped, v_ped_space)
    itp_car, itp_car_w = interpolate_state(get_car_mdp(mdp), s.car, vspace)
    itp_ego, itp_ego_w = interpolate_state(mdp, s.ego, vspace)
    itp_states = Vector{PedCarMDPState}(length(itp_ego)*length(itp_car)*length(itp_ped))
    itp_w = Vector{Float64}(length(itp_states))
    l = 1
    for (i, p) in enumerate(itp_ped)
        for (j, c) in enumerate(itp_car)
            for (k, e) in enumerate(itp_ego)
                itp_states[l] = PedCarMDPState(crash(mdp, s), e, p, c, s.route)
                itp_w[l] = itp_ped_w[i]*itp_car_w[j]*itp_ego_w[k]
                l += 1
            end
        end
    end
    @assert sum(itp_w) ≈ 1.
    return itp_states, itp_w
end

function get_mdp_state(mdp::PedCarMDP, pomdp::UrbanPOMDP, s::Scene, ped_id, car_id)
    car_i = findfirst(s, car_id)
    car = Vehicle(get_off_the_grid(mdp), mdp.car_type, car_id)
    if car_i != 0
        car = s[car_i]
    end
    ped_i = findfirst(s, ped_id)
    ped = Vehicle(get_off_the_grid(mdp), mdp.ped_type, ped_id)
    if ped_i != 0
        ped = s[ped_i]
    end
    ego = get_ego(s)
    # find route 
    sroute = nothing
    if haskey(pomdp.models, car_id)
        # find the exact route from the list of routes
        curr_route = [l.tag for l in pomdp.models[car_id].navigator.route]
        for route in get_car_routes(mdp.env)
            tags = intersect(Set(curr_route), Set(route))
            if length(tags) >= 2
                sroute = SVector{2, LaneTag}(route[1], route[end])
            elseif length(curr_route) == 1 && curr_route[1] ∈ route
                sroute = SVector{2, LaneTag}(route[1], route[end])                
            end
        end
        if sroute == nothing
            println(curr_route)
        end       
    else 
        sroute = SVector{2, LaneTag}(LaneTag(0,0), LaneTag(0,0))
    end
    e_state = VehicleState(ego.state.posG, car_roadway(mdp.env), ego.state.v)
    p_state = VehicleState(ped.state.posG, mdp.env.ped_roadway, ped.state.v)
    c_state = VehicleState(car.state.posG, car_roadway(mdp.env), car.state.v)
    return PedCarMDPState(is_colliding(ego, car), e_state, p_state, c_state, sroute)
end

function get_car_mdp(mdp::PedCarMDP)
    # return a corresponding CarMDP
    return CarMDP(mdp.env, mdp.ego_type, mdp.car_type, mdp.car_model, mdp.max_acc, 
                  mdp.pos_res, mdp.vel_res, mdp.ego_start, mdp.ego_goal, mdp.off_grid, 
                  mdp.ΔT, mdp.car_birth, mdp.collision_cost, mdp.action_cost,
                  mdp.goal_reward, mdp.γ)
end

function get_ped_mdp(mdp::PedCarMDP)
    # return a corresponding CarMDP
    return PedMDP(mdp.env, mdp.ego_type, mdp.ped_type, mdp.ped_model, mdp.max_acc, 
                  mdp.pos_res, mdp.vel_res, mdp.vel_ped_res, mdp.ego_start, mdp.ego_goal, mdp.off_grid, 
                  mdp.ΔT, mdp.ped_birth, mdp.collision_cost, mdp.action_cost,
                  mdp.goal_reward, mdp.γ)
end