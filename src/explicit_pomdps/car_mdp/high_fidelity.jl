function interpolate_state(mdp::CarMDP, s::CarMDPState)
    # interpolate s in the grid
    vspace = get_car_vspace(mdp.env, mdp.vel_res)
    itp_car, itp_car_w = interpolate_state(mdp, s.car, vspace)
    itp_ego, itp_ego_w = interpolate_state(mdp, s.ego, vspace)
    itp_states = Vector{CarMDPState}(length(itp_ego)*length(itp_car))
    itp_w = Vector{Float64}(length(itp_states))
    k = 1
    for (i, c) in enumerate(itp_car)
        for (j, e) in enumerate(itp_ego)
            crash = is_colliding(Vehicle(c, mdp.car_type, 0), Vehicle(e, mdp.ego_type, 1))
            itp_states[k] = CarMDPState(crash, e, c, s.route)
            itp_w[k] = itp_car_w[i]*itp_ego_w[j]
            k += 1
        end
    end
    @assert sum(itp_w) ≈ 1.
    return itp_states, itp_w
end


function get_mdp_state(mdp::CarMDP, pomdp::UrbanPOMDP, s::Scene, car_id = 2)
    car_i = findfirst(car_id, s)
    car = Vehicle(get_off_the_grid(mdp), mdp.car_type, car_id)
    if car_i != nothing
        car = s[car_i]
    end
    ego = get_ego(s)
    sroute = SVector{0, Lane}()
    # find route 
    sroute = nothing
    if haskey(pomdp.models, car_id) && car_i != nothing
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
    c_state = VehicleState(car.state.posG, mdp.env.roadway, car.state.v)
    return CarMDPState(is_colliding(ego, car), e_state, c_state, sroute)
end
