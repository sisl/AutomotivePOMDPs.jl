#=
Define a transition model for the PedCarMDP problem
=#


function POMDPs.transition(mdp::PedCarMDP, s::PedCarMDPState, a::PedCarMDPAction)
    ego_ps, ego_probs = ego_transition(mdp, s, a)
    ped_ps, ped_probs = ped_transition(mdp, s, a)
    car_ps, car_routes, car_probs = car_transition(mdp, s, a)
    nps = length(car_ps)*length(ped_ps)*length(ego_ps)
    states_p = Vector{PedCarMDPState}(nps)
    states_probs = Vector{Float64}(nps)
    idx = 1
    for ie=1:length(ego_ps)
        for ip=1:length(ped_ps)
            for ic=1:length(car_ps)
                weight = ego_probs[ie]*ped_probs[ip]*car_probs[ic]
                # if !(weight ≈ 0.)
                    # collision = crash(mdp, ego_ps[ie], car_ps[ic], ped_ps[ip])
                    collision = is_colliding(Vehicle(ego_ps[ie], mdp.ego_type, EGO_ID), Vehicle(car_ps[ic], mdp.car_type, CAR_ID)) || is_colliding(Vehicle(ego_ps[ie], mdp.ego_type, EGO_ID), Vehicle(ped_ps[ip], mdp.ped_type, PED_ID))
                    states_p[idx] = PedCarMDPState(collision, ego_ps[ie], ped_ps[ip], car_ps[ic], car_routes[ic])
                    states_probs[idx] = weight
                    idx += 1
                    # push!(states_p, PedCarMDPState(collision, ego_ps[ie], ped_ps[ip], car_ps[ic], car_routes[ic]))
                    # push!(states_probs, weight)
                # end
            end
        end
    end
    normalize!(states_probs, 1)
    return SparseCat(states_p, states_probs)
end

function ego_transition(mdp::PedCarMDP, s::PedCarMDPState, a::PedCarMDPAction)
    # ego transition
    acc = LonAccelDirection(a.acc, 4) #TODO paremeterize
    ego_p = propagate(s.ego, acc, mdp.env.roadway, mdp.ΔT)
    # interpolate ego_p in discrete space
    ego_v_space = get_car_vspace(mdp.env, mdp.vel_res)
    ego_ps, ego_probs = interpolate_state(mdp, ego_p, ego_v_space)
    return ego_ps, ego_probs
end

function ped_transition(mdp::PedCarMDP, s::PedCarMDPState, a::PedCarMDPAction)
    # pedestrian transition
    ped_probs = Float64[]
    if s.ped.posG == mdp.off_grid
        ped_ps = pedestrian_starting_states(mdp)
        push!(ped_ps, get_off_the_grid(mdp))
        ped_probs = ones(length(ped_ps))
        ped_probs[end] = 1-mdp.ped_birth
        ped_probs[1:end-1] = mdp.ped_birth/(length(ped_ps)-1)
        @assert sum(ped_probs) ≈ 1.0
    else
        ped_actions, ped_actions_probs = get_distribution(mdp.ped_model)
        ped_ps = VehicleState[]
        for (i, ped_a) in enumerate(ped_actions)
            p_a = ped_actions_probs[i]
            ped_p = propagate(s.ped, ped_a, mdp.env.roadway, mdp.ΔT)
            if (ped_p.posF.s >= get_end(get_lane(mdp.env.roadway, ped_p)) && isapprox(ped_p.posF.ϕ, 0.)) ||
               (isapprox(ped_p.posF.s, 0., atol=0.01) && isapprox(ped_p.posF.ϕ, float(pi)))
                ped_pp = get_off_the_grid(mdp)
                index_ped_pp = find(x -> x==ped_pp, ped_ps)
                if isempty(index_ped_pp)
                    push!(ped_ps, get_off_the_grid(mdp))
                    push!(ped_probs, p_a)
                else
                    ped_probs[index_ped_pp] += p_a
                end
                continue
            end
            # interpolate ped_p in discrete space
            ped_v_space = get_ped_vspace(mdp.env, mdp.vel_ped_res)
            itp_ped_ps, itp_ped_weights = interpolate_pedestrian(mdp, ped_p, ped_v_space)
            for (j, ped_pss) in enumerate(itp_ped_ps)
                index_itp_state = find(x -> x==ped_pss, ped_ps)
                if !(itp_ped_weights[j] ≈ 0.)
                    if isempty(index_itp_state)
                        push!(ped_ps, ped_pss)
                        push!(ped_probs, itp_ped_weights[j]*p_a)
                    else
                        ped_probs[index_itp_state] += itp_ped_weights[j]*p_a
                    end
                end
            end
        end
        @assert length(ped_probs) == length(ped_ps)
        normalize!(ped_probs, 1)
    end
    return ped_ps, ped_probs
end

function car_transition(mdp::PedCarMDP, s::PedCarMDPState, a::PedCarMDPAction)
    # car transition
    car_probs = Float64[]
    car_routes = SVector{2, LaneTag}[]
    if s.car.posG == mdp.off_grid
        car_ps, car_routes = car_starting_states(mdp)
        push!(car_ps, get_off_the_grid(mdp))
        push!(car_routes, SVector{2, LaneTag}(LaneTag(0,0), LaneTag(0,0)))
        car_probs = ones(length(car_ps))
        car_probs[end] = 1-mdp.car_birth
        car_probs[1:end-1] = mdp.car_birth/(length(car_ps)-1)
        @assert sum(car_probs) ≈ 1.0
    else
        # update model
        set_car_model!(mdp, s, a)
        lane = get_lane(mdp.env.roadway, s.car)
        car_actions, car_actions_probs = get_distribution(mdp.car_model)
        car_ps = VehicleState[]
        for (i, car_a) in enumerate(car_actions)
            p_a = car_actions_probs[i]
            car_p = propagate(s.car, car_a, mdp.env.roadway, mdp.ΔT)
            if car_p.posF.s >= get_end(get_lane(mdp.env.roadway, car_p)) && isempty(get_lane(mdp.env.roadway, car_p).exits)
                car_pp = get_off_the_grid(mdp)
                index_car_pp = find(x -> x==car_pp, car_ps)
                if isempty(index_car_pp)
                    push!(car_ps, car_pp)
                    push!(car_probs, p_a)
                else
                    car_probs[index_car_pp] += p_a
                end
                car_routes = fill(s.route, length(car_ps))
                continue
            end
            # interpolate car_p in discrete space
            car_v_space = get_car_vspace(mdp.env, mdp.vel_res)
            itp_car_ps, itp_car_weights = interpolate_state(mdp, car_p, car_v_space)
            for (j, car_pss) in enumerate(itp_car_ps)
                index_itp_state = find(x -> x==car_pss, car_ps)
                if !(itp_car_weights[j] ≈ 0.)
                    if isempty(index_itp_state)
                        push!(car_ps, car_pss)
                        push!(car_probs, itp_car_weights[j]*p_a)
                    else
                        car_probs[index_itp_state] += itp_car_weights[j]*p_a
                    end
                end
            end
            car_routes = fill(s.route, length(car_ps))
        end
        @assert length(car_probs) == length(car_ps)
        @assert length(car_routes) == length(car_ps)
        normalize!(car_probs, 1)
    end
    return car_ps, car_routes, car_probs
end

function set_car_model!(mdp::PedCarMDP, s::PedCarMDPState, a::PedCarMDPAction)
    car = s.car
    lane = get_lane(mdp.env.roadway, car)
    route = [mdp.env.roadway[tag] for tag in find_route(mdp.env, s.route)]
    intersection_entrances = get_start_lanes(mdp.env.roadway)
    if !(route[1] ∈ intersection_entrances)
        intersection = Lane[]
        intersection_exits = Lane[]
    else
        intersection_exits = get_exit_lanes(mdp.env.roadway)
        intersection=Lane[route[1], route[2]]
    end
    navigator = RouteFollowingIDM(route=route, a_max=2., σ=0.5)
    intersection_driver = StopIntersectionDriver(navigator= navigator,
                                                intersection=intersection,
                                                intersection_entrances = intersection_entrances,
                                                intersection_exits = intersection_exits,
                                                stop_delta=maximum(mdp.env.params.crosswalk_width),
                                                accel_tol=0.,
                                                priorities = mdp.env.priorities)
    crosswalk_drivers = Vector{CrosswalkDriver}(length(mdp.env.crosswalks))
    for i=1:length(mdp.env.crosswalks)
        cw_conflict_lanes = get_conflict_lanes(mdp.env.crosswalks[i], mdp.env.roadway)
        crosswalk_drivers[i] = CrosswalkDriver(navigator = navigator,
                                crosswalk = mdp.env.crosswalks[i],
                                conflict_lanes = cw_conflict_lanes,
                                intersection_entrances = intersection_entrances,
                                yield=!isempty(intersect(cw_conflict_lanes, route))
                                )
        # println(" yield to cw ", i, " ", crosswalk_drivers[i].yield)
    end
    mdp.car_model = UrbanDriver(navigator=navigator,
                                            intersection_driver=intersection_driver,
                                            crosswalk_drivers=crosswalk_drivers
                                            )
    # update model states 
    scene = Scene()
    push!(scene, Vehicle(s.ego, mdp.ego_type, EGO_ID))
    push!(scene, Vehicle(s.car, mdp.car_type, CAR_ID))
    push!(scene, Vehicle(s.ped, mdp.ped_type, PED_ID))
    # fill in hiddent state
    observe!(mdp.car_model, scene, mdp.env.roadway, CAR_ID)
    # set the decision
    observe!(mdp.car_model, scene, mdp.env.roadway, CAR_ID)
end
