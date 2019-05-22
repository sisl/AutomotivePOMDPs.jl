# create a dictionary with all the possible models
function get_car_models(env::UrbanEnv, get_model::Function)
    d = Dict{SVector{2, LaneTag}, DriverModel}()

    # 
    r1 = SVector(LaneTag(1,1), LaneTag(2,1))
    d[r1] = get_model(env, r1)

    r2 = SVector(LaneTag(1,1), LaneTag(5,1))
    d[r2] = get_model(env, r2)

    r3 = SVector(LaneTag(3,1), LaneTag(4,1))
    d[r3] = get_model(env, r3)
    
    r4 = SVector(LaneTag(3,1), LaneTag(5,1)) 
    d[r4] = get_model(env, r4)

    return d
end

function get_stop_model(env::UrbanEnv, r::SVector{2, LaneTag})
    route = [env.roadway[tag] for tag in find_route(env, r)]
    intersection_entrances = get_start_lanes(env.roadway)
    if !(route[1] ∈ intersection_entrances)
        intersection = Lane{Float64}[]
        intersection_exits = Lane{Float64}[]
    else
        intersection_exits = get_exit_lanes(env.roadway)
        intersection=Lane{Float64}[route[1], route[2]]
    end
    navigator = RouteFollowingIDM(route=route, a_max=3., σ=1.)
    intersection_driver = StopIntersectionDriver(navigator= navigator,
                                                intersection=intersection,
                                                intersection_entrances = intersection_entrances,
                                                intersection_exits = intersection_exits,
                                                stop_delta=maximum(env.params.crosswalk_width),
                                                accel_tol=0.,
                                                priorities = env.priorities)
    crosswalk_drivers = Vector{CrosswalkDriver}(length(env.crosswalks))
    for i=1:length(env.crosswalks)
        cw_conflict_lanes = get_conflict_lanes(env.crosswalks[i], env.roadway)
        crosswalk_drivers[i] = CrosswalkDriver(navigator = navigator,
                                crosswalk = env.crosswalks[i],
                                conflict_lanes = cw_conflict_lanes,
                                intersection_entrances = intersection_entrances,
                                yield=!isempty(intersect(cw_conflict_lanes, route))
                                )
        # println(" yield to cw ", i, " ", crosswalk_drivers[i].yield)
    end
    return UrbanDriver(navigator=navigator,
                    intersection_driver=intersection_driver,
                    crosswalk_drivers=crosswalk_drivers
                    )
end

function get_ttc_model(env::UrbanEnv, r::SVector{2, LaneTag})
    route = [env.roadway[tag] for tag in find_route(env, r)]
    intersection_entrances = get_start_lanes(env.roadway)
    if !(route[1] ∈ intersection_entrances)
        intersection = Lane{Float64}[]
        intersection_exits = Lane{Float64}[]
    else
        intersection_exits = get_exit_lanes(env.roadway)
        intersection=Lane{Float64}[route[1], route[2]]
    end
    navigator = RouteFollowingIDM(route=route, a_max=2., σ=1.)
    intersection_driver = TTCIntersectionDriver(navigator = navigator,
                                            intersection = intersection,
                                            intersection_pos = VecSE2(mdp.env.params.inter_x,
                                                                        mdp.env.params.inter_y),
                                            stop_delta = maximum(mdp.env.params.crosswalk_width),
                                            accel_tol = 0.,
                                            priorities = mdp.env.priorities,
                                            ttc_threshold = (mdp.env.params.x_max - mdp.env.params.inter_x)/mdp.env.params.speed_limit
                                            )
    crosswalk_drivers = Vector{CrosswalkDriver}(length(env.crosswalks))
    for i=1:length(env.crosswalks)
        cw_conflict_lanes = get_conflict_lanes(env.crosswalks[i], env.roadway)
        crosswalk_drivers[i] = CrosswalkDriver(navigator = navigator,
                                crosswalk = env.crosswalks[i],
                                conflict_lanes = cw_conflict_lanes,
                                intersection_entrances = intersection_entrances,
                                yield=!isempty(intersect(cw_conflict_lanes, route))
                                )
        # println(" yield to cw ", i, " ", crosswalk_drivers[i].yield)
    end
    return UrbanDriver(navigator=navigator,
                    intersection_driver=intersection_driver,
                    crosswalk_drivers=crosswalk_drivers
                    )
end

function AutomotiveDrivingModels.observe!(model::UrbanDriver, 
                                          ego::VehicleState, 
                                          car::VehicleState, 
                                          ped::VehicleState,
                                          roadway::Roadway)
    AutomotiveDrivingModels.observe!(model.navigator, ego, car, roadway)
    
    for driver in model.crosswalk_drivers
        AutomotiveDrivingModels.observe!(driver, ego, car, ped, roadway)
    end
    a_lon_crosswalks = minimum([driver.a.a_lon for driver in model.crosswalk_drivers])
    a_lon = min(a_lon_crosswalks, model.navigator.a)

    model.a = LonAccelDirection(a_lon, model.navigator.dir)
    return model.a
end


function AutomotiveDrivingModels.observe!(model::TTCIntersectionDriver, 
                                          ego::VehicleState, 
                                          car::VehicleState, 
                                          roadway::Roadway)

    model.priority = ttc_check(model, car, ego, roadway)
    
end

function AutomotiveDrivingModels.observe!(model::CrosswalkDriver, 
                                          ego::VehicleState, 
                                          car::VehicleState,
                                          ped::VehicleState,
                                          roadway::Roadway)
    a_lon =0.
    a_lon_idm = model.navigator.a_max
    if !model.yield 
        a_lon = a_lon_idm
    else
        model.priority = !is_crossing(ped, model.crosswalk, model.conflict_lanes, roadway) || 
        has_passed(model, car, roadway)
        if !model.priority 
            dist_to_cw = get_distance_to_crosswalk(model, car, roadway, -model.stop_delta)
            a_lon = min(a_lon_idm, stop_at_dist(model, car, dist_to_cw))
            # println("dist to cw $dist_to_cw, crosswalk $(model.crosswalk.tag), $(get_lane(roadway, car) ∈ model.intersection_entrances)")
            # println("priority $(model.priority), crossing $(is_crossing(ped, model.crosswalk, model.conflict_lanes, roadway)), a_lon $a_lon, v $(car.v)")
        else
            a_lon = a_lon_idm # just idm
        end
    end
    model.a = LonAccelDirection(a_lon, model.navigator.dir)
end

function AutomotiveDrivingModels.observe!(model::RouteFollowingIDM, 
                                          ego::VehicleState, 
                                          car::VehicleState,
                                          roadway::Roadway)
    Δv = model.v_des - car.v
    acc = Δv*model.k_spd
    model.a = clamp(acc, -model.d_max, model.a_max)
    # fore = is_neighbor_fore_along_lane(car, ego, roadway)
    # v_car = car.v
    # if fore.ind != nothing
    #     headway, v_oth = fore.Δs, car.v
    # else
    #     headway, v_oth = NaN, NaN
    # end
    # track_longitudinal!(model, v_car, v_oth, headway)
    cur_lane = get_lane(roadway, car)
    set_direction!(model, cur_lane, roadway)    
    return model
end


function is_crossing(ped::VehicleState, crosswalk::Lane{T}, conflict_lanes::Vector{Lane{T}}, roadway::Roadway) where T<:Real
    ped_lane = get_lane(roadway, ped)
    if ped_lane.tag != crosswalk.tag
        return false 
    end
    if 4.0 <= ped.posF.s <= 10. 
        return true
    end
    # at this point, the pedestrian is not on the road
    # check if the pedestrian is going to cross or not
    if AutomotivePOMDPs.direction_from_center(ped, crosswalk) > 0. && get_lane(roadway, ped).tag == crosswalk.tag
        return true
    end
    return false
end

function ttc_check(model::TTCIntersectionDriver, ego::VehicleState, car::VehicleState, roadway::Roadway)
    posF = car.state.posF
    int_x, int_y, int_θ = model.intersection_pos
    lane = get_lane(roadway, car)
    int_proj = Frenet(model.intersection_pos, lane, roadway)
    if normsquared(VecE2(model.intersection_pos - car.state.posG)) < inter_width^2 # vehicle is in the middle
        ttc = 0.
    else
        ttc = (int_proj.s - posF.s)/car.state.v
    end

    if 0 <= ttc < model.ttc_threshold
        return false
    else
        return true
    end

end

function is_neighbor_fore_along_lane(ego::VehicleState, car::VehicleState, roadway::Roadway, ego_def::VehicleDef = VehicleDef())
    tag_start = ego.posF.roadind.tag
    s_base = ego.posF.s + ego_def.length/2*cos(ego.posF.ϕ)
    tag_start = ego.posF.roadind.tag
    targetpoint_primary = VehicleTargetPointRear()
    targetpoint_valid = VehicleTargetPointFront()

    is_neighbor_fore_along_lane(ego, car, roadway, tag_start, s_base)
end


function is_neighbor_fore_along_lane(
    ego::VehicleState,
    veh::VehicleState,
    roadway::Roadway,
    tag_start::LaneTag,
    s_base::Float64;
    max_distance_fore::Float64 = 250.0, # max distance to search forward [m]
    )

    best_ind = nothing
    best_dist = max_distance_fore
    tag_target = tag_start
    lane = roadway[tag_target]

    dist_searched = 0.0
    while dist_searched < max_distance_fore

        s_adjust = NaN
        if veh.posF.roadind.tag == tag_target
            s_adjust = 0.0

        elseif is_between_segments_hi(veh.posF.roadind.ind, lane.curve) &&
                is_in_entrances(roadway[tag_target], veh.posF.roadind.tag)

            distance_between_lanes = norm(VecE2(roadway[tag_target].curve[1].pos - roadway[veh.posF.roadind.tag].curve[end].pos))
            s_adjust = -(roadway[veh.posF.roadind.tag].curve[end].s + distance_between_lanes)

        elseif is_between_segments_lo(veh.posF.roadind.ind) &&
                is_in_exits(roadway[tag_target], veh.posF.roadind.tag)

            distance_between_lanes = norm(VecE2(roadway[tag_target].curve[end].pos - roadway[veh.posF.roadind.tag].curve[1].pos))
            s_adjust = roadway[tag_target].curve[end].s + distance_between_lanes
        end

        if !isnan(s_adjust)
            s_valid = veh.posF.s + VehicleDef().length/2*cos(veh.posF.ϕ) + s_adjust
            dist_valid = s_valid - s_base + dist_searched
            if dist_valid ≥ 0.0
                s_primary = veh.posF.s - VehicleDef().length/2*cos(veh.posF.ϕ) + s_adjust
                dist = s_primary - s_base + dist_searched
                if dist < best_dist
                    best_dist = dist
                    best_ind = 1
                end
            end
        end
        if best_ind != nothing
            break
        end

        if !has_next(lane) ||
           (tag_target == tag_start && dist_searched != 0.0) # exit after visiting this lane a 2nd time
            break
        end
        
        dist_searched += (lane.curve[end].s - s_base)
        s_base = -norm(VecE2(lane.curve[end].pos - next_lane_point(lane, roadway).pos)) # negative distance between lanes
        tag_target = next_lane(lane, roadway).tag
    end
    NeighborLongitudinalResult(best_ind, best_dist)
end