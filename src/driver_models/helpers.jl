function get_stop_model(env::UrbanEnv, r::SVector{2, LaneTag})
    route = [env.roadway[tag] for tag in find_route(env, r)]
    intersection_entrances = get_start_lanes(env.roadway)
    if !(route[1] ∈ intersection_entrances)
        intersection = Lane[]
        intersection_exits = Lane[]
    else
        intersection_exits = get_exit_lanes(env.roadway)
        intersection=Lane[route[1], route[2]]
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
        intersection = Lane[]
        intersection_exits = Lane[]
    else
        intersection_exits = get_exit_lanes(env.roadway)
        intersection=Lane[route[1], route[2]]
    end
    navigator = RouteFollowingIDM(route=route, a_max=1., σ=1., v_des=6.0)
    intersection_driver = TTCIntersectionDriver(navigator = navigator,
                                            intersection = intersection,
                                            intersection_pos = VecSE2(env.params.inter_x,
                                                                      env.params.inter_y),
                                            stop_delta = maximum(env.params.crosswalk_width),
                                            accel_tol = 0.,
                                            priorities = env.priorities,
                                            ttc_threshold = (env.params.x_max - env.params.inter_x)/env.params.speed_limit
                                            )
    crosswalk_drivers = Vector{CrosswalkDriver}(undef, length(env.crosswalks))
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