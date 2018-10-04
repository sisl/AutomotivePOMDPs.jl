# A bunch of discretization helpers

# discretize velocity levels:
function get_car_vspace(env::UrbanEnv, v_res::Float64)
    return 0.:v_res:env.params.speed_limit
end

function get_ped_vspace(env::UrbanEnv, v_res::Float64)
    return 0:v_res:2.0
end

# discretize longitudinal position
function get_discretized_lane(tag::LaneTag, roadway::Roadway, pos_res::Float64)
    lane = roadway[tag]
    return 0.:pos_res:get_end(lane)
end

# enumerate ego state space
function get_ego_route(env::UrbanEnv)
    # TODO make it dependent on the topology
    return (LaneTag(6,1), LaneTag(13, 1), LaneTag(14,1), LaneTag(2,1))
end

function get_ego_states(env::UrbanEnv, pos_res::Float64, v_res::Float64)
    states_vec = VehicleState[]
    for lane in get_ego_route(env)
        discrete_lane = get_discretized_lane(lane, env.roadway, pos_res)
        v_space = get_car_vspace(env, v_res)
        for v in v_space
            for s in discrete_lane
                ego = VehicleState(Frenet(env.roadway[lane], s), env.roadway, v)
                push!(states_vec, ego)
            end
        end
    end
    return states_vec
end

# enumerate pedestrian state space
function get_ped_lanes(env::UrbanEnv)
    # TODO do not hard code it, get it form the environment structure
    return (LaneTag(17, 1), LaneTag(18, 1), LaneTag(19, 1))
    # return (LaneTag(17, 1), LaneTag(18, 1))
end

function get_ped_states(env::UrbanEnv, pos_res::Float64, v_res::Float64)
    states_vec = VehicleState[]
    for lane in get_ped_lanes(env)
        discrete_lane = get_discretized_lane(lane, env.roadway, pos_res)
        v_space = get_ped_vspace(env, v_res)
        for v in v_space
            for s in discrete_lane
                for phi in (0., float(pi))
                    ped = VehicleState(Frenet(env.roadway[lane], s, 0., phi), env.ped_roadway, v)
                    push!(states_vec, ped)
                end
            end
        end
    end
    return states_vec
end

# enumerate other car state space
function get_car_lanes(env::UrbanEnv)
    lanes = get_lanes(env.roadway)
    car_lanes = LaneTag[]
    no_go_lanes = [LaneTag(6, 1), LaneTag(15, 1), LaneTag(16, 1), LaneTag(14, 1), LaneTag(13, 1)]
    for lane in lanes
        if !(lane.tag ∈ no_go_lanes ) && !(lane.tag ∈ get_ped_lanes(env))
            push!(car_lanes, lane.tag)
        end
    end
    return car_lanes
end

# for route in routes
# for lane in route
# for s in lane ...

function get_car_states(env::UrbanEnv, pos_res::Float64, v_res::Float64)
    states_vec = VehicleState[]
    for lane in get_car_lanes(env)
        discrete_lane = get_discretized_lane(lane, env.roadway, pos_res)
        v_space = get_car_vspace(env, v_res)
        for v in v_space
            for s in discrete_lane
                car = VehicleState(Frenet(env.roadway[lane], s), env.roadway, v)
                push!(states_vec, car)
            end
        end
    end
    return states_vec
end

function get_car_states(env::UrbanEnv, route::Vector{LaneTag}, pos_res::Float64, v_res::Float64)
    states_vec = VehicleState[]
    for lane_tag in route
        discrete_lane = get_discretized_lane(lane_tag, env.roadway, pos_res)
        v_space = get_car_vspace(env, v_res)
        for v in v_space
            for s in discrete_lane
                car = VehicleState(Frenet(env.roadway[lane_tag], s), env.roadway, v)
                push!(states_vec, car)
            end
        end
    end
    return states_vec
end


# count state spaces
function n_ego_states(env::UrbanEnv, pos_res::Float64, v_res::Float64)
    N = 0
    nv = length(get_car_vspace(env, v_res))
    for lane in get_ego_route(env)
        N += nv * length(get_discretized_lane(lane, env.roadway, pos_res))
    end
    return N
end

function n_ped_states(env::UrbanEnv, pos_res::Float64, v_res::Float64)
    N = 0
    nv = length(get_ped_vspace(env, v_res))
    n_headings = 2
    for lane in get_ped_lanes(env)
        N += nv * n_headings * length(get_discretized_lane(lane, env.roadway, pos_res))
    end
    return N
end

function n_car_states(env::UrbanEnv, pos_res::Float64, v_res::Float64)
    N = 0
    nv = length(get_car_vspace(env, v_res))
    for lane in get_car_lanes(env)
        N += nv * length(get_discretized_lane(lane, env.roadway, pos_res))
    end
    return N
end

function n_car_states(env::UrbanEnv, route::Vector{LaneTag}, pos_res::Float64, v_res::Float64)
    N = 0
    nv = length(get_car_vspace(env, v_res))
    for lane_tag in route
        N += nv * length(get_discretized_lane(lane_tag, env.roadway, pos_res))
    end
    return N
end

# state indexing!
function ego_state_index(env::UrbanEnv, ego::VehicleState, pos_res::Float64, v_res::Float64)
    # find lane index
    lanes = get_ego_route(env)
    lane = get_lane(env.roadway, ego)
    li = findfirst(isequal(lane.tag), lanes) #XXX possibly inefficient
    # find position index
    s_space = get_discretized_lane(lane.tag, env.roadway, pos_res)
    si = find_range_index(s_space, ego.posF.s)
    # find velocity index
    v_space = get_car_vspace(env, v_res)
    size_v = length(v_space)
    v = ego.v
    vi = find_range_index(v_space, v)
    # use linear/Cartesian magic
    egoi = LinearIndices((length(s_space), length(v_space)))[si, vi]
    # Lanes have different lengths
    for i=2:li
        size_s = length(get_discretized_lane(lanes[i-1], env.roadway, pos_res))
        egoi += size_s*size_v
    end
    return egoi
end

function ind2ego(env::UrbanEnv, ei::Int64, pos_res::Float64, v_res::Float64)
    lanes = get_ego_route(env)
    v_space = get_car_vspace(env, v_res)
    size_v = length(v_space)
    # find lane first 
    ns = 0 
    lane_ind = 0
    lane_shift = 0 
    for (i, lane) in enumerate(lanes)
        lane_shift = ns
        ns += size_v*length(get_discretized_lane(lane, env.roadway, pos_res))
        if ns >= ei 
            lane_ind = i
            break
        end
    end
    # find s, v 
    lane = env.roadway[lanes[lane_ind]]
    ei_ = ei - lane_shift
    size_s = length(get_discretized_lane(lane.tag, env.roadway, pos_res))
    si, vi = Tuple(CartesianIndices((size_s, size_v))[ei_])
    s = get_discretized_lane(lane.tag, env.roadway, pos_res)[si]
    v = v_space[vi]
    posF = Frenet(lane, s)
    # posG = VecSE2{Float64}(NaN, NaN, NaN)
    posG = get_posG(posF, env.roadway)
    return VehicleState(posG, posF, v)
end


function car_state_index(env::UrbanEnv, car::VehicleState, pos_res::Float64, v_res::Float64)
    # find lane index
    lanes = get_car_lanes(env)
    lane = get_lane(env.roadway, car)
    li = findfirst(isequal(lane.tag), lanes) #XXX possibly inefficient
    # find position index
    s_space = get_discretized_lane(lane.tag, env.roadway, pos_res)
    si = find_range_index(s_space, car.posF.s)
    # find velocity index
    v_space = get_car_vspace(env, v_res)
    size_v = length(v_space)
    v = car.v
    vi = find_range_index(v_space, v)
    # use linear/Cartesian magic
    cari = LinearIndices((length(s_space), length(v_space)))[si, vi]
    # Lanes have different lengths
    for i=2:li
        size_s = length(get_discretized_lane(lanes[i-1], env.roadway, pos_res))
        cari += size_s*size_v
    end
    return cari
end

function car_state_index(env::UrbanEnv, car::VehicleState, route::Vector{LaneTag}, pos_res::Float64, v_res::Float64)
    lane = get_lane(env.roadway, car)
    li = findfirst(isequal(lane.tag), route)
    # position index
    # find position index
    s_space = get_discretized_lane(lane.tag, env.roadway, pos_res)
    si = find_range_index(s_space, car.posF.s)
    # find velocity index
    v_space = get_car_vspace(env, v_res)
    size_v = length(v_space)
    v = car.v
    vi = find_range_index(v_space, v)
    # use linear/Cartesian magic
    cari = LinearIndices((length(s_space), length(v_space)))[si, vi]
    # Lanes have different lengths
    for i=2:li
        size_s = length(get_discretized_lane(route[i-1], env.roadway, pos_res))
        cari += size_s*size_v
    end
    return cari
end

function ind2car(env::UrbanEnv, ci::Int64, route::Vector{LaneTag}, pos_res::Float64, v_res::Float64) 
    v_space = get_car_vspace(env, v_res)
    size_v = length(v_space)
    # find lane first
    ns = 0 
    lane_ind = 0
    lane_shift = 0 
    for (i, lane) in enumerate(route)
        lane_shift = ns
        ns += size_v*length(get_discretized_lane(lane, env.roadway, pos_res))
        if ns >= ci 
            lane_ind = i
            break
        end
    end
    # find s, v 
    lane = env.roadway[route[lane_ind]]
    ci_ = ci - lane_shift
    size_s = length(get_discretized_lane(lane.tag, env.roadway, pos_res))
    si, vi = Tuple(CartesianIndices((size_s, size_v))[ci_])
    s = get_discretized_lane(lane.tag, env.roadway, pos_res)[si]
    v = v_space[vi]
    posF = Frenet(lane, s)
    # posG = VecSE2{Float64}(NaN, NaN, NaN)
    posG = get_posG(posF, env.roadway)
    return VehicleState(posG, posF, v)

end

function ped_state_index(env::UrbanEnv, ped::VehicleState, pos_res::Float64, v_res::Float64)
    # find lane index
    lanes = get_ped_lanes(env)
    lane = get_lane(env.roadway, ped)
    li = findfirst(isequal(lane.tag), lanes) #XXX possibly inefficient
    # find position index
    s_space = get_discretized_lane(lane.tag, env.roadway, pos_res)
    si = find_range_index(s_space, ped.posF.s)
    # heading 
    n_headings = 2
    phi_i = ped.posF.ϕ == 0. ? 1 : 2
    # find velocity index
    v_space = get_ped_vspace(env, v_res)
    size_v = length(v_space)
    v = ped.v
    vi = find_range_index(v_space, v)
    # use linear/Cartesian magic
    pedi = LinearIndices((n_headings, length(s_space), length(v_space)))[phi_i, si, vi]
    # Lanes have different lengths
    for i=2:li
        size_s = length(get_discretized_lane(lanes[i-1], env.roadway, pos_res))
        pedi += size_s*size_v*n_headings
    end
    return pedi
end

function ind2ped(env::UrbanEnv, pedi::Int64, pos_res::Float64, v_res::Float64)
    v_space = get_ped_vspace(env, v_res)
    size_v = length(v_space)
    lanes = get_ped_lanes(env)
    # find lane first 
    ns = 0 
    lane_ind = 0
    lane_shift = 0 
    for (i, lane) in enumerate(lanes)
        lane_shift = ns
        ns += 2*size_v*length(get_discretized_lane(lane, env.roadway, pos_res))
        if ns >= pedi 
            lane_ind = i
            break
        end
    end
    # find s, v 
    lane = env.roadway[lanes[lane_ind]]
    pi_ = pedi - lane_shift
    size_s = length(get_discretized_lane(lane.tag, env.roadway, pos_res))
    phii, si, vi = Tuple(CartesianIndices((2, size_s, size_v))[pi_])
    s = get_discretized_lane(lane.tag, env.roadway, pos_res)[si]
    v = v_space[vi]
    phi = phii == 1 ? 0. : float(pi)
    posF = Frenet(lane, s, 0., phi)
    # posG = VecSE2{Float64}(NaN, NaN, NaN)
    posG = get_posG(posF, env.roadway)
    return VehicleState(posG, posF, v)
end

function find_range_index(r::AbstractRange, s::Float64)
    return clamp(round(Int, ((s - first(r))/step(r) + 1)), 1, length(r))
end

function get_car_routes(env::UrbanEnv)
    return [[LaneTag(1,1), LaneTag(7,1), LaneTag(2,1)],
              [LaneTag(1,1), LaneTag(9,1), LaneTag(10, 1), LaneTag(5, 1)],
              [LaneTag(3,1), LaneTag(8, 1), LaneTag(4, 1)],
              [LaneTag(3, 1), LaneTag(11, 1), LaneTag(12, 1), LaneTag(5, 1)]]
end

function get_possible_routes(lane::Lane, env::UrbanEnv)
    possible_routes = Int64[]
    sizehint!(possible_routes, 2)
    routes = get_car_routes(env)
    for i=1:length(routes)
        route = routes[i]
        if in(lane, route)
            push!(possible_routes, i)
        end
    end
    return possible_routes
end


# return the first full route in the environment which end matches with route
function find_route(env::UrbanEnv, route::SVector{2, LaneTag})
    routes = get_car_routes(env)
    for full_route in routes 
        if full_route[end] == route[end] && full_route[1] == route[1]
            return full_route
        end
    end
    # should never reach this point 
    @assert false "RouteNotFound for route=$route"
    return routes[1]
end

function find_route(env::UrbanEnv, route::Vector{Lane})
    routes = get_car_routes(env)
    for full_route in routes
        if full_route[end] == route[end].tag && full_route[1] == route[1].tag ||
           full_route[end] == route[end].tag && length(route) == 1 ||
           route[1].tag ∈ full_route && full_route[end] == route[end].tag
            return full_route
        end
    end
    @assert false "RouteNotFound for route=$route"
    return routes[1]
end

function car_projection(env::UrbanEnv, pos_res::Float64, vel_res::Float64)
    proj_dict = Dict{Tuple{Float64, Float64, LaneTag}, VehicleState}()
    for lane in get_all_lanes(env.roadway)
        if !is_crosswalk(lane)
            dlane = get_discretized_lane(lane.tag, env.roadway, pos_res)
            vspace = get_car_vspace(env, vel_res)
            for s in dlane 
                for v in vspace 
                    proj_dict[(s, v, lane.tag)] = VehicleState(Frenet(lane, s), env.roadway, v)
                end
            end
        end
    end
    return proj_dict
end

function ped_projection(env::UrbanEnv, pos_res::Float64, vel_res::Float64)
    proj_dict = Dict{Tuple{Float64, Float64, Float64, LaneTag}, VehicleState}()
    for lane in get_all_lanes(env.ped_roadway)
        vspace = get_ped_vspace(env, vel_res)
        dlane = get_discretized_lane(lane.tag, env.roadway, pos_res)
        for s in dlane 
            for v in vspace 
                proj_dict[(s, v, 0., lane.tag)] = VehicleState(Frenet(lane, s, 0., 0.), env.ped_roadway, v)
                proj_dict[(s, v, float(pi), lane.tag)] = VehicleState(Frenet(lane, s, 0., float(pi)), env.ped_roadway, v)
            end
        end
    end
    return proj_dict
end