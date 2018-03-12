# define the generative model for the urban pomdp environment

### REWARD MODEL ##################################################################################

function POMDPs.reward(pomdp::UrbanPOMDP, s::UrbanState, a::UrbanAction, sp::UrbanState)
    r = 0.
    ego = sp[findfirst(sp, EGO_ID)]
    if is_crash(sp)
        r += pomdp.collision_cost
    end
    if ego.state.posF.s >= get_end(pomdp.env.roadway[pomdp.ego_goal]) &&
       get_lane(pomdp.env.roadway, ego).tag == pomdp.ego_goal
        r += pomdp.goal_reward
    elseif a.acc > 0.
        r += pomdp.action_cost
    else
        r += pomdp.action_cost
    end
    return r
end


### TERMINAL STATES ###############################################################################

function POMDPs.isterminal(pomdp::UrbanPOMDP, s::UrbanState)
    ego = s[findfirst(s, EGO_ID)]
    return is_crash(s) || (ego.state.posF.s >= get_end(pomdp.env.roadway[pomdp.ego_goal]) &&
                           get_lane(pomdp.env.roadway, ego).tag == pomdp.ego_goal)
end


### TRANSITION MODEL ##############################################################################

function POMDPs.generate_s(pomdp::UrbanPOMDP, s::UrbanState, a::UrbanAction, rng::AbstractRNG)
    actions = Array{Any}(length(s))
    pomdp.models[1].a = a
    sp = deepcopy(s) #XXX bad
    if rand(rng) < pomdp.car_birth && n_cars(s) < pomdp.max_cars + 1
        new_car = initial_car(pomdp, sp, rng)
        if can_push(pomdp.env, sp, new_car)
            lane = get_lane(pomdp.env.roadway, new_car)
            route = random_route(rng, pomdp.env.roadway, lane)
            intersection_entrances = get_start_lanes(pomdp.env.roadway)
            if !(route[1] ∈ intersection_entrances)
                intersection = Lane[]
                intersection_exits = Lane[]
            else
                intersection_exits = get_exit_lanes(pomdp.env.roadway)
                intersection=Lane[route[1], route[2]]
            end
            navigator = RouteFollowingIDM(route=route)
            intersection_driver = TTCIntersectionDriver(navigator = navigator,
                                                        intersection = intersection,
                                                        intersection_pos = VecSE2(pomdp.env.params.inter_x,
                                                                                  pomdp.env.params.inter_y),
                                                        stop_delta = maximum(pomdp.env.params.crosswalk_width),
                                                        accel_tol = 0.,
                                                        priorities = pomdp.env.priorities,
                                                        ttc_threshold = (pomdp.env.params.x_max - pomdp.env.params.inter_x)/pomdp.env.params.speed_limit
                                                        )
            # intersection_driver = StopIntersectionDriver(navigator= navigator,
            #                                          intersection=intersection,
            #                                          intersection_entrances = intersection_entrances,
            #                                          intersection_exits = intersection_exits,
            #                                          stop_delta=maximum(pomdp.env.params.crosswalk_width),
            #                                          accel_tol=0.,
            #                                          priorities = pomdp.env.priorities)
            crosswalk_drivers = Vector{CrosswalkDriver}(length(pomdp.env.crosswalks))
            for i=1:length(pomdp.env.crosswalks)
                cw_conflict_lanes = get_conflict_lanes(pomdp.env.crosswalks[i], pomdp.env.roadway)
                crosswalk_drivers[i] = CrosswalkDriver(navigator = navigator,
                                         crosswalk = pomdp.env.crosswalks[i],
                                         conflict_lanes = cw_conflict_lanes,
                                         intersection_entrances = intersection_entrances,
                                         yield=!isempty(intersect(cw_conflict_lanes, route))
                                         )
            end
            pomdp.models[new_car.id] = UrbanDriver(navigator=navigator,
                                                    intersection_driver=intersection_driver,
                                                    crosswalk_drivers=crosswalk_drivers
                                                    )
            push!(sp, new_car)
        end
        if rand(rng) < pomdp.ped_birth && n_pedestrians(s) < pomdp.max_peds
            # println("Spawning new pedestrians")
            new_ped = initial_pedestrian(pomdp, sp, rng)
            pomdp.models[new_ped.id] = ConstantPedestrian(dt = pomdp.ΔT)#TODO parameterized
            push!(sp, new_ped)
        end
        actions = Array{Any}(length(sp))
    end
    get_actions!(actions, sp, pomdp.env.roadway, pomdp.models)
    tick!(sp, pomdp.env.roadway, actions, pomdp.ΔT)
    clean_scene!(pomdp.env, sp)
    return sp
end


#TODO move to helpers

function AutomotiveDrivingModels.propagate(veh::Vehicle, a::UrbanAction, roadway::Roadway, Δt::Float64)
    acc = LonAccelDirection(a.acc, 4) #TODO Parameterize
    return propagate(veh, acc, roadway, Δt)
end


### INITIAL STATES ################################################################################

function POMDPs.initial_state(pomdp::UrbanPOMDP, rng::AbstractRNG, no_ego::Bool=false)
    if pomdp.obstacles
        sample_obstacles!(pomdp.env, pomdp.obs_dist, rng)
    end
    scene = Scene()
    if !no_ego
        ego = initial_ego(pomdp, rng)
        push!(scene, ego)
    end

    # add cars
    for i=1:pomdp.max_cars
        if rand(rng) < pomdp.car_birth
            new_car = initial_car(pomdp, scene, rng, true)
            if can_push(pomdp.env, scene, new_car)
                push!(scene, new_car)
                lane = get_lane(pomdp.env.roadway, new_car)
                route = random_route(rng, pomdp.env.roadway, lane)
                intersection_entrances = get_start_lanes(pomdp.env.roadway)
                if !(route[1] ∈ intersection_entrances)
                    intersection = Lane[]
                    intersection_exits = Lane[]
                else
                    intersection_exits = get_exit_lanes(pomdp.env.roadway)
                    intersection=Lane[route[1], route[2]]
                end
                navigator = RouteFollowingIDM(route=route)
                intersection_driver = StopIntersectionDriver(navigator= navigator,
                                                         intersection=intersection,
                                                         intersection_entrances = intersection_entrances,
                                                         intersection_exits = intersection_exits,
                                                         stop_delta=maximum(pomdp.env.params.crosswalk_width),
                                                         accel_tol=0.,
                                                         priorities = pomdp.env.priorities)
                crosswalk_drivers = Vector{CrosswalkDriver}(length(pomdp.env.crosswalks))
                for i=1:length(pomdp.env.crosswalks)
                    cw_conflict_lanes = get_conflict_lanes(pomdp.env.crosswalks[i], pomdp.env.roadway)
                    crosswalk_drivers[i] = CrosswalkDriver(navigator = navigator,
                                            crosswalk = pomdp.env.crosswalks[i],
                                            conflict_lanes = cw_conflict_lanes,
                                            intersection_entrances = intersection_entrances,
                                            yield=!isempty(intersect(cw_conflict_lanes, route))
                                            )
                end
                pomdp.models[new_car.id] = UrbanDriver(navigator=navigator,
                                                       intersection_driver=intersection_driver,
                                                       crosswalk_drivers=crosswalk_drivers
                                                       )
            end
            # pomdp.models[new_car.id] = RouteFollowingIDM(route = random_route(rng,
            #                                                                   pomdp.env.roadway,
            #                                                                   get_lane(pomdp.env.roadway, new_car)))
            clean_scene!(pomdp.env, scene)
        end
    end

    # add pedestrians
    for i=1:pomdp.max_peds
        if rand(rng) < pomdp.ped_birth # pedestrian appear
            new_ped = initial_pedestrian(pomdp, scene, rng, true)
            pomdp.models[new_ped.id] = ConstantPedestrian(dt = pomdp.ΔT)
            push!(scene, new_ped)
        end
    end
    clean_scene!(pomdp.env, scene)
    return scene
end

function initial_car(pomdp::UrbanPOMDP, scene::Scene, rng::AbstractRNG, first_scene::Bool = false)
    env = pomdp.env

    #velocity
    v0 = rand(rng, 4.0:1.0:8.0) #XXX parameterize!
    s0 = 0.
    if first_scene
        lane = rand(rng, get_lanes(pomdp.env.roadway)) # could be precomputed
        s0 = rand(rng, 0.:0.5:get_end(lane))
    else
        lane = rand(rng, get_start_lanes(pomdp.env.roadway)) # could be precomputed
    end
    initial_posF = Frenet(lane, s0)
    initial_state = VehicleState(initial_posF, env.roadway, v0)

    id = next_car_id(pomdp, scene, rng)

    return Vehicle(initial_state, pomdp.car_type, id)
end


function initial_ego(pomdp::UrbanPOMDP, rng::AbstractRNG)
    posF = Frenet(get_ego_route(pomdp)[1], pomdp.ego_start)
    state = VehicleState(posF, pomdp.env.roadway, 0.)
    return Vehicle(state, pomdp.ego_type, EGO_ID)
end


"""
    initial_pedestrian(pomdp::UrbanPOMDP, scene::Scene, rng::AbstractRNG)
Create a new pedestrian entity at its initial state
"""
function initial_pedestrian(pomdp::UrbanPOMDP, scene::Scene, rng::AbstractRNG, first_scene::Bool = false)
    env = pomdp.env
    cw_ind = rand(rng, 1:length(env.params.crosswalk_pos))
    crosswalk_pos = env.params.crosswalk_pos[cw_ind]

    # position along the crosswalk
    t0 = rand(rng, Uniform(-env.params.crosswalk_width[cw_ind]/2, env.params.crosswalk_width[cw_ind]/2))
    s0 = rand(rng, [0., get_end(env.crosswalks[cw_ind])])
    ϕ0 = float(π)
    if s0 == 0.
        ϕ0 = 0.
    end
    if first_scene
        s0 = rand(rng, Uniform(0., get_end(env.crosswalks[cw_ind])))
    end

    #velocity
    v0 = rand(rng, Uniform(0., env.params.ped_max_speed))
    posF = Frenet(env.crosswalks[cw_ind], s0, t0, ϕ0)

    ped_initial_state = VehicleState(posF, env.roadway, v0);

    id = next_ped_id(pomdp, scene, rng)

    return Vehicle(ped_initial_state, PEDESTRIAN_DEF, id)
end

function POMDPs.initial_state_distribution(pomdp::UrbanPOMDP)
    return GenerativeDist(pomdp)
end



### Observations ##################################################################################

# uncomment for vector representation
# TODO find a better way to implement this to switch more easily between representations
function POMDPs.generate_o(pomdp::UrbanPOMDP, s::Scene, a::UrbanAction, sp::Scene, rng::AbstractRNG)
    n_features = 4
    n_obstacles = 3
    pos_noise = pomdp.pos_obs_noise
    vel_noise = pomdp.vel_obs_noise
    o = zeros(n_features*(pomdp.max_cars + pomdp.max_peds + n_obstacles + 1))
    ego_veh =sp[findfirst(sp, EGO_ID)]
    ego = ego_veh.state
    o[1] = ego.posG.x
    o[2] = ego.posG.y
    o[3] = ego.posG.θ
    o[4] = ego.v
    pos_off = get_off_the_grid(pomdp)
    for i=2:pomdp.max_cars + pomdp.max_peds + n_obstacles + 1
        o[n_features*i - 3] = pos_off.posG.x
        o[n_features*i - 2] = pos_off.posG.y
        o[n_features*i - 1] = pos_off.posG.θ
        o[n_features*i] = 0.
    end
    for veh in sp
        if veh.id == EGO_ID
            continue
        end
        if veh.def.class == AgentClass.CAR
        # @assert veh.id <= pomdp.max_cars+1
            if is_observable_fixed(ego, veh.state, pomdp.env) && is_observable_dyna(ego_veh, veh, sp)
                o[n_features*veh.id - 3] = veh.state.posG.x + pos_noise*randn(rng)
                o[n_features*veh.id - 2] = veh.state.posG.y + pos_noise*randn(rng)
                o[n_features*veh.id - 1] = veh.state.posG.θ
                o[n_features*veh.id] = veh.state.v + vel_noise*randn(rng)
            end
        end
        if veh.def.class == AgentClass.PEDESTRIAN
            # println(" veh id ", veh.id)
            # println(" index " , n_features*(veh.id - 100 + pomdp.max_cars + 1))
            if is_observable_fixed(ego, veh.state, pomdp.env) && is_observable_dyna(ego_veh, veh, sp)
                o[n_features*(veh.id - 100 + pomdp.max_cars + 1) - 3] = veh.state.posG.x + pos_noise*randn(rng)
                o[n_features*(veh.id - 100 + pomdp.max_cars + 1) - 2] = veh.state.posG.y + pos_noise*randn(rng)
                o[n_features*(veh.id - 100 + pomdp.max_cars + 1) - 1] = veh.state.posG.θ
                o[n_features*(veh.id - 100 + pomdp.max_cars + 1)] = veh.state.v + vel_noise*randn(rng)
            end
        end
    end
    for (j,obs) in enumerate(pomdp.env.obstacles)
        o[n_features*(j + pomdp.max_cars + pomdp.max_peds + 1) - 3] = get_width(obs)
        o[n_features*(j + pomdp.max_cars + pomdp.max_peds + 1) - 2] = get_height(obs)
        o[n_features*(j + pomdp.max_cars + pomdp.max_peds + 1) - 1] = get_center(obs).x
        o[n_features*(j + pomdp.max_cars + pomdp.max_peds + 1)] = get_center(obs).y
    end
    return o
end

function POMDPs.generate_o(pomdp::UrbanPOMDP, s::UrbanState, rng::AbstractRNG)
    return generate_o(pomdp, s, UrbanAction(0.), s, rng::AbstractRNG)
end

function POMDPs.convert_o(::Type{Vector{Float64}}, o::UrbanObs, pomdp::UrbanPOMDP)
    # rescale
    n_features = 4
    n_obstacles = 3
    max_ego_dist = get_end(pomdp.env.roadway[pomdp.ego_goal])
    o[1] /= max_ego_dist
    o[2] /= max_ego_dist
    o[3] /= pi/2
    o[4] /= pomdp.env.params.speed_limit
    for i=2:pomdp.max_cars + pomdp.max_peds + 1
        o[n_features*i - 3] /= max_ego_dist
        o[n_features*i - 2] /= max_ego_dist
        o[n_features*i - 1] /= pi
        o[n_features*i] /= pomdp.env.params.speed_limit # XXX parameterized
    end
    for i=pomdp.max_cars + pomdp.max_peds + 1:pomdp.max_cars + pomdp.max_peds + 1 + n_obstacles
        o[n_features*i - 3] /= max_ego_dist
        o[n_features*i - 2] /= max_ego_dist
        o[n_features*i - 1] /= max_ego_dist
        o[n_features*i] /= max_ego_dist
    end

    return o
end

#### Helpers
# """
#     ego_state(pomdp:ICPOMDP, s::Float64, v::Float64)
# Helper that returns the ego car state given its position on the lane and the velocity
# """
# function ego_state(pomdp::UrbanPOMDP, s::Float64, v::Float64)
#     # ego_lane = pomdp.env.lane_map["ego_left"] #TODO parameterized
#     posF = Frenet(get_ego_route(pomdp)[1], s)
#     return VehicleState(posF, pomdp.env.roadway, v)
# end

"""
    car_state(pomdp::UrbanPOMDP, lane_id::String, s::Float64, v::Float64)
    car_state(pomdp::UrbanPOMDP, lane::Lane, s::Float64, v::Float64)
    car_state(pomdp::UrbanPOMDP, x::Float64, y::Float64, θ::Float64, v::Float64)
Helper that returns the state of a car given the lane and the position on the lane
"""
function car_state(pomdp::UrbanPOMDP, lane_id::String, s::Float64, v::Float64)
    lane = pomdp.env.lane_map[lane_id]
    posF = Frenet(lane, s)
    return VehicleState(posF, pomdp.env.roadway, v)
end
function car_state(pomdp::UrbanPOMDP, lane::Lane, s::Float64, v::Float64)
    posF = Frenet(lane, s)
    return VehicleState(posF, pomdp.env.roadway, v)
end
function car_state(pomdp::UrbanPOMDP, x::Float64, y::Float64, θ::Float64, v::Float64)
    posG = VecSE2(x, y, θ)
    return VehicleState(posG, pomdp.env.roadway, v)
end

"""
    get_off_the_grid(pomdp::UrbanPOMDP)
return the off the grid state
"""
function get_off_the_grid(pomdp::UrbanPOMDP)
    posG = pomdp.off_grid
    return VehicleState(posG, pomdp.env.roadway, 0.)
end

"""
Reconstruct a scene object from an UrbanPOMDP observation
"""
function obs_to_scene(pomdp::UrbanPOMDP, o::UrbanObs)
    # rebuild the scene from the observation
    scene = Scene()
    for id=1:pomdp.max_cars+1
        x, y, θ, v = o[4*id-3], o[4*id-2], o[4*id-1], o[4*id]
        if VecSE2(x, y, θ) == pomdp.off_grid
            continue
        end
        state = car_state(pomdp, x, y, θ, v)
        car_type = id == 1 ? pomdp.ego_type : pomdp.car_type
        veh = Vehicle(state, pomdp.car_type, id)
        push!(scene, veh)
    end
    return scene
end

"""
return the list of lanes the ego car should take
"""
function get_ego_route(pomdp::UrbanPOMDP)
    tags = [LaneTag(6,1), LaneTag(13, 1), LaneTag(2,pomdp.env.params.nlanes_main)]
    lanes = Array{Lane}(length(tags))
    for (i,tag) in enumerate(tags)
        lanes[i] = pomdp.env.roadway[tag]
    end
    lanes
end

"""
create a unique ID for a new car
"""
function next_car_id(pomdp::UrbanPOMDP, scene::Scene, rng::AbstractRNG)
    max_id = pomdp.max_cars+1
    current_car_ids = []
    for veh in scene
        if veh.def.class == AgentClass.CAR
            push!(current_car_ids, veh.id)
        end
    end
    possible_ids = setdiff(2:max_id, current_car_ids)
    id = rand(rng, possible_ids)
    return id
end

"""
create a unique ID for a new pedestrian
"""
function next_ped_id(pomdp::UrbanPOMDP, scene::Scene, rng::AbstractRNG)
    max_id = pomdp.max_peds
    current_ped_ids = []
    for veh in scene
        if veh.def.class == AgentClass.PEDESTRIAN
            push!(current_ped_ids, veh.id)
        end
    end
    possible_ids = setdiff(101:100+max_id, current_ped_ids)
    id = rand(rng, possible_ids)
    return id
end

function n_cars(scene::Scene)
    n = 0
    for veh in scene
        if veh.def.class == AgentClass.CAR
            n += 1
        end
    end
    return n
end

function n_pedestrians(scene::Scene)
    n = 0
    for veh in scene
        if veh.def.class == AgentClass.PEDESTRIAN
            n += 1
        end
    end
    return n
end
