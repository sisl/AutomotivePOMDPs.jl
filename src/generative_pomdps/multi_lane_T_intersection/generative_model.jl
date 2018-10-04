# define the generative model for the multiple intersection environment

### REWARD MODEL ##################################################################################

function POMDPs.reward(pomdp::OIPOMDP, s::OIState, a::OIAction, sp::OIState)
    r = 0.
    ego = sp[findfirst(isequal(EGO_ID), sp)]
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

function POMDPs.isterminal(pomdp::OIPOMDP, s::OIState)
    ego = s[findfirst(isequal(EGO_ID), s)]
    return is_crash(s) || (ego.state.posF.s >= get_end(pomdp.env.roadway[pomdp.ego_goal]) &&
                           get_lane(pomdp.env.roadway, ego).tag == pomdp.ego_goal)
end



### TRANSITION MODEL ##############################################################################

function POMDPs.generate_s(pomdp::OIPOMDP, s::OIState, a::OIAction, rng::AbstractRNG)
    actions = Array{Any}(length(s))
    pomdp.models[1].a = a
    is_ego_here = clamp(findfirst(isequal(EGO_ID), s),0, 1)
    sp = deepcopy(s) #XXX bad
    max_id = 0
    for veh in sp
        if veh.id > max_id
            max_id = veh.id
        end
    end
    if rand(rng) < pomdp.p_birth && max_id < pomdp.max_cars +  is_ego_here
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
            pomdp.models[new_car.id] = StopIntersectionDriver(navigator=RouteFollowingIDM(route=route),
                                                       intersection=intersection,
                                                       intersection_entrances = intersection_entrances,
                                                       intersection_exits = intersection_exits,
                                                       stop_delta=0.,
                                                       accel_tol=0.,
                                                       priorities = pomdp.env.priorities)
            push!(sp, new_car)
        end
        actions = Array{Any}(length(sp))
    end
    get_actions!(actions, sp, pomdp.env.roadway, pomdp.models)
    tick!(sp, pomdp.env.roadway, actions, pomdp.ΔT)
    clean_scene!(pomdp.env, sp)
    return sp
end


#TODO move to helpers

function AutomotiveDrivingModels.propagate(veh::Vehicle, a::OIAction, roadway::Roadway, Δt::Float64)
    acc = LonAccelDirection(a.acc, 4) #TODO Parameterize
    return propagate(veh, acc, roadway, Δt)
end


### INITIAL STATES ################################################################################

function POMDPs.initial_state(pomdp::OIPOMDP, rng::AbstractRNG, burn_in::Int64=1)
    scene = initial_scene(pomdp, rng)
    # clean_scene!(pomdp.env, scene)
    for t = 1:burn_in
        scene = generate_s(pomdp, scene, OIAction(0.), rng)
    end
    return scene
end

function initial_scene(pomdp::OIPOMDP, rng::AbstractRNG, no_ego::Bool=false)
    scene = Scene()
    if !no_ego
        ego = initial_ego(pomdp, rng)
        push!(scene, ego)
    end
    for i=1:pomdp.max_cars
        if rand(rng) < pomdp.p_birth && n_cars(scene) < pomdp.max_cars + 1-Int(no_ego)
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
                pomdp.models[new_car.id] = StopIntersectionDriver(navigator=RouteFollowingIDM(route=route),
                                                           intersection=intersection,
                                                           intersection_entrances = intersection_entrances,
                                                           intersection_exits = intersection_exits,
                                                           stop_delta=0.,
                                                           accel_tol=0.,
                                                           priorities = pomdp.env.priorities)
            end
            # pomdp.models[new_car.id] = RouteFollowingIDM(route = random_route(rng,
            #                                                                   pomdp.env.roadway,
            #                                                                   get_lane(pomdp.env.roadway, new_car)))
            clean_scene!(pomdp.env, scene)
        end
    end

    return scene
end

function initial_car(pomdp::OIPOMDP, scene::Scene, rng::AbstractRNG, first_scene::Bool = false)
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


    # new id, increment last id
    max_id = 0
    for veh in scene
        if veh.id > max_id
            max_id = veh.id
        end
    end
    id = max_id + 1
    if max_id == 0
        id = 2
    end

    return Vehicle(initial_state, pomdp.car_type, id)
end


function initial_ego(pomdp::OIPOMDP, rng::AbstractRNG)
    posF = Frenet(get_ego_route(pomdp)[1], pomdp.ego_start)
    state = VehicleState(posF, pomdp.env.roadway, 0.)
    return Vehicle(state, pomdp.ego_type, EGO_ID)
end

function POMDPs.initial_state_distribution(pomdp::OIPOMDP)
    return GenerativeDist(pomdp)
end



### Observations ##################################################################################

# uncomment for vector representation
# TODO find a better way to implement this to switch more easily between representations
function POMDPs.generate_o(pomdp::OIPOMDP, s::Scene, a::OIAction, sp::Scene, rng::AbstractRNG)
    n_features = 4
    pos_noise = pomdp.pos_obs_noise
    vel_noise = pomdp.vel_obs_noise
    o = zeros(n_features*(pomdp.max_cars + 1))
    ego = sp[findfirst(isequal(EGO_ID), sp)].state
    o[1] = ego.posG.x
    o[2] = ego.posG.y
    o[3] = ego.posG.θ
    o[4] = ego.v
    car_off = get_off_the_grid(pomdp)
    for i=2:pomdp.max_cars+1
        o[n_features*i - 3] = car_off.posG.x
        o[n_features*i - 2] = car_off.posG.y
        o[n_features*i - 1] = car_off.posG.θ
        o[n_features*i] = 0.
    end
    for veh in sp
        if veh.id == EGO_ID
            continue
        end
        @assert veh.id <= pomdp.max_cars+1
        car = veh.state
        if is_observable_fixed(ego, car, pomdp.env)
            o[n_features*veh.id - 3] = car.posG.x + pos_noise*randn(rng)
            o[n_features*veh.id - 2] = car.posG.y + pos_noise*randn(rng)
            o[n_features*veh.id - 1] = car.posG.θ
            o[n_features*veh.id] = car.v + vel_noise*randn(rng)
        end
    end
    return o
end

function POMDPs.generate_o(pomdp::OIPOMDP, s::OIState, rng::AbstractRNG)
    return generate_o(pomdp, s, OIAction(0.), s, rng::AbstractRNG)
end

function POMDPs.convert_o(::Type{Vector{Float64}}, o::OIObs, pomdp::OIPOMDP)
    # rescale
    n_features = 4
    max_ego_dist = get_end(pomdp.env.roadway[pomdp.ego_goal])
    o[1] /= max_ego_dist
    o[2] /= max_ego_dist
    o[3] /= pi/2
    o[4] /= pomdp.env.params.speed_limit
    for i=2:pomdp.max_cars+1
        o[n_features*i - 3] /= max_ego_dist
        o[n_features*i - 2] /= max_ego_dist
        o[n_features*i - 1] /= pi
        o[n_features*i] /= pomdp.env.params.speed_limit # XXX parameterized
    end
    return o
end

#### Helpers
# """
#     ego_state(pomdp:ICPOMDP, s::Float64, v::Float64)
# Helper that returns the ego car state given its position on the lane and the velocity
# """
# function ego_state(pomdp::OIPOMDP, s::Float64, v::Float64)
#     # ego_lane = pomdp.env.lane_map["ego_left"] #TODO parameterized
#     posF = Frenet(get_ego_route(pomdp)[1], s)
#     return VehicleState(posF, pomdp.env.roadway, v)
# end

"""
    car_state(pomdp::OIPOMDP, lane_id::String, s::Float64, v::Float64)
    car_state(pomdp::OIPOMDP, lane::Lane, s::Float64, v::Float64)
    car_state(pomdp::OIPOMDP, x::Float64, y::Float64, θ::Float64, v::Float64)
Helper that returns the state of a car given the lane and the position on the lane
"""
function car_state(pomdp::OIPOMDP, lane_id::String, s::Float64, v::Float64)
    lane = pomdp.env.lane_map[lane_id]
    posF = Frenet(lane, s)
    return VehicleState(posF, pomdp.env.roadway, v)
end
function car_state(pomdp::OIPOMDP, lane::Lane, s::Float64, v::Float64)
    posF = Frenet(lane, s)
    return VehicleState(posF, pomdp.env.roadway, v)
end
function car_state(pomdp::OIPOMDP, x::Float64, y::Float64, θ::Float64, v::Float64)
    posG = VecSE2(x, y, θ)
    return VehicleState(posG, pomdp.env.roadway, v)
end

"""
    get_off_the_grid(pomdp::OIPOMDP)
return the off the grid state
"""
function get_off_the_grid(pomdp::OIPOMDP)
    posG = pomdp.off_grid
    return VehicleState(posG, pomdp.env.roadway, 0.)
end

"""
Reconstruct a scene object from an OIPOMDP observation
"""
function obs_to_scene(pomdp::OIPOMDP, o::OIObs)
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
function get_ego_route(pomdp::OIPOMDP)
    tags = [LaneTag(6,1), LaneTag(13, 1), LaneTag(2,pomdp.env.params.nlanes_main)]
    lanes = Array{Lane}(length(tags))
    for (i,tag) in enumerate(tags)
        lanes[i] = pomdp.env.roadway[tag]
    end
    lanes
end
