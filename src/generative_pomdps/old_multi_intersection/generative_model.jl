# define the generative model for the multiple intersection environment

### REWARD MODEL ##################################################################################

function POMDPs.reward(pomdp::OIPOMDP, s::OIState, a::OIAction, sp::OIState)
    r = 0.
    ego = sp[findfirst(EGO_ID, sp)]
    if is_crash(sp)
        r += pomdp.collision_cost
    end
    if ego.state.posF.s >= pomdp.ego_goal
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
    ego = s[findfirst(EGO_ID, s)]
    return is_crash(s) || ego.state.posF.s >= pomdp.ego_goal
end



### TRANSITION MODEL ##############################################################################

function POMDPs.gen(::DDNNode{:s}, pomdp::OIPOMDP, s::OIState, a::OIAction, rng::AbstractRNG)
    actions = Array{Any}(length(s))
    pomdp.models[1].a = a
    sp = deepcopy(s) #XXX bad
    max_id = 0
    for veh in sp
        if veh.id > max_id
            max_id = veh.id
        end
    end
    if rand(rng) < pomdp.p_birth && max_id < 9
        new_car = initial_car(pomdp, sp, rng)
        if !isnan(new_car.state.v)
            pomdp.models[new_car.id] = IntelligentDriverModel2D(lane = get_lane(pomdp.env.roadway, new_car))
            push!(sp, new_car)
            actions = Array{Any}(length(sp))
        end
        # push!(sp, new_car)
        # actions = Array{Any}(length(sp))
    end
    get_actions!(actions, sp, pomdp.env.roadway, pomdp.models)
    tick!(sp, pomdp.env.roadway, actions, pomdp.ΔT)
    clean_scene!(pomdp.env, sp)
    return sp
end


function clean_scene!(env::IntersectionEnv, scene::Scene)
    for veh in scene
        if veh.id == 1
            continue
        end
        s = veh.state.posF.s
        lane = get_lane(env.roadway, veh)
        s_end = get_end(lane)
        if s >= s_end
            deleteat!(scene, findfirst(veh.id, scene))
        end
    end
    for veh in scene
        if veh.id == 1; continue; end
        fore = get_neighbor_fore_along_lane(scene, findfirst(veh.id, scene), env.roadway,
                                            VehicleTargetPointFront(), VehicleTargetPointRear(), VehicleTargetPointFront())
        if fore.ind != nothing
            headway, v_oth = fore.Δs, scene[fore.ind].state.v
        else
            headway, v_oth = NaN, NaN
        end
        if !isnan(headway) && headway < 0.
            deleteat!(scene, findfirst(veh.id, scene))
        end
        for veh_ in scene
            if is_colliding(veh, veh_) && veh.id != 1 && veh_.id != 1 && veh.id != veh_.id
                deleteat!(scene, findfirst(veh_.id, scene))
            end
        end
    end
end

function AutomotiveDrivingModels.propagate(veh::Vehicle, a::OIAction, roadway::Roadway, Δt::Float64)
    action = a.acc
    s_ = veh.state.posF.s + veh.state.v*cos(veh.state.posF.ϕ)*Δt + 0.5*action*Δt^2
    if s_ <= veh.state.posF.s # no backup
        s_ = veh.state.posF.s
    end
    v_ = veh.state.v + action*Δt # no backup
    if v_ <= 0.
        v_ = 0.
    end
    veh_lane = pomdp.env.lane_map["ego_left"]#TODO parameterized
    posF = Frenet(veh_lane, s_)
    return VehicleState(posF, pomdp.env.roadway, v_)
end


### INITIAL STATES ################################################################################

function POMDPs.initialstate(pomdp::OIPOMDP, rng::AbstractRNG)

    scene = Scene()
    ego = initial_ego(pomdp, rng)
    push!(scene, ego)
    for i=1:pomdp.max_cars
        if rand(rng) < pomdp.p_birth
            new_car = initial_car(pomdp, scene, rng, true)
            push!(scene, new_car)
            pomdp.models[new_car.id] = IntelligentDriverModel2D(lane = get_lane(pomdp.env.roadway, new_car))
            clean_scene!(pomdp.env, scene)
        end
    end

    return scene
end

function initial_car(pomdp::OIPOMDP, scene::Scene, rng::AbstractRNG, first_scene::Bool = false)
    env = pomdp.env
    lane =env.lane_map[rand(rng, LANE_ID_LIST)]
    #velocity
    v0 = rand(rng, 4.0:1.0:8.0) #XXX parameterize!
    s0 = 0.
    if first_scene
        s0 = rand(rng, 0.:0.5:get_end(lane))
    end
    initial_posF = Frenet(lane, s0)
    initialstate = VehicleState(initial_posF, env.roadway, v0)


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

    return Vehicle(initialstate, pomdp.car_type, id)
end


function can_push(env, scene::Scene, car::Vehicle)
    for veh in scene
        if is_colliding(car, veh)
            return false
        end
    end
    fore = get_neighbor_fore_along_lane(scene, findfirst(car.id, scene), env.roadway,
                                        VehicleTargetPointFront(), VehicleTargetPointRear(), VehicleTargetPointFront())
    if fore.ind != nothing
        headway, v_oth = fore.Δs, scene[fore.ind].state.v
    else
        headway, v_oth = NaN, NaN
    end
    if !isnan(headway) && headway < 0.1
        return false
    end
    return true
end


function initial_ego(pomdp::OIPOMDP, rng::AbstractRNG)
    return Vehicle(ego_state(pomdp, pomdp.ego_start, 0.), pomdp.ego_type, EGO_ID)
end

mutable struct GenerativeDist
    problem::OIPOMDP
end

function POMDPs.initialstate_distribution(pomdp::OIPOMDP)
    return GenerativeDist(pomdp)
end

function Base.rand(rng::AbstractRNG, d::GenerativeDist)
    return initialstate(d.problem, rng)
end



### Observations ##################################################################################

# uncomment for vector representation
# TODO find a better way to implement this to switch more easily between representations
function POMDPs.gen(::DDNNode{:o}, pomdp::OIPOMDP, s::Scene, a::OIAction, sp::Scene, rng::AbstractRNG)
    n_features = 4
    pos_noise = pomdp.pos_obs_noise
    vel_noise = pomdp.vel_obs_noise
    o = zeros(n_features*(pomdp.max_cars + 1))
    ego = sp[findfirst(EGO_ID, sp)].state
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

function POMDPs.gen(::DDNNode{:o}, pomdp::OIPOMDP, s::OIState, rng::AbstractRNG)
    return gen(DDNNode(:o), pomdp, s, OIAction(0.), s, rng::AbstractRNG)
end

function POMDPs.convert_o(::Type{Vector{Float64}}, o::OIObs, pomdp::OIPOMDP)
    # rescale
    n_features = 4
    o[1] /= pomdp.ego_goal
    o[2] /= pomdp.ego_goal
    o[3] /= pi/2
    o[4] /= pomdp.env.params.speed_limit
    for i=2:pomdp.max_cars+1
        o[n_features*i - 3] /= pomdp.ego_goal
        o[n_features*i - 2] /= pomdp.ego_goal
        o[n_features*i - 1] /= pi
        o[n_features*i] /= pomdp.env.params.speed_limit # XXX parameterized
    end
    return o
end

#### Helpers
"""
    ego_state(pomdp:ICPOMDP, s::Float64, v::Float64)
Helper that returns the ego car state given its position on the lane and the velocity
"""
function ego_state(pomdp::OIPOMDP, s::Float64, v::Float64)
    ego_lane = pomdp.env.lane_map["ego_left"]
    posF = Frenet(ego_lane, s)
    return VehicleState(posF, pomdp.env.roadway, v)
end

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
