# define the generative model for the multiple pedestrian crosswalk environment

### REWARD MODEL ##################################################################################

function POMDPs.reward(pomdp::OCPOMDP, s::OCState, a::OCAction, sp::OCState)
    r = 0.
    ego = sp[findfirst(sp, 1)]
    if is_crash(sp)
        r += pomdp.collision_cost
    end
    if ego.state.posG.x >= pomdp.ego_goal
        r += pomdp.goal_reward
    elseif a.acc > 0.
        r += pomdp.action_cost
    else
        r += pomdp.action_cost
    end
    return r
end

### TERMINAL STATES ###############################################################################

function POMDPs.isterminal(pomdp::OCPOMDP, s::OCState)
    ego = s[findfirst(s, EGO_INDEX)]
    return is_crash(s) || ego.state.posG.x >= pomdp.ego_goal
end

### TRANSITION MODEL ##############################################################################

function POMDPs.generate_s(pomdp::OCPOMDP, s::OCState, a::OCAction, rng::AbstractRNG)
    actions = Array{Any}(length(s))
    pomdp.models[1].a = a
    sp = deepcopy(s) #XXX bad
    max_id = 0
    for veh in sp
        if veh.id > max_id
            max_id = veh.id
        end
    end
    if rand(rng) < 0.01 && max_id < 9
        # println("Spawning new pedestrians")
        new_ped = initial_pedestrian(pomdp, sp, rng)
        pomdp.models[new_ped.id] = ConstantPedestrian(dt = pomdp.ΔT)#TODO parameterized
        push!(sp, new_ped)
        actions = Array{Any}(length(sp))
    end
    get_actions!(actions, sp, pomdp.env.roadway, pomdp.models)
    tick!(sp, pomdp.env.roadway, actions, pomdp.ΔT)
    clean_scene!(pomdp.env, sp)
    return sp
end

function clean_scene!(env::CrosswalkEnv, scene::Scene)
    for veh in scene
        if veh.id == 1
            continue
        end
        y = veh.state.posG.y
        if y >= env.params.crosswalk_length/4
            deleteat!(scene, findfirst(scene, veh.id))
        end
    end
end

function AutomotiveDrivingModels.propagate(veh::Vehicle, action::OCAction, roadway::Roadway, Δt::Float64)
    x_ = veh.state.posG.x + veh.state.v*Δt + action.acc*Δt^2/2
    if x_ <= veh.state.posG.x
        x_ = veh.state.posG.x
    end
    v_ = veh.state.v + action.acc*Δt
    clamp(v_, 0., 8.) # parameterized

    return VehicleState(VecSE2(x_, veh.state.posG.y, veh.state.posG.θ), roadway, v_)
end

### INITIAL STATES ################################################################################

function POMDPs.initial_state(pomdp::OCPOMDP, rng::AbstractRNG)
    max_init_ped = 10 # maximum number of pedestrians initially present

    scene = Scene()
    n_ped = rand(rng, 0:max_init_ped)
    for i=1:n_ped
        if rand(rng) < pomdp.p_birth # pedestrian appear
            new_ped = initial_pedestrian(pomdp, scene, rng, true)
            pomdp.models[new_ped.id] = ConstantPedestrian(dt = pomdp.ΔT)
            push!(scene, new_ped)
        end
    end
    ego = initial_ego(pomdp, rng)
    push!(scene, ego)
    return scene
end

"""
    initial_ego(pomdp::OCPOMDP, rng::AbstractRNG)
generate an initial state for the ego car, returns a Vehicle object
"""
function initial_ego(pomdp::OCPOMDP, rng::AbstractRNG)
    env = pomdp.env
    x0 = pomdp.ego_start
    y0 = 0. # parameterize
    v0 = rand(rng, Uniform(6., pomdp.env.params.speed_limit)) #TODO parameterize

    return Vehicle(VehicleState(VecSE2(x0, y0, 0.0), env.roadway.segments[1].lanes[1], env.roadway, v0),
                   VehicleDef(), 1)
end


"""
    initial_pedestrian(pomdp::OCPOMDP, scene::Scene, rng::AbstractRNG)
Create a new pedestrian entity at its initial state
"""
function initial_pedestrian(pomdp::OCPOMDP, scene::Scene, rng::AbstractRNG, first_scene::Bool = false)
    env = pomdp.env
    # lateral position
    d_lat = Uniform(env.params.roadway_length - env.params.crosswalk_width + 2, env.params.roadway_length + env.params.crosswalk_width-1)
    x0 = 0.5*rand(rng, d_lat)

    # vertical position
    y0 = pomdp.ped_start
    if first_scene
        y0 = rand(rng, pomdp.ped_start:pomdp.ped_goal)
    end

    # orientation
    θ = π/2

    #velocity
    v0 = rand(rng, Uniform(0., env.params.ped_max_speed))
    cw_roadway = Roadway([RoadSegment(2, [env.crosswalk])]);
    ped_initial_state = VehicleState(VecSE2(x0, y0, θ), env.crosswalk, cw_roadway, v0);

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


    return Vehicle(ped_initial_state, PEDESTRIAN_DEF, id)
end

mutable struct GenerativeDist
    problem::OCPOMDP
end

function POMDPs.initial_state_distribution(pomdp::OCPOMDP)
    return GenerativeDist(pomdp)
end

function Base.rand(rng::AbstractRNG, d::GenerativeDist)
    return initial_state(d.problem, rng)
end




### Observations ##################################################################################

# uncomment for LIDAR observation
# function POMDPs.generate_o(pomdp::OCPOMDP, s::OCState, a::OCAction, sp::OCState, rng::AbstractRNG)
#     observe!(pomdp.sensor, sp, pomdp.env.roadway, EGO_INDEX)
#     return pomdp.sensor
# end
#
# function POMDPs.generate_o(pomdp::OCPOMDP, s::OCState, rng::AbstractRNG)
#     observe!(pomdp.sensor, s, pomdp.env.roadway, EGO_INDEX)
#     return pomdp.sensor
# end
#
# function POMDPs.convert_o(::Type{Vector{Float64}}, o::OCObs, pomdp::OCPOMDP)
#     return [o.ranges/pomdp.sensor.max_range, o.range_rates/pomdp.env.params.speed_limit]
# end

# uncomment for vector representation
function POMDPs.generate_o(pomdp::OCPOMDP, s::OCState, a::OCAction, sp::OCState, rng::AbstractRNG)
    pos_noise = pomdp.pos_obs_noise
    vel_noise = pomdp.vel_obs_noise
    o = zeros(2 + 2*pomdp.max_ped)
    ego = sp[findfirst(sp, 1)].state
    o[1] = ego.posG.x
    o[2] = ego.v
    ped_off = get_off_the_grid(pomdp)
    for i=2:pomdp.max_ped+1
        o[2*i - 1] = ped_off.posG.y
        o[2*i] = 0.
    end
    for veh in sp
        if veh.id == 1
            continue
        end
        @assert veh.id <= pomdp.max_ped+1
        ped = veh.state
        if is_observable(ped, ego, pomdp.env)
            o[2*veh.id - 1] = ped.posG.y + pos_noise*randn(rng)
            o[2*veh.id] = ped.v + vel_noise*randn(rng)
        end
    end
    return o
end

function POMDPs.generate_o(pomdp::OCPOMDP, s::OCState, rng::AbstractRNG)
    return generate_o(pomdp, s, OCAction(0.), s, rng::AbstractRNG)
end

function POMDPs.convert_o(::Type{Vector{Float64}}, o::OCObs, pomdp::OCPOMDP)
    o[1] /= pomdp.ego_goal
    o[2] /= pomdp.env.params.speed_limit
    for i=2:pomdp.max_ped+1
        o[2*i - 1] /= pomdp.ped_goal
        o[2*i] /= 2. # XXX parameterized
    end
    return o
end

### All together ##################################################################################

function POMDPs.generate_sor(pomdp::OCPOMDP, s::OCState, a::OCAction, rng::AbstractRNG)
    sp = generate_s(pomdp, s, a, rng)
    o = generate_o(pomdp, s, a, sp, rng)
    r = reward(pomdp, s, a, sp)
    return sp, o, r
end

#### Helpers

"""
    get_off_the_grid(pomdp::OCPOMDP)
return the off the grid pedestrian state
"""
function get_off_the_grid(pomdp::OCPOMDP)
    env = pomdp.env
    x_ped = 0.5*(env.params.roadway_length - env.params.crosswalk_width + 1)
    cl = pomdp.env.params.crosswalk_length
    pos = VecSE2(x_ped, -cl/2 - 1, pi/2)
    return VehicleState(pos, pomdp.env.roadway, Inf)
end

function is_observable(car::VehicleState, ego::VehicleState, env::CrosswalkEnv)
    m = length(env.obstacles)
    front = ego.posG + polar(VehicleDef().length/2, ego.posG.θ)
    angle = atan2(car.posG.y - front.y, car.posG.x - front.x)
    ray = Projectile(VecSE2(front.x, front.y, angle), 1.0)
    if isinf(car.v)
        return false
    end
    for i = 1:m
        if is_colliding(ray, env.obstacles[i], car.posG)
            return false
        end
    end
    return true
end

function AutomotiveDrivingModels.is_colliding(P::Projectile, poly::ConvexPolygon, A::VecSE2)
    # collides if at least one of the segments collides with the ray
    point_time = sqrt(abs2(A - P.pos))
    for i in 1 : length(poly)
        seg = get_edge(poly, i)
        obs_time = get_intersection_time(P, seg)
        if !isinf(obs_time) && obs_time < point_time
            return true
        end
    end
    false
end
