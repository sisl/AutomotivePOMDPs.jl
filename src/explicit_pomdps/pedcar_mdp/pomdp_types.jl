# joint problem of avoiding one car and one pedestrian

# State type
struct PedCarMDPState
    crash::Bool
    ego::VehicleState
    ped::VehicleState
    car::VehicleState
    route::SVector{2, LaneTag}
end

# copy b to a
function Base.copyto!(a::PedCarMDPState, b::PedCarMDPState)
    a.crash = b.crash
    a.ego = b.ego
    a.ped = b.ped
    a.car = b.car
    a.route = b.route
end

function Base.hash(s::PedCarMDPState, h::UInt64 = zero(UInt64))
    return hash(s.crash, hash(s.ego, hash(s.ped, hash(s.car, hash(s.route, h)))))
end

function Base.:(==)(a::PedCarMDPState, b::PedCarMDPState)
    return a.crash == b.crash && a.ego == b.ego && a.ped == b.ped && a.car == b.car && a.route == b.route
end

# Action type
const PedCarMDPAction = UrbanAction

@with_kw mutable struct PedCarMDP <: MDP{PedCarMDPState, PedCarMDPAction}
    env::UrbanEnv = UrbanEnv(params=UrbanParams(nlanes_main=1,
                     crosswalk_pos =  [VecSE2(6, 0., pi/2), VecSE2(-6, 0., pi/2), VecSE2(0., -5., 0.)],
                     crosswalk_length =  [10.0, 10., 10.0],
                     crosswalk_width = [4.0, 4.0, 3.1],
                     stop_line = 22.0))
    ego_type::VehicleDef = VehicleDef()
    car_type::VehicleDef = VehicleDef()
    car_models::Dict{SVector{2, LaneTag}, DriverModel} = get_car_models(env, get_stop_model)
    ped_type::VehicleDef = VehicleDef(AgentClass.PEDESTRIAN, 1.0, 1.0)
    ped_model::DriverModel = ConstantPedestrian()
    max_acc::Float64 = 2.0
    pos_res::Float64 = 3.0
    vel_res::Float64 = 2.0
    vel_ped_res::Float64 = 1.0
    ego_start::Float64 = env.params.stop_line - ego_type.length/2
    ego_goal::LaneTag = LaneTag(2,1)
    off_grid::VecSE2 = VecSE2(UrbanEnv().params.x_min+VehicleDef().length/2, env.params.y_min+VehicleDef().width/2, 0)
    ΔT::Float64 = 0.5
    car_birth::Float64 = 0.3
    ped_birth::Float64 = 0.3   
    collision_cost::Float64 = -1.0
    action_cost::Float64 = 0.
    goal_reward::Float64 = 1.
    γ::Float64 = 0.95
    _ped_grid::Dict{LaneTag, RectangleGrid{3}} = init_ped_grid(env, pos_res, vel_ped_res)
    _car_grid::Dict{LaneTag, RectangleGrid{2}} = init_car_grid(env, pos_res, vel_res)
    _l_grid::Dict{LaneTag, RectangleGrid{1}} = init_l_grid(env, pos_res)
    _v_grid::RectangleGrid{1} = init_v_grid(env, vel_res)
    _collision_checker::Dict{Tuple{Vararg{VehicleState, 3}}, Bool} = Dict{Tuple{Vararg{VehicleState, 3}}, Bool}()
    _car_proj::Dict{Tuple{Float64, Float64, LaneTag}, VehicleState} = car_projection(env, pos_res, vel_res)
    _ped_proj::Dict{Tuple{Float64, Float64, Float64, LaneTag}, VehicleState} = ped_projection(env, pos_res, vel_ped_res)
end


function POMDPs.reward(mdp::PedCarMDP, s::PedCarMDPState, a::PedCarMDPAction, sp::PedCarMDPState)
    r = mdp.action_cost
    ego = sp.ego
    if sp.crash
        r += mdp.collision_cost
    end
    if ego.posF.s >= get_end(mdp.env.roadway[mdp.ego_goal]) &&
       get_lane(mdp.env.roadway, ego).tag == mdp.ego_goal
        r += mdp.goal_reward
    end
    return r
end

function POMDPs.isterminal(mdp::PedCarMDP, s::PedCarMDPState)
    if s.crash
        return true
    elseif s.ego.posF.s >= get_end(mdp.env.roadway[mdp.ego_goal]) &&
       get_lane(mdp.env.roadway, s.ego).tag == mdp.ego_goal
       return true
   end
   return false
end

## Helpers

function POMDPs.discount(mdp::PedCarMDP)
    return mdp.γ
end

POMDPs.actions(mdp::PedCarMDP) = [PedCarMDPAction(-4.0), PedCarMDPAction(-2.0), PedCarMDPAction(0.0), PedCarMDPAction(2.0)]
POMDPs.n_actions(mdp::PedCarMDP) = 4

function POMDPs.action_index(mdp::PedCarMDP, action::PedCarMDPAction)
    if action.acc == -4.0
        return 1
    elseif action.acc == -2.0
        return 2
    elseif action.acc == 0.
        return 3
    else
        return 4
    end
end


function POMDPs.convert_s(::Type{V}, s::PedCarMDPState, mdp::PedCarMDP) where V<:AbstractArray
    n_routes = 4
    n_features = 4
    z = zeros(n_features*3 + n_routes + 1)
    z[1] = s.ego.posG.x / abs(mdp.env.params.x_min)
    z[2] = s.ego.posG.y / abs(mdp.env.params.y_min)
    z[3] = s.ego.posG.θ / π
    z[4] = s.ego.v / abs(mdp.env.params.speed_limit)
    z[5] = s.ped.posG.x / abs(mdp.env.params.x_min)
    z[6] = s.ped.posG.y / abs(mdp.env.params.y_min)
    z[7] = s.ped.posG.θ / π
    z[8] = s.ped.v / abs(mdp.env.params.speed_limit)
    z[9] = s.car.posG.x / abs(mdp.env.params.x_min)
    z[10] = s.car.posG.y / abs(mdp.env.params.y_min)
    z[11] = s.car.posG.θ / π
    z[12] = s.car.v / abs(mdp.env.params.speed_limit)
    # one hot encoding for the route
    routes = get_car_routes(mdp.env)
    for (i, r) in enumerate(routes)
        if r[1] == s.route[1] && r[end] == s.route[end]
            z[12+i] = 1.
        end
    end
    z[17] = float(s.crash)
    return z
end

function POMDPs.convert_s(::Type{PedCarMDPState}, z::V, mdp::PedCarMDP) where V<:AbstractArray{Float64}
    n_routes = 4
    n_features = 4
    @assert length(z) == n_features*3 + n_routes + 1 
    ego_x = z[1]*abs(mdp.env.params.x_min)
    ego_y = z[2]*abs(mdp.env.params.y_min)
    ego_θ = z[3]*π
    ego_v = z[4]*abs(mdp.env.params.speed_limit)
    ego = VehicleState(VecSE2(ego_x, ego_y, ego_θ), mdp.env.roadway, ego_v)
    ped_x = z[5]*abs(mdp.env.params.x_min)
    ped_y = z[6]*abs(mdp.env.params.y_min)
    ped_θ = z[7]*π
    ped_v = z[8]*abs(mdp.env.params.speed_limit)
    ped = VehicleState(VecSE2(ped_x, ped_y, ped_θ), mdp.env.ped_roadway, ped_v)
    car_x = z[9]*abs(mdp.env.params.x_min)
    car_y = z[10]*abs(mdp.env.params.y_min)
    car_θ = z[11]*π
    car_v = z[12]*abs(mdp.env.params.speed_limit)
    car = VehicleState(VecSE2(car_x, car_y, car_θ), mdp.env.roadway, car_v)
    # one hot encoding for the route
    routes = get_car_routes(mdp.env)
    route = SVector{2, LaneTag}(LaneTag(0, 0), LaneTag(0, 0))
    for (i, r) in enumerate(routes)
        if z[12+i] == 1.
            route = SVector{2, LaneTag}(r[1], r[end])
        end
    end
    collision = Bool(z[17])
    return PedCarMDPState(collision, ego, ped, car, route)
end

function get_off_the_grid(mdp::PedCarMDP)
    return VehicleState(mdp.off_grid, Frenet(mdp.env.roadway[LaneTag(5,1)], 25.1, -26.5, pi/2), 0.)
    # return VehicleState(mdp.off_grid, mdp.env.roadway, 0.)
end

# create labels for model checking
function labeling(mdp::PedCarMDP)
    labels = Dict{PedCarMDPState, Vector{String}}()
    for s in ordered_states(mdp)
        if crash(mdp, s)
            labels[s] = ["crash"]
        elseif s.ego.posF.s >= get_end(mdp.env.roadway[mdp.ego_goal]) &&
            get_lane(mdp.env.roadway, s.ego).tag == mdp.ego_goal
            labels[s] = ["goal"]
        end
    end
    return labels
end

function crash(mdp::PedCarMDP, s::PedCarMDPState)
    return crash(mdp, s.ego, s.car, s.ped)
end

function crash(mdp::PedCarMDP, ego::VehicleState, car::VehicleState, ped::VehicleState)
    return collision_checker(ego, car, mdp.ego_type, mdp.car_type) || collision_checker(ego, ped, mdp.ego_type, mdp.ped_type)
end

