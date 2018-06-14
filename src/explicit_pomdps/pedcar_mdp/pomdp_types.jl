# joint problem of avoiding one car and one pedestrian

# State type
struct PedCarMDPState
    crash::Bool
    ego::VehicleState
    ped::VehicleState
    car::VehicleState
    route::Vector{LaneTag}
end

# copy b to a
function Base.copy!(a::PedCarMDPState, b::PedCarMDPState)
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
    car_model::DriverModel = RouteFollowingIDM(route=random_route(env, Base.GLOBAL_RNG), σ=1.0)
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

function get_off_the_grid(mdp::PedCarMDP)
    return VehicleState(mdp.off_grid, mdp.env.roadway, 0.)
end

# create labels for model checking
function labeling(mdp::PedCarMDP)
    labels = Dict{PedCarMDPState, Vector{String}}()
    for s in ordered_states(mdp)
        if s.crash
            labels[s] = ["crash"]
        elseif s.ego.posF.s >= get_end(mdp.env.roadway[mdp.ego_goal]) &&
            get_lane(mdp.env.roadway, s.ego).tag == mdp.ego_goal
            labels[s] = ["goal"]
        end
    end
    return labels
end
