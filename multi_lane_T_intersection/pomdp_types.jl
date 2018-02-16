
# just an ADM scene

const OIState = Records.Frame{Records.Entity{AutomotiveDrivingModels.VehicleState,AutomotiveDrivingModels.VehicleDef,Int64}}

#### Observaton type

# # for LidarFeatures
# const OIObs = LidarSensor

# for vector features
const OIObs = Array{Float64, 1}
#### Action type

#### Action type

mutable struct OIAction
    acc::Float64
end

function Base.copy!(a::OIAction, b::OIAction)
    a.acc = b.acc
end

function Base.hash(a::OIAction, h::UInt64 = zero(UInt64))
    return hash(a.acc, h)
end

function Base.:(==)(a::OIAction, b::OIAction)
    return a.acc == b.acc
end



#### POMDP type

mutable struct OIPOMDP <: POMDP{OIState, OIAction, OIObs}
    env::IntersectionEnv
    sensor::LidarSensor
    models::Dict{Int64, DriverModel}
    ego_type::VehicleDef
    car_type::VehicleDef
    max_cars::Int64
    max_acc::Float64
    ego_start::Float64
    ego_goal::LaneTag
    off_grid::VecSE2
    ΔT::Float64 # decision frequency
    p_birth::Float64
    a_noise::Float64
    pos_obs_noise::Float64
    vel_obs_noise::Float64
    collision_cost::Float64
    action_cost::Float64
    goal_reward::Float64
    γ::Float64 # discount factor
end

function OIPOMDP(; env::IntersectionEnv = IntersectionEnv(),
                   sensor::LidarSensor = LidarSensor(50, max_range=30., angle_spread=float(pi)),
                   models::Dict{Int64, DriverModel} = Dict{Int64, DriverModel}(1=>EgoDriver(OIAction(0.))),
                   ego_type::VehicleDef = VehicleDef(),
                   car_type::VehicleDef = VehicleDef(),
                   max_cars::Int64 = 10,
                   max_acc::Float64 = 2.0,
                   ego_start::Float64 = env.params.stop_line - ego_type.length/2,
                   ego_goal::LaneTag = LaneTag(2,1),
                   off_grid::VecSE2 = VecSE2(14., -5, pi/2),
                   ΔT::Float64  = 0.5,
                   p_birth::Float64 = 0.3,
                   a_noise::Float64 = 1.0,
                   pos_obs_noise::Float64 = 0.5,
                   vel_obs_noise::Float64 = 0.5,
                   collision_cost::Float64 = -1.,
                   action_cost::Float64 = 0.0,
                   goal_reward::Float64 = 1.,
                   γ::Float64  = 0.95)
    return OIPOMDP(env,
                   sensor,
                   models,
                   ego_type,
                   car_type,
                   max_cars,
                   max_acc,
                   ego_start,
                   ego_goal,
                   off_grid,
                   ΔT,
                   p_birth,
                   a_noise,
                   pos_obs_noise,
                   vel_obs_noise,
                   collision_cost,
                   action_cost,
                   goal_reward,
                   γ)
end


### HELPERS

function POMDPs.discount(pomdp::OIPOMDP)
    return pomdp.γ
end

POMDPs.actions(pomdp::OIPOMDP) = [OIAction(-4.0), OIAction(-2.0), OIAction(0.0), OIAction(2.0)]
POMDPs.n_actions(pomdp::OIPOMDP) = 4

function POMDPs.action_index(pomdp::OIPOMDP, a::OIAction)
    if a.acc == -4.0
       return 1
    elseif a.acc == -2.0
       return 2
    elseif a.acc == 0.0
       return 3
    elseif a.acc == 2.0
       return 4
    end
end
