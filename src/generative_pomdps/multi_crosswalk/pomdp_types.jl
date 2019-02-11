### State type

# just an ADM scene
const OCState = Records.Frame{Records.Entity{AutomotiveDrivingModels.VehicleState,AutomotiveDrivingModels.VehicleDef,Int64}}

#### Observaton type

# # for LidarFeatures
# const OCObs = LidarSensor

# for vector features
const OCObs = Array{Float64, 1}
#### Action type

struct OCAction
    acc::Float64
end

function Base.copyto!(a::OCAction, b::OCAction)
    a.acc = b.acc
end

function Base.hash(a::OCAction, h::UInt64 = zero(UInt64))
    return hash(a.acc, h)
end

function Base.:(==)(a::OCAction, b::OCAction)
    return a.acc == b.acc
end


#### POMDP type

@with_kw mutable struct OCPOMDP <: POMDP{OCState, OCAction, OCObs}
    env::CrosswalkEnv = CrosswalkEnv(CrosswalkParams(obstacles_visible=true))
    sensor::AbstractSensor = GaussianSensor()
    models::Dict{Int64, DriverModel} = Dict{Int64, DriverModel}(1=>EgoDriver(OCAction(0.)))
    ego_type::VehicleDef =  VehicleDef()
    ped_type::VehicleDef = VehicleDef(AgentClass.PEDESTRIAN, 1.0, 1.0)
    max_acc::Float64 = 2.0
    ego_start::Float64 = 5.0
    ped_start::Float64 = -env.params.crosswalk_length/4
    ego_goal::Float64 = env.params.roadway_length/2 + 2*env.params.crosswalk_width
    ped_goal::Float64 = env.params.crosswalk_length/4
    ΔT::Float64 = 0.5 # decision frequency
    p_birth::Float64 = 0.3 # probability of appearance
    max_peds::Int64 = 10
    no_ped_prob::Float64 = 0.3 # probability that there is no pedestrian (for training)
    no_ped::Bool = false
    v_noise::Float64 = 1.0
    pos_obs_noise::Float64 = 0.5
    vel_obs_noise::Float64 = 0.5
    collision_cost::Float64 = -1.0
    action_cost::Float64 = 0.0
    goal_reward::Float64 = 1.0
    γ::Float64 = 0.95 # discount factor
end

# function OCPOMDP(; env::CrosswalkEnv = CrosswalkEnv(),
#                    sensor::LidarSensor = LidarSensor(50, max_range=30., angle_spread=float(pi)),
#                    models::Dict{Int64, DriverModel} = Dict{Int64, DriverModel}(1=>EgoDriver(OCAction(0.))),
#                    ego_type::VehicleDef = VehicleDef(),
#                    ped_type::VehicleDef = VehicleDef(AgentClass.PEDESTRIAN, 1.0, 1.0),#,env.params.crosswalk_width),
#                    max_acc::Float64 = 2.0,
#                    ego_start::Float64 = 5.,
#                    ped_start::Float64 = -env.params.crosswalk_length/4,
#                    ego_goal::Float64 = env.params.roadway_length/2 + 2*env.params.crosswalk_width,
#                    ped_goal::Float64 = env.params.crosswalk_length/4,
#                    ΔT::Float64  = 0.5,
#                    p_birth::Float64 = 0.3,
#                    max_peds::Int64=10,
#                    no_ped_prob::Float64 = 0.3,
#                    no_ped::Bool = false,
#                    v_noise::Float64 = 1.,
#                    pos_obs_noise::Float64 = 0.5,
#                    vel_obs_noise::Float64 = 0.5,
#                    collision_cost::Float64 = -1.,
#                    action_cost::Float64 = 0.0,
#                    goal_reward::Float64 = 1.,
#                    γ::Float64  = 0.95)
#     return OCPOMDP(env,
#                    sensor,
#                    models,
#                    ego_type,
#                    ped_type,
#                    max_acc,
#                    ego_start,
#                    ped_start,
#                    ego_goal,
#                    ped_goal,
#                    ΔT,
#                    p_birth,
#                    max_peds,
#                    no_ped_prob,
#                    no_ped,
#                    v_noise,
#                    pos_obs_noise,
#                    vel_obs_noise,
#                    collision_cost,
#                    action_cost,
#                    goal_reward,
#                    γ)
# end

### HELPERS

function POMDPs.discount(pomdp::OCPOMDP)
    return pomdp.γ
end

POMDPs.actions(pomdp::OCPOMDP) = [OCAction(-4.0), OCAction(-2.0), OCAction(0.0), OCAction(2.0)]
POMDPs.n_actions(pomdp::OCPOMDP) = 4

function POMDPs.actionindex(pomdp::OCPOMDP, action::OCAction)
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
