
"""
Representation of a state for the OccludedCrosswalk POMDP,
depends on ADM VehicleState type
"""
mutable struct OCState
    crash ::Bool
    ego::VehicleState
    ped::VehicleState
end

# copy b to a
function Base.copy!(a::OCState, b::OCState)
    a.crash = b.crash
    a.ego = b.ego
    a.ped = b.ped
end

function Base.hash(s::OCState, h::UInt64 = zero(UInt64))
    return hash(s.crash, hash(s.ego, hash(s.ped, h)))
end

function Base.:(==)(a::OCState, b::OCState)
    return a.crash == b.crash && a.ego == b.ego && a.ped == b.ped
end

#### Observaton type

const OCObs = OCState

#### Action type

mutable struct OCAction
    acc::Float64
end

function Base.copy!(a::OCAction, b::OCAction)
    a.acc = b.acc
end

function Base.hash(a::OCAction, h::UInt64 = zero(UInt64))
    return hash(a.acc, h)
end

function Base.:(==)(a::OCAction, b::OCAction)
    return a.acc == b.acc
end

#### POMDP type
const PED_ID = 2

mutable struct OCPOMDP <: POMDP{OCState, OCAction, OCObs}
    env::CrosswalkEnv
    ego_type::VehicleDef
    ped_type::VehicleDef
    max_acc::Float64
    pos_res::Float64
    vel_res::Float64
    x_start::Float64
    y_start::Float64
    x_goal::Float64
    y_goal::Float64
    ΔT::Float64 # decision frequency
    p_birth::Float64
    no_ped_prob::Float64
    no_ped::Bool
    v_noise::Float64
    pos_obs_noise::Float64
    vel_obs_noise::Float64
    collision_cost::Float64
    action_cost::Float64
    goal_reward::Float64
    γ::Float64 # discount factor
end

function OCPOMDP(; env::CrosswalkEnv = CrosswalkEnv(),
                   ego_type::VehicleDef = VehicleDef(),
                   ped_type::VehicleDef = VehicleDef(AgentClass.PEDESTRIAN, 1.0, 1.0),#,env.params.crosswalk_width),
                   max_acc::Float64 = 2.0,
                   pos_res::Float64 = 1.,
                   vel_res::Float64 = 1.,
                   x_start::Float64 = 5.,
                   y_start::Float64 = -env.params.crosswalk_length/4,
                   x_goal::Float64 = env.params.roadway_length/2 + 2*env.params.crosswalk_width,
                   y_goal::Float64 = env.params.crosswalk_length/4,
                   ΔT::Float64  = 0.5,
                   p_birth::Float64 = 0.3,
                   no_ped_prob::Float64 = 0.3,
                   no_ped::Bool = false,
                   v_noise::Float64 = 1.,
                   pos_obs_noise::Float64 = 1.0,
                   vel_obs_noise::Float64 = 1.0,
                   collision_cost::Float64 = -1.,
                   action_cost::Float64 = 0.0,
                   goal_reward::Float64 = 1.,
                   γ::Float64  = 0.95)
    return OCPOMDP(env,
                   ego_type,
                   ped_type,
                   max_acc,
                   pos_res,
                   vel_res,
                   x_start,
                   y_start,
                   x_goal,
                   y_goal,
                   ΔT,
                   p_birth,
                   no_ped_prob,
                   no_ped,
                   v_noise,
                   pos_obs_noise,
                   vel_obs_noise,
                   collision_cost,
                   action_cost,
                   goal_reward,
                   γ)
end

### REWARD MODEL ##################################################################################

function POMDPs.reward(pomdp::OCPOMDP, s::OCState, a::OCAction, sp::OCState)
    r = 0.
    if sp.crash
        r += pomdp.collision_cost
    end
    if sp.ego.posG.x >= pomdp.x_goal
        r += pomdp.goal_reward
    elseif a.acc > 0.
        r += pomdp.action_cost
    else
        r += pomdp.action_cost
    end
    return r
end

# other method for SARSOP
function POMDPs.reward(pomdp::OCPOMDP, s::OCState, a::OCAction)
    r = 0.
    if s.crash
        r += pomdp.collision_cost
    end
    if s.ego.posG.x >= pomdp.x_goal
        r += pomdp.goal_reward
    elseif a.acc > 0.
        r += pomdp.action_cost
    else
        r += pomdp.action_cost
    end
    return r
end

function POMDPs.isterminal(pomdp::OCPOMDP, s::OCState)
    return s.crash || s.ego.posG.x >= pomdp.x_goal
end

######## DISTRIBUTION ############################################################################

"""
Concrete type to represent a distribution over state for the OccludedCrosswalk POMDP Problem
"""
mutable struct OCDistribution
    p::Vector{Float64}
    it::Vector{OCState}
end

OCDistribution() = OCDistribution(Float64[], OCState[])

POMDPs.iterator(d::OCDistribution) = d.it

# transition and observation pdf
function POMDPs.pdf(d::OCDistribution, s::OCState)
    for (i, sp) in enumerate(d.it)
        if sp==s
            return d.p[i]
        end
    end
    return 0.
end

function POMDPs.rand(rng::AbstractRNG, d::OCDistribution)
    ns = sample(d.it, Weights(d.p)) # sample a neighbor state according to the distribution c
    return ns
end

"""
    most_likely_state(d::OCDistribution)
returns the most likely state given distribution d
"""
function most_likely_state(d::OCDistribution)
    val, ind = findmax(d.p)
    return d.it[ind]
end

### HELPERS





function POMDPs.discount(pomdp::OCPOMDP)
    return pomdp.γ
end
