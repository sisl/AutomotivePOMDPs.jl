
"""
Representation of a state for the SingleOCcludedCrosswalk POMDP,
depends on ADM VehicleState type
"""
mutable struct SingleOCState
    crash ::Bool
    ego::VehicleState
    ped::VehicleState
end

# copy b to a
function Base.copy!(a::SingleOCState, b::SingleOCState)
    a.crash = b.crash
    a.ego = b.ego
    a.ped = b.ped
end

function Base.hash(s::SingleOCState, h::UInt64 = zero(UInt64))
    return hash(s.crash, hash(s.ego, hash(s.ped, h)))
end

function Base.:(==)(a::SingleOCState, b::SingleOCState)
    return a.crash == b.crash && a.ego == b.ego && a.ped == b.ped
end

#### Observaton type

const SingleOCObs = SingleOCState

#### Action type

mutable struct SingleOCAction
    acc::Float64
end

function Base.copy!(a::SingleOCAction, b::SingleOCAction)
    a.acc = b.acc
end

function Base.hash(a::SingleOCAction, h::UInt64 = zero(UInt64))
    return hash(a.acc, h)
end

function Base.:(==)(a::SingleOCAction, b::SingleOCAction)
    return a.acc == b.acc
end

#### POMDP type
const PED_ID = 2

mutable struct SingleOCPOMDP <: POMDP{SingleOCState, SingleOCAction, SingleOCObs}
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

function SingleOCPOMDP(; env::CrosswalkEnv = CrosswalkEnv(),
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
    return SingleOCPOMDP(env,
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

function POMDPs.reward(pomdp::SingleOCPOMDP, s::SingleOCState, a::SingleOCAction, sp::SingleOCState)
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
function POMDPs.reward(pomdp::SingleOCPOMDP, s::SingleOCState, a::SingleOCAction)
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

function POMDPs.isterminal(pomdp::SingleOCPOMDP, s::SingleOCState)
    return s.crash || s.ego.posG.x >= pomdp.x_goal
end

######## DISTRIBUTION ############################################################################

"""
Concrete type to represent a distribution over state for the SingleOCcludedCrosswalk POMDP Problem
"""
mutable struct SingleOCDistribution
    p::Vector{Float64}
    it::Vector{SingleOCState}
end

SingleOCDistribution() = SingleOCDistribution(Float64[], SingleOCState[])

POMDPs.iterator(d::SingleOCDistribution) = d.it

# transition and observation pdf
function POMDPs.pdf(d::SingleOCDistribution, s::SingleOCState)
    for (i, sp) in enumerate(d.it)
        if sp==s
            return d.p[i]
        end
    end
    return 0.
end

function POMDPs.rand(rng::AbstractRNG, d::SingleOCDistribution)
    ns = sample(d.it, Weights(d.p)) # sample a neighbor state according to the distribution c
    return ns
end

"""
    most_likely_state(d::SingleOCDistribution)
returns the most likely state given distribution d
"""
function most_likely_state(d::SingleOCDistribution)
    val, ind = findmax(d.p)
    return d.it[ind]
end

### HELPERS





function POMDPs.discount(pomdp::SingleOCPOMDP)
    return pomdp.γ
end
