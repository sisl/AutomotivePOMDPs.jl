
"""
Representation of a state for the SingleOIcludedIntersection POMDP,
depends on ADM VehicleState type
"""
mutable struct SingleOIState
    crash ::Bool
    ego::VehicleState
    car::VehicleState
end

# copy b to a
function Base.copyto!(a::SingleOIState, b::SingleOIState)
    a.crash = b.crash
    a.ego = b.ego
    a.car = b.car
end

function Base.hash(s::SingleOIState, h::UInt64 = zero(UInt64))
    return hash(s.crash, hash(s.ego, hash(s.car, h)))
end

function Base.:(==)(a::SingleOIState, b::SingleOIState)
    return a.crash == b.crash && a.ego == b.ego && a.car == b.car
end

#### Observaton type

const SingleOIObs = SingleOIState

#### Action type

mutable struct SingleOIAction
    acc::Float64
end

function Base.copyto!(a::SingleOIAction, b::SingleOIAction)
    a.acc = b.acc
end

function Base.hash(a::SingleOIAction, h::UInt64 = zero(UInt64))
    return hash(a.acc, h)
end

function Base.:(==)(a::SingleOIAction, b::SingleOIAction)
    return a.acc == b.acc
end

#### POMDP type




mutable struct SingleOIPOMDP <: POMDP{SingleOIState, SingleOIAction, SingleOIObs}
    env::SimpleInterEnv
    ego_type::VehicleDef
    car_type::VehicleDef
    max_acc::Float64
    pos_res::Float64
    vel_res::Float64
    s_start::Float64
    s_goal::Float64
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

function SingleOIPOMDP(; env::SimpleInterEnv = SimpleInterEnv(),
                   ego_type::VehicleDef = VehicleDef(),
                   car_type::VehicleDef = VehicleDef(),
                   max_acc::Float64 = 2.0,
                   pos_res::Float64 = 2.,
                   vel_res::Float64 = 2.,
                   s_start::Float64 = env.params.stop_line - ego_type.length/2,
                   s_goal::Float64 = 30.,
                   off_grid::VecSE2 = VecSE2(14., -5, pi/2),
                   ΔT::Float64  = 0.5,
                   p_birth::Float64 = 0.1,
                   a_noise::Float64 = 1.0,
                   pos_obs_noise::Float64 = 1.0,
                   vel_obs_noise::Float64 = 1.0,
                   collision_cost::Float64 = -1.,
                   action_cost::Float64 = 0.0,
                   goal_reward::Float64 = 1.,
                   γ::Float64  = 0.95)
    return SingleOIPOMDP(env,
                   ego_type,
                   car_type,
                   max_acc,
                   pos_res,
                   vel_res,
                   s_start,
                   s_goal,
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

### REWARD MODEL ##################################################################################

function POMDPs.reward(pomdp::SingleOIPOMDP, s::SingleOIState, a::SingleOIAction, sp::SingleOIState)
    r = 0.
    if sp.crash
        r += pomdp.collision_cost
    end
    if sp.ego.posF.s >= pomdp.s_goal
        r += pomdp.goal_reward
    elseif a.acc > 0.
        r += pomdp.action_cost
    else
        r += pomdp.action_cost
    end
    return r
end

# other method for SARSOP
function POMDPs.reward(pomdp::SingleOIPOMDP, s::SingleOIState, a::SingleOIAction)
    return reward(pomdp, s, a, s)
end

function POMDPs.isterminal(pomdp::SingleOIPOMDP, s::SingleOIState)
    return s.crash || s.ego.posF.s >= pomdp.s_goal
end

######## DISTRIBUTION ############################################################################

"""
Concrete type to represent a distribution over state for the SingleOIcludedCrosswalk POMDP Problem
"""
mutable struct SingleOIDistribution
    p::Vector{Float64}
    it::Vector{SingleOIState}
end

SingleOIDistribution() = SingleOIDistribution(Float64[], SingleOIState[])

POMDPs.iterator(d::SingleOIDistribution) = d.it

# transition and observation pdf
function POMDPs.pdf(d::SingleOIDistribution, s::SingleOIState)
    for (i, sp) in enumerate(d.it)
        if sp==s
            return d.p[i]
        end
    end
    return 0.
end

function POMDPs.rand(rng::AbstractRNG, d::SingleOIDistribution)
    ns = sample(d.it, Weights(d.p)) # sample a neighbor state according to the distribution c
    return ns
end

"""
    most_likely_state(d::SingleOIDistribution)
returns the most likely state given distribution d
"""
function most_likely_state(d::SingleOIDistribution)
    val, ind = fargmax(d.p)
    return d.it[ind]
end

### HELPERS

function POMDPs.discount(pomdp::SingleOIPOMDP)
    return pomdp.γ
end

function state_to_scene(pomdp::SingleOIPOMDP, s::SingleOIState)
    scene = Scene()
    ego = Vehicle(s.ego, pomdp.ego_type, 1)
    car = Vehicle(s.car, pomdp.car_type, 2)
    push!(scene, ego)
    push!(scene, car)
    return scene
end
