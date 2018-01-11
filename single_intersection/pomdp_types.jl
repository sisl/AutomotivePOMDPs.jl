
"""
Representation of a state for the OIcludedIntersection POMDP,
depends on ADM VehicleState type
"""
mutable struct OIState
    crash ::Bool
    ego::VehicleState
    car::VehicleState
end

# copy b to a
function Base.copy!(a::OIState, b::OIState)
    a.crash = b.crash
    a.ego = b.ego
    a.car = b.car
end

function Base.hash(s::OIState, h::UInt64 = zero(UInt64))
    return hash(s.crash, hash(s.ego, hash(s.car, h)))
end

function Base.:(==)(a::OIState, b::OIState)
    return a.crash == b.crash && a.ego == b.ego && a.car == b.car
end

#### Observaton type

const OIObs = OIState

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

function OIPOMDP(; env::IntersectionEnv = IntersectionEnv(),
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
    return OIPOMDP(env,
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

function POMDPs.reward(pomdp::OIPOMDP, s::OIState, a::OIAction, sp::OIState)
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
function POMDPs.reward(pomdp::OIPOMDP, s::OIState, a::OIAction)
    return reward(pomdp, s, a, s)
end

function POMDPs.isterminal(pomdp::OIPOMDP, s::OIState)
    return s.crash || s.ego.posF.s >= pomdp.s_goal
end

######## DISTRIBUTION ############################################################################

"""
Concrete type to represent a distribution over state for the OIcludedCrosswalk POMDP Problem
"""
mutable struct OIDistribution
    p::Vector{Float64}
    it::Vector{OIState}
end

OIDistribution() = OIDistribution(Float64[], OIState[])

POMDPs.iterator(d::OIDistribution) = d.it

# transition and observation pdf
function POMDPs.pdf(d::OIDistribution, s::OIState)
    for (i, sp) in enumerate(d.it)
        if sp==s
            return d.p[i]
        end
    end
    return 0.
end

function POMDPs.rand(rng::AbstractRNG, d::OIDistribution)
    ns = sample(d.it, Weights(d.p)) # sample a neighbor state according to the distribution c
    return ns
end

"""
    most_likely_state(d::OIDistribution)
returns the most likely state given distribution d
"""
function most_likely_state(d::OIDistribution)
    val, ind = findmax(d.p)
    return d.it[ind]
end

### HELPERS

function POMDPs.discount(pomdp::OIPOMDP)
    return pomdp.γ
end

function state_to_scene(pomdp::OIPOMDP, s::OIState)
    scene = Scene()
    ego = Vehicle(s.ego, pomdp.ego_type, 1)
    car = Vehicle(s.car, pomdp.car_type, 2)
    push!(scene, ego)
    push!(scene, car)
    return scene
end
