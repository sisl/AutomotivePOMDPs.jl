struct DecUpdater <: Updater
    up::Updater
    problem::UrbanPOMDP
end

const DecBelief = Vector{Tuple{Symbol, Array{Float64, 2}}}

function initialize_dec_belief(up::DecUpdater, pomdp::UrbanPOMDP, rng::AbstractRNG)
    s = initial_state(pomdp, rng)
    o = generate_o(pomdp, s, rng)
    inputs = decompose_input(pomdp, o)
    init_b = initialize_dec_belief(up, inputs)
    return init_b
end

function initialize_dec_belief(up::DecUpdater, dec_o::Vector{Tuple{Symbol, Vector{Float64}}})
    init_b = Tuple{Symbol, Array{Float64,2}}[]
    for (prob_key, obs) in dec_o
        b_stacked = hcat(initialize_belief(up.up, obs)...)
        push!(init_b, (prob_key, b_stacked))
    end
    return init_b
end

function POMDPs.update(up::DecUpdater, b_old::DecBelief, a::UrbanAction, o::Array{Float64})
    b_new = Tuple{Symbol, Array{Float64, 2}}[]
    inputs = decompose_input(up.problem, o)
    for (i,(prob_key, b)) in enumerate(b_old)
        b_vec = [b[:,i] for i=1:up.up.k]
        b_stacked = hcat(update(up.up, b_vec, a, inputs[i][2][:])...)
        push!(b_new, (prob_key, b_stacked))
    end
    return b_new
end

struct DecomposedPolicy{A} <: Policy
    problem::POMDP
    problem_map::Dict
    policy_map::Dict
    fusion
    action_map::Vector{A}
end

# some default fusion function
const SUM = x -> sum(x)
const MIN = x -> minimum(hcat(l), 2)

function POMDPs.action(policy::DecomposedPolicy, beliefs::DecBelief)
    val = value(policy, beliefs)
    @assert length(val) == n_actions(policy.problem)
    return policy.action_map[indmax(val)]
end

function POMDPs.value(policy::DecomposedPolicy, beliefs::DecBelief)
    n = length(beliefs)
    val = fill(zeros(n_actions(policy.problem),1), n)
    for (i, (prob_key, b)) in enumerate(beliefs)
        val[i] = value(policy.policy_map[prob_key], b)
    end
    return policy.fusion(val)
end
