

function initialize_dec_belief(up::KMarkovUpdater, dec_o::Vector{Any})
    init_b = []
    for (prob_key, obs) in dec_o
        push!(init_b, (prob_key, initialize_belief(up, obs)))
    end
    return init_b
end


struct DecomposedPolicy{A} <: Policy
    problem::POMDP
    problem_map::Dict{Any, POMDP}
    policy_map::Dict{Any, Policy}
    fusion
    action_map::Vector{A}
end

# some default fusion function
const SUM = x -> sum(x)
const MIN = x -> minimum(hcat(l), 2)

function POMDPs.value{B}(policy::DecomposedPolicy, b::B)
    # split the belief into subproblem
    dec_belief = decompose_input(policy.problem, b)
    return value(policy, dec_belief)
end

function POMDPs.action{B}(policy::DecomposedPolicy, b::B)
    val = value(policy, b)
    @assert length(val) == n_actions(policy.problem)
    return policy.action_map[indmax(val)]
end

function POMDPs.value{B}(policy::DecomposedPolicy, beliefs::Vector{Tuple{Any, B}})
    val = fill(zeros(n_actions(policy.problem)), n)
    for (prob_key, b) in beliefs
        policy_key = get_policy_key(prob_key)
        val[i] = value(policy.policies[policy_key], b)
    end
    return policy.fusion(val)
end
