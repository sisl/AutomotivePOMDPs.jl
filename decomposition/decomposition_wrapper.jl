struct DecomposedPolicy{A} <: Policy
    problem::POMDP
    subproblems::Dict{Any, POMDP}
    policies::Dict{Any, Policy}
    fusion
    action_map::Vector{A}
end

# some default fusion function
const SUM = x -> sum(x)
const MIN = x -> minimum(hcat(l), 2)

function value{B}(policy::DecomposedPolicy, b::B)
    # split the belief into subproblem
    dec_belief = Dict()
    for problem_id, sub_policy in policy.policies
        dec_belief[problem_id] = extract_input(policy.subproblems[problem_id], b)
    end
    return value(policy, dec_belief)
end

function action{B}(policy::DecomposedPolicy, b::B)
    val = value(policy, b)
    @assert length(val) == n_actions(policy.problem)
    return policy.action_map[indmax(val)]
end

function value{B}(policy::DecomposedPolicy, beliefs::Vector{Tuple{Any, B}})
    val = fill(zeros(n_actions(policy.problem)), n)
    for prob_key, b in beliefs
        policy_key = get_policy_key(prob_key)
        val[i] = value(policy.policies[policy_key], b)
    end
    return policy.fusion(val)
end
