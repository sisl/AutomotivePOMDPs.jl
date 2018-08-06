## Model to evaluate QMDP Policy

mutable struct QMDPEval <: Policy
    env::CrosswalkEnv
    pomdp::OCPOMDP
    policy::Policy
    a::Float64
    tick::Int64
    Δt::Float64
end

function QMDPEval(env::CrosswalkEnv, pomdp::OCPOMDP, policy::Policy)
    return QMDPEval(env, pomdp, policy, 0., 0, pomdp.ΔT)
end

function reset_policy!(policy::QMDPEval)
    policy.tick = 0
    policy.a = 0.
end

function action(policy::QMDPEval, b::Dict{Int64, OCDistribution},  verbose::Bool = false)
    policy.a = POMDPs.action(policy.policy, b).acc
    policy.tick += 1
    return policy.a
end
