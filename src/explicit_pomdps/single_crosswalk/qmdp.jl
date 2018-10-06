function POMDPs.action(policy::QMDPPolicy, b::SingleOCDistribution)
    alphas = policy.alphas' #size |A|x|S|
    util = zeros(n_actions(pomdp)) # size |A|
    for i=1:n_actions(pomdp)
        res = 0.0
        for (j,s) in enumerate(b.it)
            si = stateindex(pomdp, s)
            res += alphas[i, si]*b.p[j]
        end
        util[i] = res
    end
    ihi = indmax(util)
    return policy.action_map[ihi]
end


"""
    POMDPs.value(policy::QMDPPolicy, b::OCDistribution)
Return the value of the belief according to the QMDP policy
"""
function POMDPs.value(policy::QMDPPolicy, b::SingleOCDistribution)
    val = zeros(n_actions(policy.pomdp))
    for (i,s) in enumerate(b.it)
        val += value(policy, s)*b.p[i]
        # si = stateindex(pomdp, s)
        # val += policy.alphas[si, :]*b.p[i]
    end
    return val
end
