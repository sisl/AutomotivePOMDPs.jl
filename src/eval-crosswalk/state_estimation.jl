
type MixedUpdater <: Updater
    problem::OCPOMDP
    dt::Float64
    states::Vector{OCState}
end

function MixedUpdater(problem::OCPOMDP, dt::Float64 = 0.1)
    return MixedUpdater(problem, dt, states(problem))
end

# efficient version of the discrete updater
function POMDPs.update(bu::MixedUpdater, bold::OCBelief, a::OCAction, o::OCObs)
    bnew = OCBelief()
    pomdp = bu.problem

    ego = o.ego
    Y = get_Y_grid(pomdp)
    V_ped = get_V_ped_grid(pomdp) # try to avoid collect
    n_ped_states = length(Y)*length(V_ped) + 1
    saved_prob = 0.
    for i=1:n_ped_states
        if i == n_ped_states
            ped = get_off_the_grid(pomdp)
        else
            k, l = ind2sub((length(Y), length(V_ped)), i)
            y = Y[k]
            v = V_ped[l]
            ped = yv_to_state(pomdp, y, v)
        end
        sp = OCState(is_crash(pomdp, ego, ped), ego, ped)
        probo = obs_weight(o, sp, pomdp)
        # if observation prob is 0.0, then skip rest of update b/c bnew[i] is zero
        if probo ≈ 0.
            continue
        elseif saved_prob < probo
            saved_prob = probo
        end

        b_sum = 0.0 # belief for state sp
        for (j, s) in enumerate(bold.it)
            td = transition(pomdp, s, a, bu.dt)
            # use pdf on the pedestrian only
            pp = partial_pdf(td, sp)
            b_sum += pp * bold.p[j]
        end
        if b_sum != 0.
            push!(bnew.it, sp)
            push!(bnew.p, probo*b_sum)
        end
    end
    # if norm is zero, the update was invalid
    if sum(bnew.p) == 0.0
        # println("Invalid update for: ", bold, " ", a, " ", o)
        # val, ind = findmax(bold.p)
        # ml_state = bold.it[ind]
        println("Invalid update for: observation ", o,
                "\n action ", a,
                "\n prob", saved_prob,
                "\n old belief ", bold)
        # bnew = OCDistribution([1.0], [o])
        throw("UpdateError")
        # println("Invalid update, probo = $probo")
        # bnew = observation(pomdp, a, o)
    else
        normalize!(bnew.p, 1)
    end
    bnew
end

function partial_pdf(d::OCDistribution, sp::OCState)
    for (i, s) in enumerate(d.it)
        if sp.ped == s.ped
            return d.p[i]
        end
    end
    return 0.
end

"""
    function obs_weight(o::OCState, s::OCState, pomdp::OCPOMDP)
Given a continuous observation returns a weight proportional to the probability of observing o
while being in the state s
"""
function obs_weight(o::OCState, s::OCState, pomdp::OCPOMDP)
    weight = 1.0
    if !is_observable(s.ped, s.ego, pomdp.env)
        if off_the_grid(pomdp, o.ped)
            return 3*weight
        else
            return 0.
        end
    else
        if off_the_grid(pomdp, o.ped) || !is_observable(o.ped, o.ego, pomdp.env)
            return 0.
        else
            pos_noise = pomdp.pos_obs_noise
            vel_noise = pomdp.vel_obs_noise
            weight *= 1/(sqrt(2*pi*pos_noise*vel_noise))*exp(-1/pos_noise*(s.ped.posG.y - o.ped.posG.y)^2 - 1/vel_noise*(s.ped.v - o.ped.v)^2)
            return weight
        end
    end
    return float(NaN)
end

"""
    initialize_off_belief(pomdp::OCPOMDP)
Initialize a belief on all the non observable position
"""
function initialize_off_belief(policy::QMDPEval, ego::VehicleState)
    b = Dict{Int64, OCDistribution}()
    b[OFF_KEY] = initial_state_distribution(policy.pomdp, ego)
    return b
end

# function POMDPs.initial_state_distribution(pomdp::OCPOMDP, n::Int64)
#     init_dis = DecBelief()
#     for i=1:n
#         init_dis[i+1] = initial_state_distribution(pomdp)
#     end
#     return init_dis
# end

function POMDPs.update(up::MixedUpdater, bold::Dict{Int64, OCDistribution}, a::Float64, o::Dict{Int64, OCObs})
    bnew = Dict{Int64, OCDistribution}()
    for oid in keys(o)
        if haskey(bold, oid) && oid != OFF_KEY
            bnew[oid] = update(up, bold[oid], OCAction(a), o[oid])
        elseif oid == OFF_KEY
            bnew[OFF_KEY] = update(up, bold[OFF_KEY], OCAction(a), o[oid])
        else # cars appeared
            bnew[oid] = update(up, bold[OFF_KEY], OCAction(a), o[oid])
        end
    end
    # print_b_stats(up.problem, bnew[OFF_KEY])
    return bnew
end

function print_b_stats(pomdp::OCPOMDP, b::OCDistribution)
    off_prob = 0.
    for (i,s) in enumerate(b.it)
        if off_the_grid(pomdp, s.ped)
            off_prob = b.p[i]
        end
    end
    max_s = most_likely_state(b)
    println("States to consider: $(length(b.it)) ; off prob $off_prob, Most likely state $max_s ")
end
