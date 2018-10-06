
type MixedUpdater <: Updater
    problem::SingleOCPOMDP
    dt::Float64
    states::Vector{SingleOCState}
end

function MixedUpdater(problem::SingleOCPOMDP, dt::Float64 = 0.1)
    return MixedUpdater(problem, dt, states(problem))
end

# efficient version of the discrete updater
function POMDPs.update(bu::MixedUpdater, bold::SingleOCBelief, a::SingleOCAction, o::SingleOCObs)
    bnew = SingleOCBelief()
    pomdp = bu.problem

    ego = o.ego
    Y = AutomotivePOMDPs.get_Y_grid(pomdp)
    V_ped = AutomotivePOMDPs.get_V_ped_grid(pomdp) # try to avoid collect
    n_ped_states = length(Y)*length(V_ped) + 1
    saved_prob = 0.
    for i=1:n_ped_states
        if i == n_ped_states
            ped = get_off_the_grid(pomdp)
        else
            k, l = Tuple(CartesianIndices((length(Y), length(V_ped)))[i])
            y = Y[k]
            v = V_ped[l]
            ped = AutomotivePOMDPs.yv_to_state(pomdp, y, v)
        end
        collision = collision_checker(ego, ped, pomdp.ego_type, pomdp.ped_type)
        sp = SingleOCState(collision, ego, ped)
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
        # val, ind = fargmax(bold.p)
        # ml_state = bold.it[ind]
        println("Invalid update for: observation ", o,
                "\n action ", a,
                "\n prob", saved_prob,
                "\n old belief ", bold)
        # bnew = SingleOCDistribution([1.0], [o])
        throw("UpdateError")
        # println("Invalid update, probo = $probo")
        # bnew = observation(pomdp, a, o)
    else
        normalize!(bnew.p, 1)
    end
    bnew
end

function partial_pdf(d::SingleOCDistribution, sp::SingleOCState)
    for (i, s) in enumerate(d.it)
        if sp.ped == s.ped
            return d.p[i]
        end
    end
    return 0.
end


"""
    initialize_off_belief(pomdp::OCPOMDP)
Initialize a belief on all the non observable position
"""
function initialize_off_belief(policy::QMDPEval, ego::VehicleState)
    b = Dict{Int64, SingleOCDistribution}()
    b[OFF_KEY] = initialstate_distribution(policy.pomdp, ego)
    return b
end

# function POMDPs.initialstate_distribution(pomdp::OCPOMDP, n::Int64)
#     init_dis = DecBelief()
#     for i=1:n
#         init_dis[i+1] = initialstate_distribution(pomdp)
#     end
#     return init_dis
# end

function POMDPs.update(up::MixedUpdater, bold::Dict{Int64, SingleOCDistribution}, a::Float64, o::Dict{Int64, SingleOCObs})
    bnew = Dict{Int64, SingleOCDistribution}()
    for oid in keys(o)
        if haskey(bold, oid) && oid != OFF_KEY
            bnew[oid] = update(up, bold[oid], SingleOCAction(a), o[oid])
        elseif oid == OFF_KEY
            bnew[OFF_KEY] = update(up, bold[OFF_KEY], SingleOCAction(a), o[oid])
        else # cars appeared
            bnew[oid] = update(up, bold[OFF_KEY], SingleOCAction(a), o[oid])
        end
    end
    # print_b_stats(up.problem, bnew[OFF_KEY])
    return bnew
end

function print_b_stats(pomdp::SingleOCPOMDP, b::SingleOCDistribution)
    off_prob = 0.
    for (i,s) in enumerate(b.it)
        if off_the_grid(pomdp, s.ped)
            off_prob = b.p[i]
        end
    end
    max_s = most_likely_state(b)
    println("States to consider: $(length(b.it)) ; off prob $off_prob, Most likely state $max_s ")
end
