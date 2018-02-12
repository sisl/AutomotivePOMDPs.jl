# Implement all the belief related function

const SingleOIBelief = SingleOIDistribution

mutable struct SingleOIUpdater <: Updater
    pomdp::SingleOIPOMDP
end

#for sarsop for exploration
#function POMDPs.initial_state_distribution(pomdp::SingleOIPOMDP)
#    state_space = states(pomdp)
#    states_to_add = SingleOIState[]
#    for s in state_space
#        if !s.crash
#            push!(states_to_add, s)
#        end
#    end
#    probs = ones(length(states_to_add))
#    normalize!(probs, 1)
#    return SingleOIDistribution(probs, states_to_add)
#end

function POMDPs.initial_state_distribution(pomdp::SingleOIPOMDP)
     env = pomdp.env
     v_grid = get_v_grid(pomdp)
     states = SingleOIState[]
     ego = ego_state(pomdp, pomdp.s_start, 0.)
     for lane_id in LANE_ID_LIST
         for s in get_lane_s(pomdp, lane_id)
             for v in v_grid[3:end]
                 car = car_state(pomdp, lane_id, s, v)
                 if !is_observable_fixed(ego, car, env)
                     push!(states, SingleOIState(is_crash(pomdp, ego, car), ego, car))
                 end
             end
         end
     end

     # Add also the off the grid state, weight using the probability p_birth
     n_in_grid = length(states)
     push!(states, SingleOIState(false, ego, get_off_the_grid(pomdp)))
     n_off_grid = length(states) - n_in_grid
     probs = ones(length(states))
     probs[1:n_in_grid] = pomdp.p_birth/n_in_grid
     probs[n_in_grid + 1:end] = (1.0 - pomdp.p_birth)/n_off_grid
     @assert n_off_grid == 1
     @assert sum(probs) ≈ 1.
     # normalize!(probs, 1)
     return SingleOIBelief(probs, states)
end

function initial_ego_state(pomdp::SingleOIPOMDP)
    return ego_state(pomdp, pomdp.s_start, 0.)
end

# Updates the belief given the current action and observation
function POMDPs.update(bu::SingleOIUpdater, bold::SingleOIBelief, a::SingleOIAction, o::SingleOIObs)
    bnew = SingleOIBelief()
    pomdp = bu.pomdp
    # initialize spaces
    pomdp_states = ordered_states(pomdp)

    # iterate through each state in belief vector
    for (i, sp) in enumerate(pomdp_states)
        # get the distributions
        od = observation(pomdp, a, sp)
        # get prob of observation o from current distribution
        probo = pdf(od, o)
        # if observation prob is 0.0, then skip rest of update b/c bnew[i] is zero
        if probo == 0.0
            continue
        end
        b_sum = 0.0 # belief for state sp
        for (j, s) in enumerate(bold.it)
            td = transition(pomdp, s, a)
            pp = pdf(td, sp)
            @inbounds b_sum += pp * bold.p[j]
        end
        if b_sum != 0.
            push!(bnew.it, sp)
            push!(bnew.p, probo*b_sum)
        end
    end

    # if norm is zero, the update was invalid - reset to uniform
    if sum(bnew.p) == 0.0
        println("Invalid update for: ", bold, " ", a, " ", o)
        throw("UpdateError")
    else
        normalize!(bnew.p, 1)
    end
    bnew
end
