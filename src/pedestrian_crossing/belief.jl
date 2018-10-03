
mutable struct SingleOCFUpdater <: Updater
    pomdp::SingleOCFPOMDP
end




const SingleOCFBelief = SparseCat{Vector{SingleOCFState},Vector{Float64}}


function POMDPs.update(up::SingleOCFUpdater, b::SingleOCFBelief, a::SingleOCFAction, o::SingleOCFObs)

    states_p = SingleOCFState[]
    sizehint!(states_p, n_states(pomdp));

    bp = Float64[] 
    sizehint!(bp, n_states(pomdp));
    
    bp_sum = 0.0   # to normalize the distribution
    
    for sp_ped in pomdp.state_space_ped

        sp = SingleOCFState(o.ego_y, o.ego_v, sp_ped.ped_s, sp_ped.ped_T, sp_ped.ped_theta ,sp_ped.ped_v)

        # po = O(a, sp, o)
        od = observation(pomdp, a, sp)
        po = pdf(od, o)
  
        if po == 0.0
            continue
        end

        b_sum = 0.0
        for (s, prob) in weighted_iterator(b)
            td = transition(pomdp, s, a)
            pp = pdf(td, sp)
            b_sum += pp * prob
        end
        
        if b_sum > 0.
            push!(states_p, sp)
            push!(bp, po * b_sum) 
            bp_sum += po * b_sum
        end
        
    end
    if bp_sum == 0.0
        error("""
              Failed discrete belief update: new probabilities sum to zero.
              b = $b
              a = $a6
              o = $o
              Failed discrete belief update: new probabilities sum to zero.
              """)
        
    else
        bp ./= bp_sum
    end

    return SingleOCFBelief(states_p, bp)  
    
end



function POMDPs.initial_state_distribution(pomdp::SingleOCFPOMDP)
    
    bp = ones(n_states(pomdp)) / n_states(pomdp)

    states = SingleOCFState[]
    sizehint!(states, n_states(pomdp))
    
    for state in pomdp.state_space
        push!(states,state)
    end
        
    return SingleOCFBelief(states, bp)  
end

function initial_state_distribution_ego_known(pomdp::SingleOCFPOMDP, ego_y::Float64, ego_v::Float64)
    
    pomdp.state_space_ped

    bp = ones(length(pomdp.state_space_ped)) / length(pomdp.state_space_ped)

    states = SingleOCFState[]
    sizehint!(states, length(pomdp.state_space_ped))
    
    for sp_ped in pomdp.state_space_ped
           state = SingleOCFState(ego_y, ego_v, sp_ped.ped_s, sp_ped.ped_T, sp_ped.ped_theta ,sp_ped.ped_v)
        push!(states,state)
    end
        
    return SingleOCFBelief(states, bp)  
end