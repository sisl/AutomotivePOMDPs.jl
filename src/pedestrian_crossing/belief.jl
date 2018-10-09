
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
    cnt = 0
    
    ego_y_o_state_space = ego_y_to_state_space(pomdp, o.ego_y)
    ego_v_o_state_space = ego_v_to_state_space(pomdp, o.ego_v)
    
    if (  o.ped_s > pomdp.S_MAX || o.ped_s < pomdp.S_MIN || o.ped_T > pomdp.T_MAX ||  o.ped_T < pomdp.T_MIN )
        absent_state = pomdp.state_space[end]
        o = SingleOCFObs(absent_state.ego_y, absent_state.ego_v, absent_state.ped_s, absent_state.ped_T, absent_state.ped_theta, absent_state.ped_v)

    else
        obs_cont = SVector(ego_y_o_state_space, ego_v_o_state_space, o.ped_s, o.ped_T, o.ped_theta, o.ped_v)
        obs_int, obs_weight = interpolants(pomdp.state_space_grid, obs_cont)
    end
    
#println("observation cont: ", o)
#println("observation discrete: ", pomdp.state_space[obs_int])

#println("obs_int_id: ", obs_int)
#println("cont_weights: ", obs_weight)  
println("observation: ", o)
    for sp_ped in pomdp.state_space_ped
        cnt += 1
        if ( cnt % 1000 == 0)
            println(cnt)
        end
   
        if ( sp_ped != pomdp.state_space_ped[end]) 
            # not absent state
            sp = SingleOCFState(ego_y_o_state_space, ego_v_o_state_space, sp_ped.ped_s, sp_ped.ped_T, sp_ped.ped_theta ,sp_ped.ped_v)
        else
            # absent state
            sp = SingleOCFState(0., 0., -10., -10., 0., 0.)
            po = observation_weight(pomdp, sp, o) 

        end
        po = observation_weight(pomdp, sp, o) 


        if po < 0.00001
            continue
        end

        b_sum = 0.0   
        for (s, prob) in weighted_iterator(b)
            s = SingleOCFState(o.ego_y, o.ego_v, s.ped_s, s.ped_T, s.ped_theta ,s.ped_v)

            
            #debug
            #=
            if ( s == SingleOCFState(0., 0., -10., -10., 0., 0.))
                println("s: ", s)
                println("sp: ", sp)
                println("bingo debug")
                td = transition(pomdp, s, a) 
               # println(td) 
                pp = pdf(td, sp)
                println(pp)
                println(prob)


                b_sum += pp * prob

            end
            =#
            #debug end

            td = transition(pomdp, s, a)  
            pp = pdf(td, sp)
            b_sum += pp * prob
        end
        
        if b_sum > 0. && po * b_sum > 0.000001
            push!(states_p, sp)
            push!(bp, po * b_sum) 
            bp_sum += po * b_sum
        end
 
    end

    if bp_sum == 0.0
        #return caclulateBeliefBasedOnObservation(pomdp,o)
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
    

    bp = ones(length(pomdp.state_space_ped)) / length(pomdp.state_space_ped)

    states = SingleOCFState[]
    sizehint!(states, length(pomdp.state_space_ped))
    
    for sp_ped in pomdp.state_space_ped

        if ( sp_ped != pomdp.state_space_ped[end]) 
            # not absent state
            state = SingleOCFState(ego_y, ego_v, sp_ped.ped_s, sp_ped.ped_T, sp_ped.ped_theta ,sp_ped.ped_v)
        else
            # absent state
            state = SingleOCFState(0., 0., sp_ped.ped_s, sp_ped.ped_T, sp_ped.ped_theta ,sp_ped.ped_v)
        end
        push!(states,state)
    end
        
  #  states = [pomdp.state_space[end]]
  #  bp = [1]

    return SingleOCFBelief(states, bp)  
end



function AutomotivePOMDPs.action(policy::AlphaVectorPolicy, b::SingleOCFBelief)
    alphas = policy.alphas 
    util = zeros(n_actions(pomdp)) 
    for i=1:n_actions(pomdp)
        res = 0.0
        for (j,s) in enumerate(b.vals)
            si = state_index(pomdp, s)
            res += alphas[i][si]*b.probs[j]
        end
        util[i] = res
    end
    ihi = indmax(util)
    return policy.action_map[ihi]
end