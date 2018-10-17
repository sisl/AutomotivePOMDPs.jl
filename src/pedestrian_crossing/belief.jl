
mutable struct SingleOCFUpdater <: Updater
    pomdp::SingleOCFPOMDP
end




const SingleOCFBelief = SparseCat{Vector{SingleOCFState},Vector{Float64}}

function POMDPs.update(up::SingleOCFUpdater, b::SingleOCFBelief, a::SingleOCFAction, o::SingleOCFObs)

    states_p = SingleOCFState[]
    sizehint!(states_p, 100);

    bp = Float64[] 
    sizehint!(bp, 100);
    
    bp_sum = 0.0   # to normalize the distribution
    cnt = 0
    
    (ego_y_state_space, ego_v_state_space) = getEgoDataInStateSpace(pomdp, o.ego_y, o.ego_v)

    # object is outside the defined state space
    if (  o.ped_s > pomdp.S_MAX || o.ped_s < pomdp.S_MIN || o.ped_T > pomdp.T_MAX ||  o.ped_T < pomdp.T_MIN )
        absent_state = pomdp.state_space[end]
        o = pomdp.state_space[end]
        b = initBeliefAbsentPedestrian(pomdp, ego_y_state_space, ego_v_state_space)
        println("Pedestrian is absent: ", o)
    end

    for sp_ped in pomdp.state_space_ped
        #cnt += 1
        #if ( cnt % 1000 == 0)
        #    println("Progress: ", cnt/length(pomdp.state_space_ped)*100)
        #end
   
        if ( sp_ped != pomdp.state_space_ped[end])  # not absent state
            sp = SingleOCFState(ego_y_state_space, ego_v_state_space, sp_ped.ped_s, sp_ped.ped_T, sp_ped.ped_theta ,sp_ped.ped_v)
        else # absent state
            sp = pomdp.state_space[end]
        end
        po = observation_weight(pomdp, sp, o) 


        if po < 0.000000001
            continue
        end

        b_sum = 0.0   
        for (s, prob) in weighted_iterator(b)
            s = SingleOCFState(o.ego_y, o.ego_v, s.ped_s, s.ped_T, s.ped_theta ,s.ped_v)

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
        #return caclulateBeliefBasedOnObservation(pomdp,o)
        error("""
              Failed discrete belief update: new probabilities sum to zero.
            #  b = $b
              a = $a
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


function initBeliefAbsentPedestrian(pomdp::SingleOCFPOMDP, ego_y::Float64, ego_v::Float64)
    

    states = SingleOCFState[]
    sizehint!(states, 2500);
    probs = Float64[] 
    sizehint!(probs, 2500);

    (ego_y_state_space, ego_v_state_space) = getEgoDataInStateSpace(pomdp, ego_y, ego_v)

    # add states on the right side
    for ped_theta in pomdp.PED_THETA_RANGE
        for ped_v in pomdp.PED_V_RANGE
            for ped_s in pomdp.S_RANGE
                push!(states,SingleOCFState(ego_y_state_space, ego_v_state_space, ped_s, pomdp.T_MIN, ped_theta, ped_v))
                push!(states,SingleOCFState(ego_y_state_space, ego_v_state_space, ped_s, pomdp.T_MIN+0.5, ped_theta, ped_v))
                push!(states,SingleOCFState(ego_y_state_space, ego_v_state_space, ped_s, pomdp.T_MIN+1.0, ped_theta, ped_v))

            end
        end
    end
    
    # add absent state
    push!(states,pomdp.state_space[end])

    # add occluded states
    for i = 1:length(pomdp.obstacles)
        ego_pos = VecE2(pomdp.ego_vehicle.state.posG.x, pomdp.ego_vehicle.state.posG.y) 
        (obst_s, obst_T, right_side) = getObstructionCorner(pomdp.obstacles[i], ego_pos )
        #println("obst_s: ", obst_s, " obst_T: ", obst_T , " right_side: ", right_side)   
        if ( right_side ) 
            occluded_positions = calulateHiddenPositionsRightSide(obst_s, obst_T)
            #println(occluded_positions)
            for ped_theta in pomdp.PED_THETA_RANGE
                for ped_v in pomdp.PED_V_RANGE
                    for (hidden_s, hidden_T) in occluded_positions
                        push!(states,SingleOCFState(ego_y_state_space, ego_v_state_space, hidden_s, hidden_T, ped_theta, ped_v))
                    end
                end
            end
        end
    end
    probs = ones(length(states)) / length(states)
    
    return SparseCat(states,probs)
end

function initBeliefPedestrian(pomdp::SingleOCFPOMDP, o::SingleOCFObs)
    
    states = SingleOCFState[]
    sizehint!(states, 2500);
    probs = Float64[] 
    sizehint!(probs, 2500);

    (ego_y_state_space, ego_v_state_space) = getEgoDataInStateSpace(pomdp, o.ego_y, o.ego_v)

    for ped_theta in pomdp.PED_THETA_RANGE
        for ped_v in pomdp.PED_V_RANGE
            for ds = -2:0.5:2
                for dT = -2:0.5:2
                    obs = SingleOCFState(ego_y_state_space, ego_v_state_space, o.ped_s+ds, o.ped_T+dT, ped_theta, ped_v)
                    obs_int = pomdp.state_space[state_index(pomdp, obs)]
                    push!(states, obs_int) 
                end
            end
        end
    end
    probs = ones(length(states)) / length(states)

    return SparseCat(states,probs)
end
