### Transition model  ############################################################################
function POMDPs.transition(pomdp::SingleOCFPOMDP, s::SingleOCFState, a::SingleOCFAction, dt::Float64 = pomdp.ΔT)
    

   # ego transition 
    x_delta_ego = s.ego_v*dt + 0.5*a.acc*dt^2
    v_ego = s.ego_v + a.acc*dt 
    if (v_ego < 0)
        sp_ego_v = 0
    else
        sp_ego_v = v_ego
    end
    
 # 0.5m in 0.5s should be fine   
    sp_ego_y = s.ego_y + a.lateral_movement * 0.5  
    if ( sp_ego_y > 1 ) 
        sp_ego_y = 1
    end
    if ( sp_ego_y < -1 ) 
        sp_ego_y = -1
    end    

 
    states = SingleOCFState[]
    sizehint!(states, 100);
    probs = Float64[] 
    sizehint!(probs, 100);

    
    cnt = 0
   # ped transition
    for v_n in pomdp.v_noise
        sp_ped_v = s.ped_v + v_n

        if (sp_ped_v < 0) 
            sp_ped_v = 0
        end
        if (sp_ped_v > 3) 
            sp_ped_v = 3
        end
        for theta_n in pomdp.theta_noise
            theta_ped = s.ped_theta + theta_n
            sp_ped_theta = theta_ped
            sp_ped_s = s.ped_s + sp_ped_v*cos(theta_ped) - x_delta_ego
            sp_ped_T = s.ped_T + sp_ped_v*sin(theta_ped)
            
            state_vector = SVector(sp_ego_y, sp_ego_v, sp_ped_s, sp_ped_T, sp_ped_theta, sp_ped_v)
            
            ind, weight = interpolants(pomdp.state_space_grid, state_vector)
            for i=1:length(ind)
                if weight[i] > 0.1
                    state = pomdp.state_space[ind[i]]

                    cnt = cnt + 1
              #      if !(state in states) # check for doublons
                        push!(states, state)
                        push!(probs, weight[i])
                  #  println(weight[i])
               #     else
                #        state_ind = find(x->x==state, states)
                 #       probs[state_ind] += weight[i]
                  #  end
                end
            end
            
        end
    end


    # add roughening
    if length(probs) > 1
        normalize!(probs, 1)
        probs += maximum(probs)
        normalize!(probs,1)
    end
    
    #println(cnt)
    
    return SparseCat(states,probs)
end