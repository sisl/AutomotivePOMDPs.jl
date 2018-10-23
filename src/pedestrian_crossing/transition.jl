### Transition model  ############################################################################
function POMDPs.transition(pomdp::SingleOCFPOMDP, s::SingleOCFState, a::SingleOCFAction, dt::Float64 = pomdp.ΔT)
    
    # ego transition 
    v_ego = s.ego_v + a.acc*pomdp.ΔT 
    v_ego = s.ego_v + a.acc*pomdp.ΔT 
    sp_ego_v = clamp(v_ego, 0, v_ego)
  
    x_delta_ego = s.ego_v*pomdp.ΔT + 0.5*a.acc*pomdp.ΔT^2

    # 0.5m in 0.5s should be fine   
    sp_ego_y = s.ego_y + a.lateral_movement * pomdp.ΔT  # simplified movement, not exactly correct -> a=1, t = 1 --> 1m movement to the left
    sp_ego_y = clamp(sp_ego_y, pomdp.EGO_Y_MIN, pomdp.EGO_Y_MAX )

    # absent or not
    if ( !is_state_absent(pomdp, s) )
        # pedestrian is not absent
        states = SingleOCFState[]
        sizehint!(states, 30);
        probs = Float64[] 
        sizehint!(probs, 30);

        # ped transition
        for a_ped in pomdp.PED_A_RANGE
            sp_ped_v = s.ped_v + a_ped * pomdp.ΔT
            sp_ped_v = clamp(sp_ped_v, 0, pomdp.PED_V_MAX)
            if a_ped == 0
                a_ped_prob = 3.0
            else
                a_ped_prob = 1.0
            end
            for theta_n in pomdp.PED_THETA_NOISE

                theta_ped = s.ped_theta + theta_n
          #      println("theta_ped: ", theta_ped, " sp_ped_v*cos(theta_ped): ", sp_ped_v*cos(theta_ped))
                sp_ped_theta = theta_ped
                @fastmath begin
                sp_ped_s = s.ped_s + sp_ped_v*cos(theta_ped) * pomdp.ΔT - x_delta_ego
                sp_ped_T = s.ped_T + sp_ped_v*sin(theta_ped) * pomdp.ΔT - a.lateral_movement * pomdp.ΔT  # a_lat corresponds to lateral velocity --> a_lat == v_lat
                
                sp_ped_s = clamp(sp_ped_s, pomdp.S_MIN, pomdp.S_MAX)
                sp_ped_T = clamp(sp_ped_T, pomdp.T_MIN, pomdp.T_MAX)
                end
           #     println("s.ped_T: ", s.ped_T , " s.ped_v: ", s.ped_v, " a_ped: ", a_ped, " sp_ped_v: ", sp_ped_v, " a_ped * pomdp.ΔT: ", a_ped * pomdp.ΔT, " sp_ped_s: ", sp_ped_s, " sp_ped_T: ", sp_ped_T)

           #=
                (ego_y_state_space,ego_v_state_space) = getEgoDataInStateSpace(pomdp, sp_ego_y, sp_ego_v)
                state_vector = SVector(sp_ped_s, sp_ped_T, sp_ped_theta, sp_ped_v) # looks faster
                @inbounds ind, weight = interpolants(pomdp.state_space_grid_ped, state_vector)
                for i=1:length(ind)
                    if weight[i] > 0.1
                        state = pomdp.state_space_ped[ind[i]]
                        push!(states, SingleOCFState(ego_y_state_space, ego_v_state_space, state.ped_s, state.ped_T, state.ped_theta, state.ped_v))
                        push!(probs, weight[i]*a_ped_prob)
                    end
                end
=#
                state_vector = SVector(sp_ego_y, sp_ego_v, sp_ped_s, sp_ped_T, sp_ped_theta, sp_ped_v) # looks faster
                @inbounds ind, weight = interpolants(pomdp.state_space_grid, state_vector)
                for i=1:length(ind)
                    if weight[i] > 0.1
                        state = pomdp.state_space[ind[i]]
                        push!(states, state)
                        push!(probs, weight[i]*a_ped_prob)
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
        return SparseCat{Vector{SingleOCFState},Vector{Float64}}(states,probs)

    else
        # pedestrian is absent
        return initBeliefAbsentPedestrianBorder(pomdp, sp_ego_y, sp_ego_v)
    end
#=
        states = SingleOCFState[]
        sizehint!(states, 500);
        probs = Float64[] 
        sizehint!(probs, 500);
        (ego_y_state_space,ego_v_state_space) = getEgoDataInStateSpace(pomdp, sp_ego_y, sp_ego_v)

        # 
        s_min = pomdp.S_MIN
        if ( pomdp.env.params.obstacles_visible )
            for i = 1:length(pomdp.env.params.obstacles)
                ego_pos = VecE2(pomdp.ego_vehicle.state.posG.x, pomdp.ego_vehicle.state.posG.y) 
                (obst_s, obst_T, right_side) = getObstructionCorner(pomdp.env.params.obstacles[i], ego_pos )
                if ( right_side ) 
                    s_min = clamp(obst_s, pomdp.S_MIN, pomdp.S_MAX)
                end
            end
        end
#println(s_min)
        # add states on the right side
        for ped_theta in pomdp.PED_THETA_RANGE
            for ped_v in pomdp.PED_V_RANGE
                for ped_s in pomdp.S_RANGE
                    if ( ped_s > 100)
                        push!(states,SingleOCFState(ego_y_state_space, ego_v_state_space, ped_s, pomdp.T_MIN, ped_theta, ped_v))
                    end
                end
            end
        end
                
        # add absent state
       # absent_state = get_state_absent(pomdp, ego_y_state_space, pomdp.ego_vehicle.state.v)
        absent_state = get_state_absent(pomdp, pomdp.ego_vehicle.state.posF.t, pomdp.ego_vehicle.state.v)
        push!(states, absent_state)

        probs = ones(length(states))
        probs[1:end - 1] = pomdp.pedestrian_birth / length(states)
        probs[end] = 1.0 - pomdp.pedestrian_birth

        #probs = ones(length(states)) / length(states)

    end

    return SparseCat{Vector{SingleOCFState},Vector{Float64}}(states,probs)
=#
end
