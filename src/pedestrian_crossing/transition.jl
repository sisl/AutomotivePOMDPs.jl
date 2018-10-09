### Transition model  ############################################################################
function POMDPs.transition(pomdp::SingleOCFPOMDP, s::SingleOCFState, a::SingleOCFAction, dt::Float64 = pomdp.ΔT)
    

    # ego transition 
    x_delta_ego = s.ego_v*dt + 0.5*a.acc*dt^2
    v_ego = s.ego_v + a.acc*dt 
    if (v_ego < 0.)
        sp_ego_v = 0.
    else
        sp_ego_v = v_ego
    end
        
    # 0.5m in 0.5s should be fine   
    sp_ego_y = s.ego_y + a.lateral_movement * pomdp.ΔT 
    if ( sp_ego_y > pomdp.EGO_Y_MAX ) 
        sp_ego_y = pomdp.EGO_Y_MAX
    end
    if ( sp_ego_y < pomdp.EGO_Y_MIN ) 
        sp_ego_y = pomdp.EGO_Y_MIN
    end   
    
    ego_y_state_space = ego_y_to_state_space(pomdp, sp_ego_y)
    ego_v_state_space = ego_v_to_state_space(pomdp, sp_ego_v)

    # absent or not
    if (s != pomdp.state_space[end] )
        # pedestrian is absent
        states = SingleOCFState[]
        sizehint!(states, 100);
        probs = Float64[] 
        sizehint!(probs, 100);

        # ped transition
        for v_n in pomdp.PED_V_NOISE
            sp_ped_v = s.ped_v + v_n

            if (sp_ped_v < 0.) 
                sp_ped_v = 0.
            end
            if (sp_ped_v > pomdp.PED_V_MAX ) 
                sp_ped_v = pomdp.PED_V_MAX
            end
            for theta_n in pomdp.PED_THETA_NOISE
                theta_ped = s.ped_theta + theta_n
                sp_ped_theta = theta_ped
                sp_ped_s = s.ped_s + sp_ped_v*cos(theta_ped) - x_delta_ego
                sp_ped_T = s.ped_T + sp_ped_v*sin(theta_ped) - a.lateral_movement * pomdp.ΔT  # a_lat corresponds to lateral velocity --> a_lat == v_lat
                
                #state_vector = SVector{6, Float64}(sp_ego_y, sp_ego_v, sp_ped_s, sp_ped_T, sp_ped_theta, sp_ped_v)

                state_vector = SVector(ego_y_state_space, ego_v_state_space, sp_ped_s, sp_ped_T, sp_ped_theta, sp_ped_v) # looks faster
                ind, weight = interpolants(pomdp.state_space_grid, state_vector)
                for i=1:length(ind)
                    if weight[i] > 0.1
                        state = pomdp.state_space[ind[i]]
                        push!(states, state)
                        push!(probs, weight[i])
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

    else
        # pedestrian is absent
#println("bingo tran absent")
        states = SingleOCFState[]
        sizehint!(states, 2500);
        probs = Float64[] 
        sizehint!(probs, 2500);


        # add states on the right side
        for ped_theta in pomdp.PED_THETA_RANGE
            for ped_v in pomdp.PED_V_RANGE
                for ped_s in pomdp.S_RANGE
                    push!(states,SingleOCFState(ego_y_state_space, ego_v_state_space, ped_s, pomdp.T_MIN, ped_theta, ped_v))
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
    end

    return SparseCat(states,probs)
end



function getObstructionCorner(obstacle::ConvexPolygon, ego_pos::VecE2)
 
    x = Vector{Float64}(obstacle.npts)
    y = Vector{Float64}(obstacle.npts)
    for i = 1:obstacle.npts
        x[i] = obstacle.pts[i].x
        y[i] = obstacle.pts[i].y
    end
    

    delta_s = maximum(x) - ego_pos.x
        
    right_side = true
    if ( ego_pos.y > mean(y) )
        delta_t = -(ego_pos.y -  maximum(y))
        right_side = true
    else
        delta_t = minimum(y) - ego_pos.y 
        right_side = false
    end
    return delta_s, delta_t, right_side 
end


function calulateHiddenPositionsRightSide(obst_s::Float64, obst_T::Float64)

    idx = findfirst(x -> x >= obst_s, pomdp.S_RANGE)
    s_grid = pomdp.S_RANGE[idx:end]
   
    idx = findlast(x -> x < obst_T, pomdp.T_RANGE)
    T_grid = pomdp.T_RANGE[2:idx]

    sT_pos = []
    thetha = atan(obst_T, obst_s)
    for s in s_grid
        dT = tan(thetha)*(s-obst_s)
        for T in T_grid
            if T < obst_T+dT
                push!(sT_pos, [s, T])
            end
        end
    end
    return sT_pos
end

function calulateHiddenPositionsLeftSide(obst_s::Float64, obst_T::Float64)

    idx = findfirst(x -> x >= obst_s, pomdp.S_RANGE)
    s_grid = pomdp.S_RANGE[idx:end]
   
    idx = findfirst(x -> x >= obst_T, pomdp.T_RANGE)
    T_grid = pomdp.T_RANGE[idx:end-1]
    
    sT_pos = []
    thetha = atan(obst_T, obst_s)
    for s in s_grid
        dT = tan(thetha)*(s-obst_s)
        for T in T_grid
            if T > obst_T+dT
                push!(sT_pos, [s, T])
            end
        end
    end
    return sT_pos
end
