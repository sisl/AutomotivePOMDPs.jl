### Observation model  ############################################################################


function observation_weight(pomdp::SingleOCFPOMDP, sp::SingleOCFState, o::SingleOCFObs)

    absent_state = pomdp.state_space[end]
    absent_state_obs = pomdp.state_space[end] #SingleOCFObs(absent_state.ego_y, absent_state.ego_v, absent_state.ped_s, absent_state.ped_T, absent_state.ped_theta, absent_state.ped_v)
   
    if ( o == absent_state_obs )
        o_absent = true
    else
        o_absent = false
    end

    if ( sp == absent_state )
        sp_absent = true
    else
        sp_absent = false
    end

   # pomdp.ego_vehicle.state
   if ( AutomotivePOMDPs.is_observable_fixed(pomdp.ego_vehicle.state, VehicleState(VecSE2(pomdp.ego_vehicle.state.posG.x+sp.ped_s, pomdp.ego_vehicle.state.posG.y+sp.ped_T, 0.), 0.0), pomdp.env) == false )
  #  if ( AutomotivePOMDPs.is_observable_fixed(sp.ped_s, sp.ped_T, VehicleState(VecSE2(0., 0., 0.), 0.0), pomdp.env) == false )
   # if ( AutomotiveSensors.occlusion_checker(VehicleState(VecSE2(0., 0., 0.), 0.0), VehicleState(VecSE2(sp.ped_s, sp.ped_T, 0.), 0.0), pomdp.env.obstacles) == false )
        occluded = true
    else
        occluded = false
    end



    if ( o_absent && sp_absent )
        # absent
       # println("absent")
        return 1.0
    end

    if ( o_absent && occluded )
        # occluded
       # println("occluded")
        return 1.0
    end

    if ( !o_absent && !sp_absent && !o_absent )
        # visible
       # println("visible")

        ob_dist = MultivariateNormal([sp.ped_s, sp.ped_T, sp.ped_theta ,sp.ped_v], 0.5*eye(4))

        (ego_y_state_space, ego_v_state_space) = getEgoDataInStateSpace(pomdp, o.ego_y, o.ego_v)

        obs_cont = SVector(ego_y_state_space, ego_v_state_space, o.ped_s, o.ped_T, o.ped_theta, o.ped_v)
        obs_int, obs_weight = interpolants(pomdp.state_space_grid, obs_cont)

        po = 0.0
        N = length(obs_int)
        for i=1:N
            obs_int_state = pomdp.state_space[obs_int[i]]
            po += obs_weight[i]*pdf(ob_dist, [obs_int_state.ped_s, obs_int_state.ped_T, obs_int_state.ped_theta, obs_int_state.ped_v])
        end
        po = po / N
        return po

    else
       # println("visible-else")       
        return 0.0
    end

end


function POMDPs.observation(pomdp::SingleOCFPOMDP, a::SingleOCFAction, sp::SingleOCFState)
    
    states = SingleOCFObs[]
    sizehint!(states, 100);
    probs = Float64[] 
    sizehint!(probs, 100);
    
    
    push!(states, sp)
    push!(probs, 1)
    
    return SparseCat(states,probs)

end