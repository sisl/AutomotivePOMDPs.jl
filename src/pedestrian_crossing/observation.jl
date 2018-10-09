### Observation model  ############################################################################


function observation_weight(pomdp::SingleOCFPOMDP, sp::SingleOCFState, o::SingleOCFObs)

    absent_state = pomdp.state_space[end]
    absent_state_obs = SingleOCFObs(absent_state.ego_y, absent_state.ego_v, absent_state.ped_s, absent_state.ped_T, absent_state.ped_theta, absent_state.ped_v)
   
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

    if ( AutomotivePOMDPs.is_observable_fixed(sp.ped_s, sp.ped_T, VehicleState(VecSE2(0., 0., 0.), 0.0), pomdp.env) == false )
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

        ob_dist = MultivariateNormal([sp.ped_s, sp.ped_T, sp.ped_theta ,sp.ped_v], 0.1*eye(4))

        ego_y_o_state_space = ego_y_to_state_pace(pomdp, o.ego_y)
        ego_v_o_state_space = ego_v_to_state_pace(pomdp, o.ego_v)
        obs_cont = SVector(ego_y_o_state_space, ego_v_o_state_space, o.ped_s, o.ped_T, o.ped_theta, o.ped_v)
        obs_int, obs_weight = interpolants(pomdp.state_space_grid, obs_cont)

        po = 0.0
        N = length(obs_int)
        for i=1:N
            obs_int_state = pomdp.state_space[obs_int[i]]
            po += obs_weight[i]*pdf(ob_dist, [obs_int_state.ped_s, obs_int_state.ped_T, obs_int_state.ped_theta, obs_int_state.ped_v])
        end
        po = po / N

    else
       # println("visible-else")       
        return 0.0
    end



end

#=
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

=#
