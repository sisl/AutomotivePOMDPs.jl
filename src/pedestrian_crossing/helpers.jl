
function get_state_absent(pomdp::SingleOCFPOMDP, ego_y::Float64, ego_v::Float64)

    (ego_y_state_space,ego_v_state_space) = getEgoDataInStateSpace(pomdp, ego_y, ego_v)
    return SingleOCFState(ego_y_state_space, ego_v_state_space, -10.0, -10.0, 0.0 ,0.0)

end

function get_state_absent(pomdp::SingleOCFPOMDP, s::SingleOCFState)

    (ego_y_state_space,ego_v_state_space) = getEgoDataInStateSpace(pomdp, s.ego_y, s.ego_v)
    return SingleOCFState(ego_y_state_space, ego_v_state_space, -10.0, -10.0, 0.0 ,0.0)

end

function is_state_absent(pomdp::SingleOCFPOMDP, s::SingleOCFState)

    if (s.ped_s == -10.0 && s.ped_T == -10.0 )
        return true;
    else
        return false;
    end

end

function is_observation_absent(pomdp::SingleOCFPOMDP, o::SingleOCFObs)

    if (o.ped_s > pomdp.S_MAX || o.ped_s < pomdp.S_MIN || o.ped_T > pomdp.T_MAX ||  o.ped_T < pomdp.T_MIN )
        return true;
    else
        return false;
    end

end


function getEgoDataInStateSpace(pomdp::SingleOCFPOMDP, y::Float64, v::Float64)
    y_grid = RectangleGrid(pomdp.EGO_Y_RANGE)
    (id, weight) = interpolants(y_grid, [y])
    id_max = indmax(weight)
    ego_y = ind2x(y_grid, id[id_max])[1]
    
    v_grid = RectangleGrid(pomdp.EGO_V_RANGE)
    (id, weight) = interpolants(v_grid, [v])
    id_max = indmax(weight)
    ego_v = ind2x(v_grid, id[id_max])[1]
    return ego_y, ego_v
end