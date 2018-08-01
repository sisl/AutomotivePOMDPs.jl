
"""
Return the longitudinal acceleration to come to a stop at the end of the current lane
follows idm style braking
"""
function stop_at_end(model::DriverModel, veh::Vehicle, roadway::Roadway)
    # run the IDM model, consider that the stop line is a fixed vehicle
    headway = get_dist_to_end(veh, roadway)
    return stop_at_dist(model, veh, headway)
end

"""
Return the longitudinal acceleration to come to a stop at the distance `headway` following idm
"""
function stop_at_dist(model::DriverModel, veh::Vehicle, headway::Float64)
    s_min = model.stop_delta
    acc = Inf
    v_des = 0.
    v = veh.state.v
    idm = model.navigator
    if headway > 0.0
        Δv = v_des - v
        s_des = s_min + v*idm.T - v*Δv / (2*sqrt(idm.a_max*idm.d_cmf))
        v_ratio = idm.v_des > 0.0 ? (v/idm.v_des) : 1.0
        acc = idm.a_max * (1.0 - v_ratio^idm.δ - (s_des/headway)^2)
    else # probably unnecessary
        Δv = v_des - v
        acc = Δv*idm.k_spd
        acc = -model.navigator.d_max
    end

    acc = clamp(acc, -idm.d_max, idm.a_max)
    if abs(acc) < model.accel_tol
        acc = 0.
    end
    return acc
end

"""
set the state stop! to true if veh is stopped at the intersection
"""
function update_stop!(model::DriverModel, veh::Vehicle, roadway::Roadway)
    dist_to_end = get_dist_to_end(veh, roadway)
    if veh.state.v ≈ 0. && isapprox(dist_to_end - model.stop_delta, 0, atol=0.5) # parameterize rtol?
        model.stop = true
    end
end
