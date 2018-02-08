@with_kw mutable struct IntersectionDriver <: DriverModel{LonAccelDirection}
    a::LonAccelDirection = LonAccelDirection(0., 1)
    navigator::RouteFollowingIDM = RouteFollowingIDM()
    intersection::Vector{Lane} = Lane[]
    intersection_entrances::Vector{Lane} = Lane[]
    intersection_exits::Vector{Lane} = Lane[]
    horizon::Float64 = 20.0 # distance from the intersection to start the logic
    stop_delta::Float64 = 0. # precision for stopping slightly before the line
    accel_tol::Float64 = 1e-2 # if |a| < accel_tol then a = 0.

    # states
    wait_list::Vector{Int} = Int[]
    priority::Bool = false
    stop::Bool = false # switch to true when stopped at the intersection, stays true until the end
    n_yield::Int64 = 0 # number of vehicles to yield to
end

get_name(::IntersectionDriver) = "IntersectionDriver"
function AutomotiveDrivingModels.set_desired_speed!(model::IntersectionDriver, v_des::Float64)
    model.navigator.v_des = v_des
    model
end

function AutomotiveDrivingModels.reset_hidden_state!(model::IntersectionDriver)
    model.priority = false
    model.stop = false
    model.n_yield = 0
    model
end


function AutomotiveDrivingModels.observe!(model::IntersectionDriver, scene::Scene, roadway::Roadway, egoid::Int)
    ego = scene[findfirst(scene, egoid)]
    AutomotiveDrivingModels.observe!(model.navigator, scene, roadway, egoid) # set the direction
    dir = model.navigator.dir
    a_lon =0.
    a_lon_idm = model.navigator.a
    if isempty(model.intersection) # no intersection, go
        a_lon = model.navigator.a
    else
        if !model.priority && !model.stop # reach stop line
            a_lon = min(a_lon_idm, stop_at_end(model, ego, roadway))
        elseif !model.priority && model.stop # wait
            a_lon = -model.navigator.d_max # negative to make sure v stays 0
        else
            a_lon = a_lon_idm # just idm
        end
    end
    if !model.stop
        update_stop!(model, ego, roadway)
    end
    if !model.stop
        grow_wait_list!(model, scene, roadway, egoid)
    else
        ungrow_wait_list!(model, scene, roadway, egoid)
    end
    if !model.priority
        update_priority!(model, scene, roadway, egoid)
    end

    # println("stop ", model.stop)
    # println("priority ", model.priority)
    # println("n_yield ", model.n_yield)
    # println("a_lon ", a_lon)
    # println("v ", ego.state.v)
    model.a = LonAccelDirection(a_lon, dir)
    d_stop = get_dist_to_end(ego, roadway)
    # println("veh ", ego.id, " wait_list ", model.wait_list, " stop ", model.stop, " priority ", model.priority, " dstop ", d_stop, " a_lon ", a_lon, " v ", ego.state.v )
    # println("veh ", ego.id, " taking action ", model.a)
    model
end

"""
If other vehicles reaches the intersection before the ego vehicle they are added to the waitlist
"""
function grow_wait_list!(model::IntersectionDriver, scene::Scene, roadway::Roadway, egoid::Int)
    ego = scene[findfirst(scene, egoid)]
    ego_dist = get_dist_to_end(ego, roadway)
    ego_tts = tts(ego_dist, ego.state.v)
    n_yield = 0
    for veh in scene
        if veh.id == egoid
            continue
        end
        if is_at_intersection(model, veh, roadway)
            veh_dist = get_dist_to_end(veh, roadway)
            veh_tts = tts(veh_dist, veh.state.v)
            if veh_tts < ego_tts
                n_yield += 1
                if !(veh.id ∈ model.wait_list)
                    push!(model.wait_list, veh.id)
                end
            end
        end
    end
    model.n_yield = length(model.wait_list)
end

"""
Remove from the wait list vehicles that are now exiting the intersection
"""
function ungrow_wait_list!(model::IntersectionDriver, scene::Scene, roadway::Roadway, egoid::Int)
    n = length(model.wait_list)
    to_remove = Int64[]
    for i=1:n
        vehid = model.wait_list[i]
        veh_ind = findfirst(scene, vehid)
        if veh_ind == 0
            push!(to_remove, i)
            continue
        end
        veh = scene[veh_ind]
        lane = get_lane(roadway, veh)
        # if !(lane ∈ model.intersection_entrances) &&  veh.state.posF.s > veh.def.length # vehicle has exited for more than 1 car length
        if lane ∈ model.intersection_exits
            push!(to_remove, i)
            model.n_yield -= 1
        end
    end
    deleteat!(model.wait_list, to_remove)
end


"""
Robust function to compute the time to drive dist at speed v
"""
function tts(dist::Float64, v::Float64)
    if isapprox(dist, 0, atol=1e-1)
        return 0
    end
    return dist/v
end

function is_intersection_cleared(model::IntersectionDriver, scene::Scene, roadway::Roadway, egoid::Int, delta::Float64=2.0)
    ego = scene[findfirst(scene, egoid)]
end

"""
return true if veh is before the intersection
"""
function is_at_intersection(model::IntersectionDriver, veh::Vehicle, roadway::Roadway)
    lane = get_lane(roadway, veh)
    return lane ∈ model.intersection_entrances
end

"""
Check if the ego car has priority or not
"""
function update_priority!(model::IntersectionDriver, scene::Scene, roadway::Roadway, egoid::Int)
    ego = scene[findfirst(scene, egoid)]
    model.priority = isempty(model.wait_list) && model.stop ||
                       !(get_lane(roadway, ego) ∈ model.intersection_entrances)
end

Base.rand(model::IntersectionDriver) = model.a

"""
set the state stop! to true if veh is stopped at the intersection
"""
function update_stop!(model::IntersectionDriver, veh::Vehicle, roadway::Roadway)
    dist_to_end = get_dist_to_end(veh, roadway)
    if veh.state.v ≈ 0. && isapprox(dist_to_end, 0, atol=1.0) # parameterize rtol?
        model.stop = true
    end
end

"""
Return the distance to the end of the lane
"""
function get_dist_to_end(veh::Vehicle, roadway::Roadway)
    lane = get_lane(roadway, veh)
    s_end = get_end(lane)
    dist_to_end = s_end - veh.state.posF.s
    # println("dist_to_end ", dist_to_end)
    # println("State ", veh.state.posF)
    return dist_to_end
end

"""
Return the longitudinal acceleration to come to a stop at the end of the current lane
follows idm style braking
"""
function stop_at_end(model::IntersectionDriver, veh::Vehicle, roadway::Roadway)
    # run the IDM model, consider that the stop line is a fixed vehicle
    headway = get_dist_to_end(veh, roadway)
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
        Δv = idm.v_des - v
        acc = Δv*idm.k_spd
    end


    acc = clamp(acc, -idm.d_max, idm.a_max)
    if abs(acc) < model.accel_tol
        acc = 0.
    end
    return acc
end
