@with_kw mutable struct CrosswalkDriver <: DriverModel{LonAccelDirection}
    a::LonAccelDirection = LonAccelDirection(0., 1)
    navigator::RouteFollowingIDM = RouteFollowingIDM()
    crosswalk::Lane = Lane()
    conflict_lanes::Vector{Lane} = Lane[]
    intersection_entrances::Vector{Lane} = Lane[]
    ped_model::ConstantPedestrian = ConstantPedestrian()
    ped_start::Float64 = 4.0 # assume a 5meter buffer before a pedestrian reaches the road
    stop_delta::Float64 = 0.7
    accel_tol::Float64 = 0.1
    d_tol::Float64 = 0.5

    #states
    yield::Bool = true
    priority::Bool = false
    stop::Bool = false
    wait_list::Vector{Int64} = Int64[]

    # transition
    clear::Bool = false

    debug::Bool = false
end

function AutomotiveDrivingModels.reset_hidden_state!(model::CrosswalkDriver)
    model.priority = false
    model.stop = false
    model.clear = false 
    model
end


AutomotiveDrivingModels.get_name(::CrosswalkDriver) = "CrosswalkDriver"
Base.rand(model::CrosswalkDriver) = model.a

function AutomotiveDrivingModels.observe!(model::CrosswalkDriver, scene::Scene, roadway::Roadway, egoid::Int)
    ego = scene[findfirst(scene, egoid)]
    AutomotiveDrivingModels.observe!(model.navigator, scene, roadway, egoid) # set the direction
    dir = model.navigator.dir
    a_lon =0.
    a_lon_idm = model.navigator.a
    dist_to_cw = get_distance_to_crosswalk(model, ego, roadway, -model.stop_delta)
    if !model.yield
        a_lon = a_lon_idm
        # println("veh ", egoid, " not yielding to crosswalk ", model.crosswalk.tag)
    else
        if !model.priority && !model.stop # reach stop line
            a_lon = min(a_lon_idm, stop_at_dist(model, ego, dist_to_cw))
        elseif !model.priority && model.stop # wait
            a_lon = -model.navigator.d_max # negative to make sure v stays 0
        else
            a_lon = a_lon_idm # just idm
        end
    end
    if !model.stop
        update_stop!(model, ego, roadway, dist_to_cw)
    end
    grow_wait_list!(model, scene, roadway, egoid)
    ungrow_wait_list!(model, scene, roadway, egoid)
    update_priority!(model, scene, roadway, egoid)

    if model.debug
        println("ID ", egoid, " priority ", model.priority, " stop ", model.stop, " a ", a_lon, " wait list ", model.wait_list, " dist_to_cw ", dist_to_cw)
    end
    model.a = LonAccelDirection(a_lon, dir)
end

"""
Check if all the pedestrian have crossed
"""
function update_priority!(model::CrosswalkDriver, scene::Scene, roadway::Roadway, egoid::Int)
    ego = scene[findfirst(scene, egoid)]
    model.priority = isempty(model.wait_list) || has_passed(model, ego, roadway)
end

function has_passed(model::CrosswalkDriver, ego::Vehicle, roadway::Roadway)
    cw_length = get_end(model.crosswalk)
    cw_center = get_posG(Frenet(model.crosswalk, cw_length/2), roadway)
    cw_to_car = ego.state.posG - cw_center 
    car_vec = get_front(ego) - ego.state.posG
    has_passed = dot(cw_to_car, car_vec) > 0.
    return has_passed
end

function has_passed(model::CrosswalkDriver, ego::VehicleState, roadway::Roadway)
    cw_length = get_end(model.crosswalk)
    cw_center = get_posG(Frenet(model.crosswalk, cw_length/2), roadway)
    cw_to_car = ego.posG - cw_center 
    car_vec = polar(1., ego.posG.θ) - ego.posG + ego.posG
    has_passed = dot(cw_to_car, car_vec) > 0.
    return has_passed
end

function grow_wait_list!(model::CrosswalkDriver, scene::Scene, roadway::Roadway, egoid::Int)
    for veh in scene
        if veh.def.class == AgentClass.PEDESTRIAN &&
           is_crossing(veh, model.crosswalk, model.conflict_lanes, scene, roadway) &&
           !(veh.id ∈ model.wait_list)
            push!(model.wait_list, veh.id)
        end
    end
end

function ungrow_wait_list!(model::CrosswalkDriver, scene::Scene, roadway::Roadway, egoid::Int)
    n = length(model.wait_list)
    to_remove = Int64[]
    for i=1:n
        pedid = model.wait_list[i]
        ped_ind = findfirst(scene, pedid)
        if ped_ind == 0
            push!(to_remove, i)
            continue
        end
        ped = scene[ped_ind]
        if !is_crossing(ped, model.crosswalk, model.conflict_lanes, scene, roadway)
            push!(to_remove, i)
        end
    end
    deleteat!(model.wait_list, to_remove)
end

function is_crossing(ped::Vehicle, crosswalk::Lane, conflict_lanes::Vector{Lane}, scene::Scene, roadway::Roadway)
    # check if the pedestrian is in the conflict zone
    ped_lane = get_lane(roadway, ped)
    if ped_lane.tag != crosswalk.tag
        return false 
    end
    for lane in conflict_lanes
        ped_f = Frenet(ped.state.posG, lane, roadway)
        if abs(ped_f.t) < lane.width/2 && get_lane(roadway, ped).tag == crosswalk.tag
            return true
        end
    end
    # at this point, the pedestrian is not on the road
    # check if the pedestrian is going to cross or not
    if AutomotivePOMDPs.direction_from_center(ped, crosswalk) > 0. && get_lane(roadway, ped).tag == crosswalk.tag && is_free(crosswalk, scene, roadway)
        return true
    end
    return false
end

function is_free(crosswalk::Lane, scene::Scene, roadway::Roadway)
    #check that no vehicle is stopped on the crosswalk
    for veh in scene 
        if veh.def.class == AgentClass.PEDESTRIAN
            continue 
        end
        posG = veh.state.posG 
        cw_posF = Frenet(posG, crosswalk, roadway)
        lane = get_lane(roadway, veh)
        t_car = cw_posF.t 
        if abs(cw_posF.t) < crosswalk.width/2 + veh.def.length/2*sin(cw_posF.ϕ)
            return false 
        end
    end
    return true
end

function get_distance_to_crosswalk(model::CrosswalkDriver, veh::VehicleState, roadway::Roadway, delta::Float64 = 0.)
    lane = get_lane(roadway, veh)
    Δs = 0.
    # if lane ∈ model.conflict_lanes
    #     cw_length = get_end(model.crosswalk)
    #     cw_center = get_posG(Frenet(model.crosswalk, cw_length/2), roadway)
    #     collision_point = VecSE2(cw_center.x, veh.posG.y)
    #     cw_proj = Frenet(collision_point, lane, roadway)
    #     Δs = cw_proj.s - veh.posF.s + delta - model.crosswalk.width
    # elseif lane ∈ model.intersection_entrances # stop before the intersection to not block traffic
    #     Δs = get_end(lane) - veh.posF.s + delta - model.crosswalk.width
    # end
    if lane ∈ model.intersection_entrances # stop before the intersection to not block traffic
        Δs = get_end(lane) - veh.posF.s + delta - model.crosswalk.width
    elseif lane ∈ model.conflict_lanes
        cw_length = get_end(model.crosswalk)
        cw_start = model.crosswalk.curve[1].pos
        cw_center = get_posG(Frenet(model.crosswalk, cw_length/2), roadway)
        cw_to_car = veh.posG - cw_center 
        car_vec = polar(1., veh.posG.θ) - veh.posG + veh.posG
        d = dot(car_vec, cw_to_car)
        Δs = d + delta - model.crosswalk.width/2
    end
    return Δs
end

function get_distance_to_crosswalk(model::CrosswalkDriver, veh::Vehicle, roadway::Roadway, delta::Float64 = 0.)
    return get_distance_to_crosswalk(model, veh.state, roadway, delta)
end

function update_stop!(model::CrosswalkDriver, veh::Vehicle, roadway::Roadway, dist_to_cw::Float64)
    update_stop!(model, veh.state, roadway, dist_to_cw)
end

function update_stop!(model::CrosswalkDriver, veh::VehicleState, roadway::Roadway, dist_to_cw::Float64)
    if veh.v ≈ 0. && isapprox(dist_to_cw, 0, atol=1.)
        model.stop = true
    end
end

# # croswalk driver
# @with_kw mutable struct CrosswalkDriver <: DriverModel{LonAccelDirection}
#     a::LonAccelDirection = LonAccelDirection(0., 1)
#     navigator::RouteFollowingIDM = RouteFollowingIDM()
#     crosswalk::Lane = Lane()
#     intersect::Lane = Lane()
#     ped_model::ConstantPedestrian = ConstantPedestrian()
#     ped_start::Float64 = 4.0 # assume a 5meter buffer before a pedestrian reaches the road
#     stop_delta::Float64 = 0.0
#     accel_tol::Float64 = 0.1
#     d_tol::Float64 = 0.5
#
#     #states
#     go::Bool = false
#     stop::Bool = false
#
#     # transition
#     clear::Bool = false
#
#     debug::Bool = false
# end
#
# AutomotiveDrivingModels.get_name(::CrosswalkDriver) = "CrosswalkDriver"
# Base.rand(model::CrosswalkDriver) = model.a
#
# function AutomotiveDrivingModels.reset_hidden_state!(model::CrosswalkDriver)
#     model.go = false
#     model.stop = false
#     model
# end
#
# function AutomotiveDrivingModels.observe!(model::CrosswalkDriver, scene::Scene, roadway::Roadway, egoid::Int)
#     ego = scene[findfirst(scene, egoid)]
#     AutomotiveDrivingModels.observe!(model.navigator, scene, roadway, egoid) # set the direction
#     dir = model.navigator.dir
#     a_lon =0.
#     a_lon_idm = model.navigator.a
#     if model.go
#         a_lon = a_lon_idm
#     else
#         update_clear!(model, ego, scene, roadway)
#         if model.clear
#             model.go = true
#         else # brake!
#             model.clear = false
#             d = dist_to_stop(model, ego, roadway)
#             if model.debug
#                 println("braking! d = ", d, "a = ", stop_at_dist(model, ego, d))
#             end
#             a_lon = min(a_lon_idm, stop_at_dist(model, ego, d))
#             if ego.state.v ≈ 0. && d < model.d_tol
#                 a_lon = 0.
#             end
#         end
#     end
#     if model.debug
#         println("veh ", egoid, " go ", model.go, " stop ", model.stop, " clear ", model.clear)
#     end
#     model.a = LonAccelDirection(a_lon, dir)
# end
#
#
# function update_clear!(model::CrosswalkDriver, ego::Vehicle, scene::Scene, roadway::Roadway)
#     # check how many pedestrians are present
#     peds = Vehicle[]
#     for veh in scene
#         if veh.def.class == AgentClass.PEDESTRIAN && get_lane(roadway, veh) == model.crosswalk
#             push!(peds, veh)
#         end
#     end
#     cw_length = get_end(model.crosswalk)
#     cw_center = get_posG(Frenet(model.crosswalk, cw_length/2), roadway)
#     collision_point = VecSE2(cw_center.x, ego.state.posG.y)
#     # set to the maximum value
#     buffer_ped = VehicleState(Frenet(model.crosswalk, model.ped_start), roadway,
#                               model.ped_model.v_desired)
#     # println("compute time ped")
#     min_ped_ttc = compute_time_of_approach(model, collision_point, buffer_ped, roadway, -model.intersect.width/2)
#     if model.debug
#         println("buffer ped ttc ", min_ped_ttc)
#     end
#     ego_ttc = compute_time_of_approach(model, collision_point, ego.state, roadway,
#                                        +model.crosswalk.width/2 + ego.def.length/2)
#     if ego.state.v ≈ 0.
#         ego_ttc = sqrt((model.crosswalk.width + ego.def.length)/model.navigator.a_max/2) # it is stopped
#     end
#     if model.debug
#         println("ego ttc ", ego_ttc)
#     end
#     has_crossed = -0.5*model.intersect.width/model.ped_model.v_desired
#     if model.debug
#         println("has_crossed ttc", has_crossed)
#     end
#     for ped in peds
#         ttc = compute_time_of_approach(model, collision_point, ped.state, roadway, -model.intersect.width/2)
#         if model.debug
#             println("ped $(ped.id) ttc ", ttc)
#         end
#         if has_crossed < ttc < min_ped_ttc
#             min_ped_ttc = ttc
#         end
#     end
#     if model.debug
#         println("min ped ttc ", min_ped_ttc)
#     end
#     if has_crossed < min_ped_ttc < ego_ttc
#         model.clear = false
#     else
#         model.clear = true
#     end
# end
#
# """
# compute the time to reach the projection of cw on the ego lane
# """
# function compute_time_of_approach(model::CrosswalkDriver, cw::VecSE2, ego::VehicleState, roadway::Roadway, delta::Float64=0.)
#     lane = get_lane(roadway, ego)
#     cw_proj = Frenet(cw, lane, roadway)
#     Δs = cw_proj.s - ego.posF.s + delta
#     # println("lane ", lane)
#     # println("cw_proj ", cw_proj.s)
#     # println("delta ", delta)
#     # println("ego posF", ego.posF.s)
#     # println("ds ", Δs)
#     v = max(ego.v, model.ped_model.v_desired)
#     time_of_approach = Δs/v
# end
#
# function dist_to_stop(model::CrosswalkDriver, ego::Vehicle, roadway::Roadway)
#     cw_length = get_end(model.crosswalk)
#     cw_center = get_posG(Frenet(model.crosswalk, cw_length/2), roadway)
#     collision_point = VecSE2(cw_center.x, ego.state.posG.y)
#     intersect_pt = Frenet(collision_point, model.intersect, roadway)
#     return intersect_pt.s - model.crosswalk.width/2 - ego.state.posF.s - ego.def.length/2 - model.stop_delta
# end
#
# function is_crossing(ped::Vehicle, crosswalk::Lane, roadway::Roadway)
# end
