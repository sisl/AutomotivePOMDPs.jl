@with_kw mutable struct TTCIntersectionDriver <: DriverModel{LonAccelDirection}
    a::LonAccelDirection = LonAccelDirection(0., 1)
    navigator::RouteFollowingIDM = RouteFollowingIDM()
    intersection::Vector{Lane} = Lane[]
    intersection_pos::VecSE2 = VecE2(0., 0., 0.)
    ttc_threshold::Float64 = 4.5
    horizon::Float64 = 20.0 # distance from the intersection to start the logic
    stop_delta::Float64 = 0. # precision for stopping slightly before the line
    accel_tol::Float64 = 1e-2 # if |a| < accel_tol then a = 0.
    priorities::Dict{Tuple{LaneTag, LaneTag}, Bool} = Dict{Tuple{LaneTag, LaneTag}, Bool}()
    # states
    priority::Bool = false
    stop::Bool = false # switch to true when stopped at the intersection, stays true until the end
end


get_name(::TTCIntersectionDriver) = "TTCIntersectionDriver"
Base.rand(model::TTCIntersectionDriver) = model.a

function AutomotiveDrivingModels.set_desired_speed!(model::TTCIntersectionDriver, v_des::Float64)
    model.navigator.v_des = v_des
    model
end

function AutomotiveDrivingModels.reset_hidden_state!(model::TTCIntersectionDriver)
    model.priority = false
    model.stop = false
    model
end

function AutomotiveDrivingModels.observe!(model::TTCIntersectionDriver, scene::Scene, roadway::Roadway, egoid::Int)
    ego = scene[findfirst(egoid, scene)]
    AutomotiveDrivingModels.observe!(model.navigator, scene, roadway, egoid) # set the direction
    dir = model.navigator.dir
    a_lon =0.
    a_lon_idm = model.navigator.a
    passed = has_passed(model, scene, roadway, egoid)
    is_engaged = engaged(model, scene, roadway, egoid)
    right_of_way = model.priorities[(model.navigator.route[1].tag,model.navigator.route[end].tag)]
    is_clogged = is_intersection_clogged(model, scene, roadway, egoid)
    ttc = ttc_check(model, scene, roadway, egoid)
    # if !model.stop
    #     model.stop = isapprox(ego.state.v, 0.)
    # end
    
    if isempty(model.intersection) || passed 
        a_lon = a_lon_idm 
    elseif !passed 
        if right_of_way
            if is_clogged && !passed && is_engaged && !isapprox(ego.state.v, 0.)
                # println("Vehicle $egoid : emergency break")
                a_lon = -model.navigator.d_max
            else
                a_lon = a_lon_idm
            end
        else # left turn
            if !ttc && !is_engaged  # before left turn
                a_lon = min(a_lon_idm, stop_at_end(model, ego, roadway))
            elseif is_clogged && !passed && is_engaged && !isapprox(ego.state.v, 0.) #!ttc && !passed && is_engaged || (is_clogged && is_engaged)
                # println("Vehicle $egoid : emergency break")
                a_lon = -model.navigator.d_max
            elseif ttc 
                a_lon = a_lon_idm 
            end
        end
    end
    # println("veh $egoid | clogged $is_clogged | passed $passed | engaged $is_engaged | stop $(isapprox(ego.state.v, 0.))| right of way $right_of_way")
    # println("veh $egoid: ttc $ttc") 
    # println(" ID ", egoid, " stop ", model.stop, " priority ", model.priority)
    model.a = LonAccelDirection(a_lon, dir) # add noise to break ties #XXX remove with priorities
    model
end

function engaged(model::TTCIntersectionDriver, scene::Scene, roadway::Roadway, egoid::Int)
    ego = scene[findfirst(egoid, scene)]
    lane = get_lane(roadway, ego)
    inter_width = 7.5 #todo parameterized
    if normsquared(VecE2(model.intersection_pos - ego.state.posG)) < inter_width^2
        return true 
    end
    return false 
    # if isempty(lane.entrances)
    #     return false
    # end
    # return true
end

function has_passed(model::TTCIntersectionDriver, scene::Scene, roadway::Roadway, egoid::Int)
    ego = scene[findfirst(egoid, scene)]
    lane = get_lane(roadway, ego)
    inter_to_car = ego.state.posG - model.intersection_pos
    car_vec = get_front(ego) - ego.state.posG
    has_passed = dot(inter_to_car, car_vec) > 0. || lane ∈ get_exit_lanes(roadway)
    return has_passed
end

"""
Check if the time to collision is above some threshold
"""
function ttc_check(model::TTCIntersectionDriver, scene::Scene, roadway::Roadway, egoid::Int)
    min_ttc = Inf
    inter_width = 6.0 #todo parameterized
    otherid = -1
    ego = scene[findfirst(egoid, scene)]
    for veh in scene
        if veh.id != egoid && veh.def.class != AgentClass.PEDESTRIAN && !is_behind(ego, veh, roadway)
            posF = veh.state.posF
            int_x, int_y, int_θ = model.intersection_pos
            lane = get_lane(roadway, veh)
            int_proj = Frenet(proj(model.intersection_pos, lane, roadway, move_along_curves=false), roadway) 
            if normsquared(VecE2(model.intersection_pos - veh.state.posG)) < inter_width^2 # vehicle is in the middle
                ttc = 0.
            else
                ttc = (int_proj.s - posF.s)/veh.state.v
            end
            if 0 <= ttc < min_ttc
                otherid = veh.id
                min_ttc = ttc
            end
        end
    end
    # println("veh id ", egoid, "min_ttc ", min_ttc, " threshold ", model.ttc_threshold, " other id ", otherid)
    if 0 <= min_ttc < model.ttc_threshold
        return false
    else
        return true
    end
end

function is_intersection_clogged(model::TTCIntersectionDriver, scene::Scene, roadway::Roadway, egoid::Int64)
    inter_width = 5.0 #todo parameterized
    ego = scene[findfirst(egoid, scene)]
    for veh in scene 
        if veh.id == egoid
            continue
        end
        # println("veh id $(veh.id) : ", sqrt(normsquared(VecE2(model.intersection_pos - veh.state.posG))))
        if normsquared(VecE2(model.intersection_pos - veh.state.posG)) < inter_width^2 && !is_behind(ego, veh, roadway)
            return true 
        end
    end
    return false
end

function is_behind(veh1::Vehicle, veh2::Vehicle, roadway::Roadway)
    lane1 = get_lane(roadway, veh1)
    # project veh2 on lane2 
    veh2_l1 = Frenet(proj(veh2.state.posG, lane1, roadway, move_along_curves=false), roadway)
    return veh2_l1.s < veh1.state.posF.s 
end