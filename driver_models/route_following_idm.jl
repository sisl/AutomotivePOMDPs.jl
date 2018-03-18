"""
Follow a given route, longitudinal acceleration is controlled by the IntelligentDriverModel
"""
@with_kw mutable struct RouteFollowingIDM <: LaneFollowingDriver
    route::Vector{Lane} = Lane[]
    dir::Int64 = 1
    a::Float64 = NaN # predicted acceleration
    σ::Float64 = NaN # optional stdev on top of the model, set to zero or NaN for deterministic behavior

    k_spd::Float64 = 1.0 # proportional constant for speed tracking when in freeflow [s⁻¹]

    δ::Float64 = 4.0 # acceleration exponent [-]
    T::Float64  = 1.5 # desired time headway [s]
    v_des::Float64 = 8.0 # desired speed [m/s]
    s_min::Float64 = 5.0 # minimum acceptable gap [m]
    a_max::Float64 = 2.0 # maximum acceleration ability [m/s²]
    d_cmf::Float64 = 2.0 # comfortable deceleration [m/s²] (positive)
    d_max::Float64 = 9.0 # maximum decelleration [m/s²] (positive)
end

AutomotiveDrivingModels.get_name(::RouteFollowingIDM) = "RouteFollowingIDM"

function AutomotiveDrivingModels.set_desired_speed!(model::RouteFollowingIDM, v_des::Float64)
    model.v_des = v_des
    model
end



"""
Compute acceleration according to IDM
"""
function AutomotiveDrivingModels.track_longitudinal!(model::RouteFollowingIDM, v_ego::Float64, v_oth::Float64, headway::Float64)

    if !isnan(v_oth) && headway > 0.
        # @assert !isnan(headway) && headway > 0

        Δv = v_oth - v_ego
        s_des = model.s_min + v_ego*model.T - v_ego*Δv / (2*sqrt(model.a_max*model.d_cmf))
        v_ratio = model.v_des > 0.0 ? (v_ego/model.v_des) : 1.0
        model.a = model.a_max * (1.0 - v_ratio^model.δ - (s_des/headway)^2)
    elseif headway < 0.
        model.a = -model.d_max
    else
        # no lead vehicle, just drive to match desired speed
        Δv = model.v_des - v_ego
        model.a = Δv*model.k_spd # predicted accel to match target speed
    end

    @assert !isnan(model.a)

    model.a = clamp(model.a, -model.d_max, model.a_max)

    model
end


"""
set the lane exit to take at the next junction to reach the goal
"""
function set_direction!(model::RouteFollowingIDM, scene::Scene, roadway::Roadway, egoid::Int64)
    ego = scene[findfirst(scene, egoid)]
    cur_lane = get_lane(roadway, ego)
    ind = findfirst(model.route, cur_lane)
    if ind == length(model.route)
        return
    end
    next_exit = model.route[ind+1]
    for i=1:length(cur_lane.exits)
        if cur_lane.exits[i].target.tag == next_exit.tag
            model.dir=i
            break
        end
    end
end

function AutomotiveDrivingModels.observe!(model::RouteFollowingIDM, scene::Scene, roadway::Roadway, egoid::Int)

    vehicle_index = findfirst(scene, egoid)

    fore = get_neighbor_fore_along_lane(scene, vehicle_index, roadway,
                                        VehicleTargetPointFront(), VehicleTargetPointRear(), VehicleTargetPointFront())
    v_ego = scene[vehicle_index].state.v

    if fore.ind != 0
        headway, v_oth = fore.Δs, scene[fore.ind].state.v
    else
        headway, v_oth = NaN, NaN
    end
    track_longitudinal!(model, v_ego, v_oth, headway)
    set_direction!(model, scene, roadway, egoid)

    return model
end

function Base.rand(model::RouteFollowingIDM)
    if isnan(model.σ) || model.σ ≤ 0.0
        LonAccelDirection(model.a, model.dir)
    else
        LonAccelDirection(rand(Normal(model.a, model.σ)), model.dir)
    end
end
