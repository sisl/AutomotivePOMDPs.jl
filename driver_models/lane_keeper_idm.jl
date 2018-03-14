mutable struct KeepLaneAcc
    a::Float64
    lane::Lane
end

function AutomotiveDrivingModels.propagate(veh::Vehicle, action::KeepLaneAcc, roadway::Roadway, Δt::Float64)
    s_ = veh.state.posF.s + veh.state.v*cos(veh.state.posF.ϕ)*Δt + 0.5*action.a*Δt^2
    s_ = clamp(s_, veh.state.posF.s, get_end(action.lane) )
    # println("From ground truth $(veh.state.posF.s) -> $s_ at $(veh.state.v)")
    v_ = veh.state.v + action.a*Δt # no backup
    v_ = clamp(v_, 0., action.lane.speed_limit.hi)

#     println(action.a, s_, action.lane)
    posF = Frenet(action.lane, s_)
    return VehicleState(posF, roadway, v_)
end

"""
Commonly referred to as IDM
"""
@with_kw mutable struct IntelligentDriverModel2D <: LaneFollowingDriver
    lane::Lane = env.roadway.segments[6].lanes[1]
    a::Float64 = NaN # predicted acceleration
    σ::Float64 = NaN # optional stdev on top of the model, set to zero or NaN for deterministic behavior

    k_spd::Float64 = 1.0 # proportional constant for speed tracking when in freeflow [s⁻¹]

    δ::Float64 = 4.0 # acceleration exponent [-]
    T::Float64  = 1.5 # desired time headway [s]
    v_des::Float64 = 9.0 # desired speed [m/s]
    s_min::Float64 = 5.0 # minimum acceptable gap [m]
    a_max::Float64 = 2.0 # maximum acceleration ability [m/s²]
    d_cmf::Float64 = 2.0 # comfortable deceleration [m/s²] (positive)
    d_max::Float64 = 9.0 # maximum decelleration [m/s²] (positive)
end

AutomotiveDrivingModels.get_name(::IntelligentDriverModel) = "IDM"
function AutomotiveDrivingModels.set_desired_speed!(model::IntelligentDriverModel2D, v_des::Float64)
    model.v_des = v_des
    model
end
function AutomotiveDrivingModels.track_longitudinal!(model::IntelligentDriverModel2D, v_ego::Float64, v_oth::Float64, headway::Float64)

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
function Base.rand(model::IntelligentDriverModel2D)
    if isnan(model.σ) || model.σ ≤ 0.0
        KeepLaneAcc(model.a, model.lane)
    else
        KeepLaneAcc(rand(Normal(model.a, model.σ)), model.lane)
    end
end
function Distributions.pdf(model::IntelligentDriverModel2D, a::KeepLaneAcc)
    if isnan(model.σ) || model.σ ≤ 0.0
        Inf
    else
        pdf(Normal(model.a, model.σ), a.a)
    end
end
function Distributions.logpdf(model::IntelligentDriverModel, a::KeepLaneAcc)
    if isnan(model.σ) || model.σ ≤ 0.0
        Inf
    else
        logpdf(Normal(model.a, model.σ), a.a)
    end
end

function AutomotiveDrivingModels.observe!(model::IntelligentDriverModel2D, scene::Scene, roadway::Roadway, egoid::Int)

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
#     println(model.a)
    return model
end
