#### DEFINE A PEDESTRIAN MODEL AND A PEDESTRIAN FLOW #####

const PED_WIDTH = 1.0
const PED_LENGTH = 1.0
const PEDESTRIAN_DEF = VehicleDef(AgentClass.PEDESTRIAN, PED_LENGTH, PED_WIDTH)

"""
Action type for the pedestrian model
"""
struct ConstantSpeedDawdling
    v::Float64 # instantaneous change in velocity
    lat::Float64 # lateral displacement
end

"""
    Simple model, pedestrian walking at constant speed with noise
    and dawdling in the y direction
"""
@with_kw mutable struct ConstantPedestrian <: DriverModel{ConstantSpeedDawdling}
    update_freq::Int64 = 1
    tick::Int64 = 0.
    a::ConstantSpeedDawdling = ConstantSpeedDawdling(0., 0.)
    dawdling_amp::Float64 = 0.
    v_noise::Float64 = 0.5
    v_desired::Float64 = 1.0
    dt::Float64 = 0.1
end

AutomotiveDrivingModels.get_name(model::ConstantPedestrian) = "ConstantPedestrian"
Base.rand(rng::AbstractRNG, model::ConstantPedestrian) = model.a
Distributions.pdf(model::ConstantPedestrian, a::ConstantSpeedDawdling) = 1/3*pdf(Normal(0., model.dawdling_amp))
AutomotiveDrivingModels.reset_hidden_state!(model::ConstantPedestrian) = model

function AutomotiveDrivingModels.observe!(model::ConstantPedestrian,
                                          scene::EntityFrame{VehicleState, VehicleDef, Int64},
                                          roadway::Roadway,
                                          egoid::Int)
    if model.tick % model.update_freq == 0
        model.a = ConstantSpeedDawdling(model.v_desired + rand([-model.v_noise, 0, model.v_noise]),
                                        model.dawdling_amp*randn())
    end
    model.a = ConstantSpeedDawdling(model.a.v, model.dawdling_amp*randn())
    model.tick += 1
end

function AutomotiveDrivingModels.propagate(veh::VehicleState, action::ConstantSpeedDawdling, roadway::Roadway, ΔT::Float64)
    lane_tag_orig = veh.posF.roadind.tag

    # update value in Frenet coordinates
    v = veh.v
    lat = action.lat
    ϕ = veh.posF.ϕ
    s = veh.posF.s
    Δs = v*cos(ϕ)*ΔT + lat*sin(ϕ)
    t = veh.posF.t
    Δt = v*sin(ϕ)*ΔT + lat*cos(ϕ)
    lane = get_lane(roadway, veh)
    # cannot go offroad
    if t + Δt > lane.width/2
        Δt = 0.
    elseif t - Δt < -lane.width/2
        Δt = 0.
    end

    speed = action.v
    # println(s + Δs)
    s_ = clamp(s + Δs, 0., get_end(lane))
    # println(s_)
    
    #XXX what exactly is happening here, ask Tim ?
    # roadind = move_along(veh.posF.roadind, roadway, Δs)
    # footpoint = roadway[roadind]    
    # posG = convert(VecE2, footpoint.pos) + polar(t + Δt, footpoint.pos.θ + π/2)
    
    # posG = VecSE2(posG.x, posG.y, footpoint.pos.θ + ϕ)
    
    # state = VehicleState(posG, roadway, action.v)
    
    # roadproj = proj(state.posG, roadway[lane_tag_orig], roadway, move_along_curves=false)
    # retval = VehicleState(Frenet(roadproj, roadway), roadway, state.v)
    # return retval
    return VehicleState(Frenet(lane, s_, t + Δt, ϕ), roadway, speed)
end

function AutomotiveDrivingModels.propagate(veh::Vehicle, action::ConstantSpeedDawdling, roadway::Roadway, ΔT::Float64)
    return propagate(veh.state, action, roadway, ΔT)
end

## Discrete distribution
# P(a | s)

function action_space(model::ConstantPedestrian)
    return (ConstantSpeedDawdling(0., 0.), ConstantSpeedDawdling(1., 0.), ConstantSpeedDawdling(2., 0.))
end

function get_distribution(model::ConstantPedestrian)
    actions = action_space(model)
    probs = (1/3, 1/3, 1/3)
    return actions, probs
end
