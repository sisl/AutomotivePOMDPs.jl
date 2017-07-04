#### DEFINE A PEDESTRIAN MODEL AND A PEDESTRIAN FLOW #####

const PED_WIDTH = 1.0
const PED_LENGTH = 1.0
const PEDESTRIAN_DEF = VehicleDef(AgentClass.PEDESTRIAN, PED_LENGTH, PED_WIDTH)

"""
Action type for the pedestrian model
"""
immutable ConstantSpeedDawdling
    v::Float64 # instantaneous change in velocity
    lat::Float64 # lateral displacement
end

"""
    Simple model, pedestrian walking at constant speed with noise
    and dawdling in the y direction
"""
type ConstantPedestrian <: DriverModel{ConstantSpeedDawdling}
    dawdling_amp::Float64
    v_noise::Float64
    v_desired::Float64
    dt::Float64
end

AutomotiveDrivingModels.get_name(model::ConstantPedestrian) = "ConstantPedestrian"
Base.rand(model::ConstantPedestrian) = ConstantSpeedDawdling(model.v_desired + model.v_noise*randn(),
                                                             model.dawdling_amp*randn())
Distributions.pdf(model::ConstantPedestrian, a::ConstantSpeedDawdling) = pdf(Normal(0., v_noise))*pdf(Normal(0., model.dawdling_amp))

function AutomotiveDrivingModels.propagate(veh::Vehicle, action::ConstantSpeedDawdling, roadway::Roadway, ΔT::Float64)
    lane_tag_orig = veh.state.posF.roadind.tag

    # update value in Frenet coordinates
    v = veh.state.v
    lat = action.lat
    ϕ = veh.state.posF.ϕ
    s = veh.state.posF.s
    Δs = v*cos(ϕ)*ΔT + lat*sin(ϕ)
    t = veh.state.posF.t
    Δt = v*sin(ϕ)*ΔT + lat*cos(ϕ)
    # cannot go offroad
    if t + Δt > roadway.segments[1].lanes[1].width
        Δt = 0.
    elseif t - Δt < -roadway.segments[1].lanes[1].width
        Δt = 0.
    end

    speed = action.v

    #XXX what exactly is happening here, ask Tim ?
    roadind = move_along(veh.state.posF.roadind, roadway, Δs)
    footpoint = roadway[roadind]

    posG = convert(VecE2, footpoint.pos) + polar(t + Δt, footpoint.pos.θ + π/2)

    posG = VecSE2(posG.x, posG.y, footpoint.pos.θ + ϕ)

    state = VehicleState(posG, roadway, action.v)

    roadproj = proj(state.posG, roadway[lane_tag_orig], roadway, move_along_curves=false)
    retval = VehicleState(Frenet(roadproj, roadway), roadway, state.v)
    return retval
end
