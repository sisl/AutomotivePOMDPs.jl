
@with_kw mutable struct FrenetPedestrianPOMDP <: DriverModel{LatLonAccel}
    a::LatLonAccel = LatLonAccel(0.0, 0)
    env::CrosswalkEnv = CrosswalkEnv(CrosswalkParams())
    sensor::AutomotiveSensors.GaussianSensor = AutomotiveSensors.GaussianSensor(AutomotiveSensors.LinearNoise(10, 0., 0.), 
                                                               AutomotiveSensors.LinearNoise(10, 0., 0.), 0, 0, MersenneTwister(1)) 
    timestep::Float64 = 0
    t_current::Float64 = 0
    tick::Int64 = 0
   
    obstacles::Vector{ConvexPolygon}

    risk::Float64 = 0.0
    sensor_observations::Vector{Vehicle} = []


    update_tick_high_level_planner::Int64 = 10

    pomdp::SingleOCFPOMDP = SingleOCFPOMDP()
    policy::AlphaVectorPolicy{SingleOCFPOMDP,SingleOCFAction} = AlphaVectorPolicy(pomdp, Vector{Vector{Float64}}())
    updater::SingleOCFUpdater = SingleOCFUpdater(pomdp)
    b::SingleOCFBelief = SingleOCFBelief(Vector{SingleOCFState}(), Vector{Float64}())

end


function AutomotiveDrivingModels.propagate(veh::Vehicle, action::LatLonAccel, roadway::Roadway, Δt::Float64)

    # new velocity
    v_ = veh.state.v + action.a_lon*Δt
    if v_ < 0.
        v_ = 0.
    end
    
    # lateral offset
    delta_y = action.a_lat*Δt
    if v_ < 0.
        delta_y = 0.
    end
    s_new = v_ * Δt

    # longitudional distance based on required velocity and lateral offset
    delta_x = sqrt(s_new^2 - delta_y^2 )

    y_ = veh.state.posG.y + delta_y
    x_ = veh.state.posG.x + delta_x

    return VehicleState(VecSE2(x_, y_, veh.state.posG.θ), roadway, v_)
end

function AutomotiveDrivingModels.get_name(model::FrenetPedestrianPOMDP)
    return "Frenet Pedestrian POMDP"
end

AutomotiveDrivingModels.rand(model::FrenetPedestrianPOMDP) = model.a




function animate_record(rec::SceneRecord,dt::Float64, env::CrosswalkEnv, sensor::GaussianSensor, sensor_o::Vector{Vector{Vehicle}}, risk::Vector{Float64}, cam=FitToContentCamera(0.0))
    duration =rec.nframes*dt::Float64
    fps = Int(1/dt)
    function render_rec(t, dt)
        frame_index = Int(floor(t/dt)) + 1
        text_to_visualize = [string(risk[frame_index])]
        sensor_overlay = GaussianSensorOverlay(sensor=sensor, o=sensor_o[frame_index])
        occlusion_overlay = OcclusionOverlay(obstacles=env.obstacles)
        text_overlay = TextOverlay(text=text_to_visualize,pos=VecE2(10.,15.),incameraframe=true,color=colorant"white",font_size=30)
        return render(rec[frame_index-nframes(rec)], env, [sensor_overlay, occlusion_overlay, text_overlay, IDOverlay()], cam=cam)

    end
    return duration, fps, render_rec
end

@with_kw mutable struct ObservationCallback
    risk::Vector{Float64}
    sensor_observations::Vector{Vector{Vehicle}}
end

function AutomotiveDrivingModels.run_callback{S,D,I,R,M<:DriverModel}(
        callback::ObservationCallback,
        rec::EntityQueueRecord{S,D,I},
        roadway::R,
        models::Dict{I,M},
        tick::Int)
    
    push!(callback.risk, models[1].risk)
    push!(callback.sensor_observations, models[1].sensor_observations)
    
    return false
end
