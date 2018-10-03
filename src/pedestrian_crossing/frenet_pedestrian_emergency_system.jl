
@with_kw mutable struct EmergencySystem <: DriverModel{LatLonAccel}
    a::LatLonAccel = LatLonAccel(0.0, 0)
    env::CrosswalkEnv = CrosswalkEnv(CrosswalkParams())
    sensor::AutomotiveSensors.GaussianSensor = AutomotiveSensors.GaussianSensor(AutomotiveSensors.LinearNoise(10, 0., 0.), 
                                                               AutomotiveSensors.LinearNoise(10, 0., 0.), 0, 0, MersenneTwister(1)) 
    timestep::Float64 = 0
    t_current::Float64 = 0
    tick::Int64 = 0
   
    state::Int32 = 0


    a_request::Float64 = 0
    a_current::Float64 = 0
    t_brake_trigger::Float64 = 0

    
    TS_BREAK::Float64 = 0.2
    TD_BREAK::Float64 = 0.2
    SAFETY_DISTANCE_LON::Float64 = 1.0


    collision_rate::Float64 = 0.0
    risk::Float64 = 0.0
    ttc::Float64 = 0.0
    brake_request::Bool = false
    brake_release::Bool = false

    obstacles::Vector{ConvexPolygon}

    prediction::Array{Float64} = []
    sensor_observations::Vector{Vehicle} = []


    update_tick_emergency_system::Int64 = 1

end

  
function AutomotiveDrivingModels.observe!(model::EmergencySystem, scene::Scene, roadway::Roadway, egoid::Int)

    model.t_current = model.t_current + model.timestep 
    model.tick += 1
    
    ego = scene[findfirst(scene, egoid)]
    model.sensor_observations = measure(model.sensor, ego, scene, roadway, model.obstacles)
    
    ################ Emergency System #####################################################
    (ttb, stop_distance) = getStopDistanceVehicle(model, ego.state.v, 10.0, 1.0)
    print("TTB: ", ttb)
    
    model.risk = 0
    model.collision_rate = 0
    model.brake_request = false
    model.prediction = []

    emergency_system_brake_request = false
    a = 0.0; phi_dot = 0.0
    t0 = 0.0; x0 = 0.0; y0 = 0.0; phi0 = 0.0
    TS = 0.01; T = 3.0
   

    if ( model.tick % model.update_tick_emergency_system == 0 )

        ego_data = caclulateCTRAModel(t0, x0, y0, phi0, ego.state.v, phi_dot, a, TS, T)
        for object in model.sensor_observations
        #  println(object)
            object_posF = Frenet(object.state.posG, get_lane(env.roadway, ego.state), env.roadway)

            delta_s = object_posF.s - ego.state.posF.s - ego.def.length/2
            delta_d = object_posF.t - ego.state.posF.t
            delta_theta = object_posF.ϕ - ego.state.posF.ϕ
            #ttc = delta_s / ego.v
            ped_v = object.state.v
            if (delta_s > 2)

                # in general, trajectory should come from a prediction model!!
                object_data = caclulateCTRAPredictionSimple(t0, delta_s, 0.1, delta_d, 0.1, 
                                                                delta_theta, 0.1, ped_v, 0.1, TS, T)

                (idx, ti) = getConflictZone(ego_data, object_data)
                (ttc, model.collision_rate, prediction) = caluclateCollision(ego_data, object_data, idx)
                model.prediction = prediction

                ttc_m = mean(ttc)
                ttc_std = std(ttc)

                print(" collision_rate: ", model.collision_rate, " ttc_m: ", ttc_m, " ttc_std: ", ttc_std)
             #   println("ttc_min: ", ttc_m-ttc_std)
                model.ttc = ttc_m-ttc_std
                model.risk = min(ttb / model.ttc, 1.0)
                println("Risk: ", model.risk, " ttc_min: ", model.ttc)

                if ( model.risk > 0.99 && model.collision_rate >= 0.6 )
                    emergency_system_brake_request = true
             #       println("Brake request!!!")
                end
            end
        end


        if ( emergency_system_brake_request == true )
            model.brake_request = true
        else
            model.brake_request = false
        end
        
    end
    
    # brake system
    model = brakeSystemStateMachine(model, ego.state)
    #model.a_current = 0
    model.a = LatLonAccel(0., model.a_current)
  #  println("model_state: ", model.state, " ", model.a) 
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

function AutomotiveDrivingModels.get_name(model::EmergencySystem)
    return "Emergency System"
end

AutomotiveDrivingModels.rand(model::EmergencySystem) = model.a


@with_kw struct PretictionOverlay <: SceneOverlay
    prediction::Array{Float64} = Array{Float64}()
    color::Colorant = RGBA(1., 0.0, 0.0, 0.2)
end

function AutoViz.render!(rendermodel::RenderModel, overlay::PretictionOverlay, scene::Scene, roadway::R) where R

    predictions = overlay.prediction
    ego = scene[findfirst(scene, 1)]
    for i=1:size(predictions)[1]
      pos = predictions[i,:]
      ped = Vehicle(VehicleState(VecSE2(pos[1], pos[2], 1.57), 0.), VehicleDef(AutomotivePOMDPs.PEDESTRIAN_DEF), 1)
      add_instruction!(rendermodel, render_vehicle, (ego.state.posG.x+ped.state.posG.x+2, ego.state.posG.y+ped.state.posG.y, ped.state.posG.θ, ped.def.length, ped.def.width, overlay.color))
    end
    return rendermodel
end

function animate_record(rec::SceneRecord,dt::Float64, env::CrosswalkEnv, sensor::GaussianSensor, sensor_o::Vector{Vector{Vehicle}}, risk::Vector{Float64}, prediction::Vector{Array{Float64}}, cam=FitToContentCamera(0.0))
    duration =rec.nframes*dt::Float64
    fps = Int(1/dt)
    function render_rec(t, dt)
        frame_index = Int(floor(t/dt)) + 1
        text_to_visualize = [string(risk[frame_index])]
        sensor_overlay = GaussianSensorOverlay(sensor=sensor, o=sensor_o[frame_index])
        occlusion_overlay = OcclusionOverlay(obstacles=env.obstacles)
        prediction_overlay = PretictionOverlay(prediction=prediction[frame_index])
        text_overlay = TextOverlay(text=text_to_visualize,pos=VecE2(10.,15.),incameraframe=true,color=colorant"white",font_size=30)
        return render(rec[frame_index-nframes(rec)], env, [prediction_overlay, sensor_overlay, occlusion_overlay, text_overlay, IDOverlay()], cam=cam)

    end
    return duration, fps, render_rec
end

@with_kw mutable struct ObservationCallback
    risk::Vector{Float64}
    collision_rate::Vector{Float64}
    ttc::Vector{Float64}
    brake_request::Vector{Bool}
    prediction::Vector{Array{Float64}}
    sensor_observations::Vector{Vector{Vehicle}}
end

function AutomotiveDrivingModels.run_callback{S,D,I,R,M<:DriverModel}(
        callback::ObservationCallback,
        rec::EntityQueueRecord{S,D,I},
        roadway::R,
        models::Dict{I,M},
        tick::Int)
    
    push!(callback.risk, models[1].risk)
    push!(callback.collision_rate, models[1].collision_rate)
    push!(callback.ttc, models[1].ttc)
    push!(callback.brake_request, models[1].brake_request)
    push!(callback.prediction, models[1].prediction)
    push!(callback.sensor_observations, models[1].sensor_observations)
    
    return false
end

function brakeSystemStateMachine(model::EmergencySystem, ego::VehicleState)
    

    if model.brake_release == true
        model.state = 0
    end

    println("")
    if model.state == 0 
        println("standby")
        model.a_current = 0
        if ( model.brake_request == true )
            model.state = 1
        end
    end
    
    if model.state == 1
        println("brake_request")
        model.a_current = 0
        model.a_request = -10
        model.t_brake_trigger = model.t_current + model.TD_BREAK
        model.state = 2
    end
    
    if model.state == 2
        println("brake_delay")
        model.a_current = 0
        if (model.t_current >= model.t_brake_trigger)
          model.state = 3    
        end
    end
    
    if model.state == 3
        println("brake_ramp")
        model.a_current = model.a_current - 50 * model.timestep 
        if ( model.a_request >=  model.a_current )
            model.state = 4
        end
    end
    if model.state == 4
        println("brake_max")
        if (ego.v > 0)
            model.a_current = model.a_request
        else
            model.a_current = 0.0
            model.state = 0
        end
    end

    return model
end




function getStopDistanceVehicle(model::EmergencySystem, delta_v::Float64, ax_max::Float64, friction_coefficient::Float64)

    a = min(ax_max,friction_coefficient*10);
  
    v_s = delta_v - (a / 2) *  model.TS_BREAK;      # velocity after ramp up phase     
    s_s = v_s *  model.TS_BREAK;                    # distance during ramp up phase
    s_v = (v_s * v_s) / (2 * a);                    # distance to stop with full acceleration

    distance_to_stop = model.TD_BREAK * delta_v + s_s + s_v;
    distance_to_stop = distance_to_stop + model.SAFETY_DISTANCE_LON


    if ( distance_to_stop < 0.0) 
        distance_to_stop = 0
    end
    ttb = distance_to_stop / delta_v
  
    return ttb, distance_to_stop
  end



@with_kw mutable struct Trajectory
    a::Float32 = 0
    phi_dot::Float64 = 0
    X::Matrix{Float64} = Array{Float64}(0,9)
end


function AutomotivePOMDPs.collision_checker(veh_a::VehicleState, veh_b::VehicleState, veh_a_def::VehicleDef, veh_b_def::VehicleDef)
    center_a = veh_a.posG
    center_b = veh_b.posG
    # first fast check:
    @fastmath begin
        Δ = sqrt((veh_a.posG.x - veh_b.posG.x)^2 + (veh_a.posG.y - veh_b.posG.y)^2)
        r_a = sqrt(veh_a_def.length*veh_a_def.length/4 + veh_a_def.width*veh_a_def.width/4)
        r_b = sqrt(veh_b_def.length*veh_b_def.length/4 + veh_b_def.width*veh_b_def.width/4)
    end
    if Δ ≤ r_a + r_b
        # fast check is true, run parallel axis theorem
        Pa = AutomotivePOMDPs.polygon(center_a, veh_a_def)
        Pb = AutomotivePOMDPs.polygon(center_b, veh_b_def)
        return AutomotivePOMDPs.overlap(Pa, Pb)
    end
    return false
end

function caclulateCTRAModel(t0::Float64, x0::Float64, y0::Float64, phi0::Float64, v0::Float64, 
                                        phi_dot::Float64, a::Float64, TS::Float64, T::Float64)

    trajectory = Trajectory()
    trajectory.a = a;
    trajectory.phi_dot = phi_dot;

    time = 0:TS:(T-TS)
    X =  Matrix(length(time), 9)
    x = [t0 x0 y0 phi0 v0 0.0 0.0 0.0 0.0]
    X[1,:] = x

    for i=2:length(time)
        phi = x[4]
        v = x[5]
        x = x + TS *[1 v*cos(phi) v*sin(phi) trajectory.phi_dot trajectory.a 0.0 0.0 0.0 0.0]
        X[i,:] = x
    end

    trajectory.X = X
    return trajectory
end


function caclulateCTRAPredictionSimple(t0::Float64, x0::Float64, x_s::Float64, y0::Float64, y_s::Float64,
                            phi::Float64, phi_s::Float64, v0::Float64, v_s::Float64, TS::Float64, T::Float64)

    trajectory = Trajectory()
    trajectory.a = 0;
    trajectory.phi_dot = 0;

    time = 0:TS:(T-TS)
    X = Matrix(length(time), 9)
    x = [t0 x0 y0 phi v0 x_s y_s phi_s v_s]
    X[1,:] = x

    STD_X = 0.2
    STD_Y = 0.1
    STD_NN = 0.1
    for i=2:length(time)
        phi = x[4]
        v = x[5]
        x = x + TS *[1 v*cos(phi) v*sin(phi) trajectory.phi_dot trajectory.a STD_X STD_Y STD_NN STD_NN ]
        X[i,:] = x
    end

    trajectory.X = X
    return trajectory
end


function getConflictZone(object_1::Trajectory, object_2::Trajectory)
    time = object_1.X[:,1]
    min_di = 500.
    min_di_idx = 500
    for i = 1:length(time)
        di = sqrt((object_1.X[i,2] - object_2.X[i,2])^2 + (object_1.X[i,3] - object_2.X[i,3])^2 )
        if ( di <= min_di )  
            min_di = di
            min_di_idx = i
        end
    end
    return min_di_idx, time[min_di_idx]
end


function caluclateCollision(object_1::Trajectory, object_2::Trajectory, critical_index::Int)
   
    time = object_1.X[:,1]
    ego_state_t0 = object_1.X[1,:]
    
    ttc = Float64[]
    collisions = 0

    ellipses = [] #ConvexPolygon([VecE2(0., 0.), VecE2(0., 0.), VecE2(0., 0.), VecE2(0., 0.)], 4)

    if (critical_index < length(time) )

        ego = object_1.X[critical_index,:]
        ego_def =VehicleDef()
        ego_state = VehicleState(VecSE2(ego[2],ego[3],ego[4]),ego[5])
      #  println(ego_state)
        object = object_2.X[critical_index,:]
        object_state = VehicleState(VecSE2(object[2],object[3],object[4]),object[5])
      #  println(object_state)
        ellipses = getObjectConfidenceInterval(object[2], object[6], object[3], object[7], object[4])
        object_positions = size(ellipses)[1]
        for i = 1:object_positions
            object_state = VehicleState(VecSE2(ellipses[i,1],ellipses[i,2],object[4]),object[5])
           # println(ellipses[i,1]," ",ellipses[i,2])
            collision = AutomotivePOMDPs.collision_checker(ego_state, object_state, ego_def, AutomotivePOMDPs.PEDESTRIAN_DEF)
            if (collision)
               # println("--> collision")
                collisions = collisions + 1
                ttc_tmp = (ellipses[i,1]-ego_state_t0[2] + AutomotivePOMDPs.PEDESTRIAN_DEF.width/2) / ego_state_t0[5]
                push!(ttc, ttc_tmp)
            end
        end

        x_min = minimum(ellipses[:,1]) - AutomotivePOMDPs.PEDESTRIAN_DEF.width/2;
        x_max = maximum(ellipses[:,1]) + AutomotivePOMDPs.PEDESTRIAN_DEF.width/2;

        y_min = minimum(ellipses[:,2]) - AutomotivePOMDPs.PEDESTRIAN_DEF.width/2;
        y_max = maximum(ellipses[:,2]) + AutomotivePOMDPs.PEDESTRIAN_DEF.width/2;
        prediction_area = ConvexPolygon([VecE2(x_min, y_max), VecE2(x_max, y_max), VecE2(x_max, y_min), VecE2(x_min, y_min)], 4)

        collision_rate = collisions/object_positions
    else
        collision_rate = 0
        push!(ttc, 100.0)
    end
    return ttc, collision_rate, ellipses
end



function getObjectConfidenceInterval(x::Float64, x_sigma::Float64, y::Float64, y_sigma::Float64, theta::Float64)

    # get the 99% confidence interval error ellipse
    confidence = 0.99
    k = (-2*log(1-confidence))^0.5
    
    x_s = k*x_sigma;
    y_s = k*y_sigma;
    
    # caluclation of the ellipse
    theta_grid = linspace(0.0, 2*pi, 20)

    ellipse_x  = x_s * cos.(theta_grid);
    ellipse_y  = y_s * sin.(theta_grid)

    # rotation via theta
    R = [ cos(theta) sin(theta); -sin(theta) cos(theta) ];
    ellipse = [ellipse_x ellipse_y] * R;

    # transform to x/y-position
    ellipse = [(ellipse[:,1] .+ x) (ellipse[:,2] .+ y)]
    return ellipse
end
