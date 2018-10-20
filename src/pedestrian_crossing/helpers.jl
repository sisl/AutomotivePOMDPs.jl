
function get_state_absent(pomdp::SingleOCFPOMDP, ego_y::Float64, ego_v::Float64)

    (ego_y_state_space,ego_v_state_space) = getEgoDataInStateSpace(pomdp, ego_y, ego_v)
    return SingleOCFState(ego_y_state_space, ego_v_state_space, -10.0, -10.0, 0.0 ,0.0)

end

function get_state_absent(pomdp::SingleOCFPOMDP, s::SingleOCFState)

    (ego_y_state_space,ego_v_state_space) = getEgoDataInStateSpace(pomdp, s.ego_y, s.ego_v)
    return SingleOCFState(ego_y_state_space, ego_v_state_space, -10.0, -10.0, 0.0 ,0.0)

end

function is_state_absent(pomdp::SingleOCFPOMDP, s::SingleOCFState)

    if (s.ped_s == -10.0 && s.ped_T == -10.0 )
        return true;
    else
        return false;
    end

end

function is_observation_absent(pomdp::SingleOCFPOMDP, o::SingleOCFObs)

    if (o.ped_s > pomdp.S_MAX || o.ped_s < pomdp.S_MIN || o.ped_T > pomdp.T_MAX ||  o.ped_T < pomdp.T_MIN )
        return true;
    else
        return false;
    end

end


function getEgoDataInStateSpace(pomdp::SingleOCFPOMDP, y::Float64, v::Float64)
    y_grid = RectangleGrid(pomdp.EGO_Y_RANGE)
    (id, weight) = interpolants(y_grid, [y])
    id_max = indmax(weight)
    ego_y = ind2x(y_grid, id[id_max])[1]
    
    v_grid = RectangleGrid(pomdp.EGO_V_RANGE)
    (id, weight) = interpolants(v_grid, [v])
    id_max = indmax(weight)
    ego_v = ind2x(v_grid, id[id_max])[1]
    return ego_y, ego_v
end





@with_kw struct BeliefOverlay <: SceneOverlay
    belief::SingleOCFBelief = SingleOCFBelief()
    ego_vehicle::Vehicle = Vehicle(VehicleState(VecSE2(0., 0., 0.), 0.), VehicleDef(), 1)
    color::Colorant = RGBA(0.0, 0.0, 1.0, 0.2)
end


function AutoViz.render!(rendermodel::RenderModel, overlay::BeliefOverlay, scene::Scene, roadway::R) where R

    beliefs = overlay.belief
    ego = overlay.ego_vehicle
    
    for (s, prob) in weighted_iterator(beliefs)
        ped = Vehicle(VehicleState(VecSE2(s.ped_s, s.ped_T, s.ped_theta), 0.), VehicleDef(AutomotivePOMDPs.PEDESTRIAN_DEF), 1)
        prob_color = RGBA(0.0, 0.0, 1.0, prob*10)
        add_instruction!(rendermodel, render_vehicle, (ego.state.posG.x+ped.state.posG.x, ego.state.posG.y+ped.state.posG.y, ego.state.posG.θ + ped.state.posG.θ, ped.def.length, ped.def.width, prob_color))
    end
    return rendermodel
end


function animate_record(rec::SceneRecord,dt::Float64, env::CrosswalkEnv, sensor::GaussianSensor, sensor_o::Vector{Vector{Vehicle}}, risk::Vector{Float64}, belief::Vector{SingleOCFBelief}, ego_vehicle::Vector{Vehicle}, action_pomdp::Vector{SingleOCFAction}, cam=FitToContentCamera(0.0))
    duration =rec.nframes*dt::Float64
    fps = Int(1/dt)
    function render_rec(t, dt)
       
        frame_index = Int(floor(t/dt)) + 1
        text_to_visualize = [string("v-ego: ", ego_vehicle[frame_index].state.v, " m/s" , 
                                " y: ", ego_vehicle[frame_index].state.posG.y, " m",
                                " ax: ", action_pomdp[frame_index].acc, " m/s²")]
        sensor_overlay = GaussianSensorOverlay(sensor=sensor, o=sensor_o[frame_index])
        occlusion_overlay = OcclusionOverlay(obstacles=env.obstacles)
        text_overlay = TextOverlay(text=text_to_visualize,pos=VecE2(20.,10.),incameraframe=true,color=colorant"white",font_size=20)
        belief_overlay = BeliefOverlay(belief=belief[frame_index], ego_vehicle=ego_vehicle[frame_index])
     #   max_speed = 14.0
     #   histogram_overlay = HistogramOverlay(pos = VecE2(15.0, 10.0), val=ego_vehicle[frame_index].state.v/max_speed, label="v speed")

        return render(rec[frame_index-nframes(rec)], env, [belief_overlay, sensor_overlay, occlusion_overlay, text_overlay, IDOverlay()], cam=cam)

    end
    return duration, fps, render_rec
end