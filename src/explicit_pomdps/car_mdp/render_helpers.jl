"""
convert to AutomotiveDrivingModels.Scene
"""
function state2scene(mdp::CarMDP, s::CarMDPState, car_type::VehicleDef = mdp.car_type)
    scene = Scene()
    push!(scene, Vehicle(s.ego, mdp.ego_type, EGO_ID))
    push!(scene, Vehicle(s.car, mdp.car_type, EGO_ID+1))
    return scene
end

function animate_states(mdp::CarMDP, states::Vector{CarMDPState}, actions::Vector{CarMDPAction};
                        overlays=SceneOverlay[IDOverlay()],
                        cam=StaticCamera(VecE2(0, -5.), 17.))
    duration = length(states)*mdp.ΔT
    fps = Int(1/mdp.ΔT)    
    function render_states(t, dt)
        frame_index = Int(floor(t/dt)) + 1
        scene = state2scene(mdp, states[frame_index])
        return AutoViz.render(scene,
                mdp.env,
              cat(1, overlays,   TextOverlay(text = ["v: $(get_ego(scene).state.v)"],
                                            font_size=20,
                                            pos=VecE2(mdp.env.params.x_min + 3.,6.),
                                            incameraframe=true),
                                 TextOverlay(text = ["Acc: $(actions[frame_index].acc)"],
                                            font_size=20,
                                            pos=VecE2(mdp.env.params.x_min + 3.,8.),
                                            incameraframe=true),
                                TextOverlay(text = ["step: $frame_index"],
                                            font_size=20,
                                            pos=VecE2(mdp.env.params.x_min + 3.,4.),
                                            incameraframe=true)),
                cam=cam,
                car_colors=get_colors(scene))
    end
    return duration, fps, render_states
end


