"""
convert to AutomotiveDrivingModels.Scene
"""
function state2scene(mdp::PedCarMDP, s::PedCarMDPState, ped_type::VehicleDef = mdp.ped_type)
    scene = Scene()
    push!(scene, Vehicle(s.ego, mdp.ego_type, EGO_ID))
    push!(scene, Vehicle(s.car, mdp.ego_type, CAR_ID))
    push!(scene, Vehicle(s.ped, mdp.ped_type, PED_ID))
    return scene
end

function animate_states(mdp::PedCarMDP, states::Vector{PedCarMDPState}, actions::Vector{PedCarMDPAction};
                        overlays=SceneOverlay[IDOverlay()],
                        cam=StaticCamera(VecE2(0, -5.), 17.))
    duration = length(states)*mdp.ΔT
    fps = Int(1/mdp.ΔT)    
    function render_states(t, dt)
        frame_index = Int(floor(t/dt)) + 1
        scene = state2scene(mdp, states[frame_index])
        return AutoViz.render(scene,
                mdp.env,
                cat(1, overlays, TextOverlay(text = ["Acc: $(actions[frame_index].acc)"],
                                            font_size=20,
                                            pos=VecE2(0.,8.),
                                            incameraframe=true)),
                cam=cam,
                car_colors=get_colors(scene))
    end
    return duration, fps, render_states
end