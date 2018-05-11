
function get_colors(scene::Scene, ego_id::Int64 = 1)
    colors = Dict{Int64, Colorant}()
    for veh in scene
        colors[veh.id] = veh.id == ego_id ?  COLOR_CAR_EGO : COLOR_CAR_OTHER
    end
    return colors
end

function animate_record(env, rec::SceneRecord; sim_dt::Float64=0.1, cam=FitToContentCamera(0.), overlays = SceneOverlay[])
    duration =rec.nframes*sim_dt
    fps = Int(1/sim_dt)
    function render_rec(t, dt)
        frame_index = Int(floor(t/dt)) + 1
        return AutoViz.render(rec[frame_index-nframes(rec)], env, overlays, cam=cam)
    end
    return duration, fps, render_rec
end

# function animate_scenes(scenes::Vector{Scene}, env; cam=FitToContentCamera(0.), sim_dt=0.1, overlays = SceneOverlay[])
#     duration =length(scenes)*sim_dt
#     fps = Int(1/sim_dt)
#     # cam = FitToContentCamera(0.)
#     function render_rec(t, dt)
#         frame_index = Int(floor(t/dt)) + 1
#         return AutoViz.render(scenes[frame_index], env, overlays, cam=cam)
#     end
#     return duration, fps, render_rec
# end

function animate_scenes(scenes::Vector{Scene}, env; overlays::Vector{SceneOverlay} = SceneOverlay[], cam=FitToContentCamera(0.),  sim_dt=0.1)
    duration =length(scenes)*sim_dt
    fps = Int(1/sim_dt)
    # cam = FitToContentCamera(0.)
    function render_rec(t, dt)
        frame_index = Int(floor(t/dt)) + 1

        return AutoViz.render(scenes[frame_index], env, overlays, cam=cam)
    end
    return duration, fps, render_rec
end

function animate_scenes(scenes::Vector{Scene},
                        actions::Vector{Float64},
                        env;
                        overlays::Vector{SceneOverlay} = SceneOverlay[],
                        cam=FitToContentCamera(0.),
                        sim_dt=0.1)
    duration =length(scenes)*sim_dt
    fps = Int(1/sim_dt)
    function render_rec(t, dt)
        frame_index = Int(floor(t/dt)) + 1
        return AutoViz.render(scenes[frame_index],
                              env,
                              cat(1, overlays, TextOverlay(text = ["Acc: $(actions[frame_index])"],
                                                           font_size=20,
                                                           pos=VecE2(-15,-10),
                                                           incameraframe=true)),
                              cam=cam,
                              car_colors=get_colors(scenes[frame_index]))
    end
    return duration, fps, render_rec
end

mutable struct IDOverlay <: SceneOverlay
    verbosity::Int
    color::Colorant
    font_size::Int

    function IDOverlay(verbosity::Int=1;
        color::Colorant=colorant"white",
        font_size::Int=20,
        )

        new(verbosity, color,font_size)
    end
end

function AutoViz.render!(rendermodel::RenderModel, overlay::IDOverlay, scene::Scene, env::OccludedEnv)
    font_size = overlay.font_size
    for veh in scene
        add_instruction!(rendermodel, render_text, ("$(veh.id)", veh.state.posG.x, veh.state.posG.y, font_size, colorant"black"), incameraframe=true)
    end
    return rendermodel
end
