function animate_hist(pomdp::UrbanPOMDP, hist, overlays::Vector{SceneOverlay} = SceneOverlay[IDOverlay()], cam = FitToContentCamera(0.))
    duration = n_steps(hist)*pomdp.ΔT
    fps = Int(1/pomdp.ΔT)
    function render_hist(t, dt)
        stateindex = Int(floor(t/dt)) + 1
        scene = state_hist(hist)[stateindex]
        return AutoViz.render(scene, pomdp.env, overlays, cam = cam,  car_colors=get_colors(scene))
    end
    return duration, fps, render_hist
end

function animate_hist(pomdp::UrbanPOMDP, state_hist, lidar_hist, overlays::Vector{SceneOverlay} = SceneOverlay[IDOverlay()], cam = FitToContentCamera(0.))
    duration = length(state_hist)*pomdp.ΔT
    fps = Int(1/pomdp.ΔT)
    function render_hist(t, dt)
        stateindex = Int(floor(t/dt)) + 1
        scene = state_hist[stateindex]
        overlays_t = [overlays; LidarOverlay(lidar_hist[stateindex])]
        return AutoViz.render(scene, pomdp.env, overlays_t, cam = cam,  car_colors=get_colors(scene))
    end
    return duration, fps, render_hist
end

function animate_scenes(scenes::Vector{Scene}, models, env::UrbanEnv; overlays::Vector{SceneOverlay} = SceneOverlay[], cam=FitToContentCamera(0.),  sim_dt=0.1)
    duration =length(scenes)*sim_dt
    fps = Int(1/sim_dt)
    # cam = FitToContentCamera(0.)
    function render_rec(t, dt)
        frame_index = Int(floor(t/dt)) + 1
        disp = SceneOverlay[overlays...]
        push!(disp, TextOverlay(text = ["Step: $frame_index"],
                                                           font_size=20,
                                                           pos=VecE2(-15,-10),
                                                           incameraframe=true))
        for veh in scenes[frame_index]
            if veh.id == EGO_ID || veh.def.class == AgentClass.PEDESTRIAN
                continue
            end
            route = models[frame_index][veh.id].navigator.route
            blk_on, blk_right = turn_map(env, (route[1].tag, route[end].tag))
            push!(disp, BlinkerOverlay(on=blk_on, right=blk_right, veh=veh))
        end
        return AutoViz.render(scenes[frame_index], env, disp, cam=cam)
    end
    return duration, fps, render_rec
end
