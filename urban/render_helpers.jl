function animate_hist(pomdp::UrbanPOMDP, hist, overlays = SceneOverlay[IDOverlay()], cam = FitToContentCamera(0.))
    duration = n_steps(hist)*pomdp.ΔT
    fps = Int(1/pomdp.ΔT)
    function render_hist(t, dt)
        state_index = Int(floor(t/dt)) + 1
        scene = state_hist(hist)[state_index]
        return AutoViz.render(scene, pomdp.env, overlays, cam = cam,  car_colors=get_colors(scene))
    end
    return duration, fps, render_hist
end

function animate_hist(pomdp::UrbanPOMDP, state_hist, lidar_hist, overlays = SceneOverlay[IDOverlay()], cam = FitToContentCamera(0.))
    duration = length(state_hist)*pomdp.ΔT
    fps = Int(1/pomdp.ΔT)
    function render_hist(t, dt)
        state_index = Int(floor(t/dt)) + 1
        scene = state_hist[state_index]
        overlays_t = [overlays; LidarOverlay(lidar_hist[state_index])]
        return AutoViz.render(scene, pomdp.env, overlays_t, cam = cam,  car_colors=get_colors(scene))
    end
    return duration, fps, render_hist
end
