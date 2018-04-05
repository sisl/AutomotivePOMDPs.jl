function animate_hist(pomdp::UrbanPOMDP, hist, overlays = SceneOverlay[], cam = FitToContentCamera(0.))
    duration = n_steps(hist)*pomdp.ΔT
    fps = Int(1/pomdp.ΔT)
    function render_hist(t, dt)
        state_index = Int(floor(t/dt)) + 1
        scene = state_hist(hist)[state_index]
        return AutoViz.render(scene, pomdp.env, overlays, cam = cam,  car_colors=get_colors(scene))
    end
    return duration, fps, render_hist
end
