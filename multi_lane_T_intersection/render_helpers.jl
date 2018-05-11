# additional method to render the intersection environment with obstacles
function AutoViz.render!(rendermodel::RenderModel, env::IntersectionEnv)
    render!(rendermodel, env.roadway)

    for obs in env.obstacles
        pts = Array{Float64}(2, obs.npts)
        for (i, pt) in enumerate(obs.pts)
            pts[1,i] = pt.x
            pts[2,i] = pt.y
        end

        add_instruction!(rendermodel, render_fill_region, (pts, colorant"gray"))
    end

    stop_line = get_posG(Frenet(env.roadway[LaneTag(6,1)], env.params.stop_line), env.roadway)
    x_pos, y_pos = stop_line.x, stop_line.y
    stop_pts = zeros(2,2)
    stop_pts[1,:] =  [(x_pos - env.params.lane_width/2) , (x_pos + env.params.lane_width/2)]
    stop_pts[2,:] =  [y_pos, y_pos]
    add_instruction!(rendermodel, render_line, (stop_pts, colorant"white", 1.0, Cairo.CAIRO_LINE_CAP_BUTT))
    return rendermodel
end

function animate_record(rec::SceneRecord, env, overlays::Vector{SceneOverlay} = SceneOverlay[]; cam=FitToContentCamera(0.0), sim_dt::Float64=0.1)
    duration =rec.nframes*sim_dt
    fps = Int(1/sim_dt)
    function render_rec(t, dt)
        frame_index = Int(floor(t/dt)) + 1
        return AutoViz.render(rec[frame_index-nframes(rec)], overlays, env, cam=cam, car_colors=get_colors(scene))
    end
    return duration, fps, render_rec
end

function animate_hist(pomdp::OIPOMDP, hist; overlays = SceneOverlay[], cam = FitToContentCamera(0.))
    duration = n_steps(hist)*pomdp.ΔT
    fps = Int(1/pomdp.ΔT)
    function render_hist(t, dt)
        state_index = Int(floor(t/dt)) + 1
        scene = state_hist(hist)[state_index]
        return AutoViz.render(scene, pomdp.env, overlays, cam = cam,  car_colors=get_colors(scene))
    end
    return duration, fps, render_hist
end
