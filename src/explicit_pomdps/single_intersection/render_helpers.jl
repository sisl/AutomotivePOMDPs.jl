# additional method to render the crosswalk environment with obstacles
function AutoViz.render!(rendermodel::RenderModel, env::SimpleInterEnv)
    render!(rendermodel, env.roadway)

    obs = env.obstacles[1]
    for obs in env.obstacles
        pts = Array{Float64}(2, obs.npts)
        for (i, pt) in enumerate(obs.pts)
            pts[1,i] = pt.x
            pts[2,i] = pt.y
        end

        add_instruction!(rendermodel, render_fill_region, (pts, colorant"gray"))
    end

    stop_line = get_posG(Frenet(env.lane_map["ego_left"], env.params.stop_line), env.roadway)
    x_pos, y_pos = stop_line.x, stop_line.y
    stop_pts = zeros(2,2)
    stop_pts[1,:] =  [(x_pos - env.params.lane_width/2) , (x_pos + env.params.lane_width/2)]
    stop_pts[2,:] =  [y_pos, y_pos]
    add_instruction!(rendermodel, render_line, (stop_pts, colorant"white", 1.0, Cairo.CAIRO_LINE_CAP_BUTT))
    return rendermodel
end

function animate_record(rec::SceneRecord; sim_dt::Float64=0.1)
    duration =rec.nframes*sim_dt
    fps = Int(1/sim_dt)
    function render_rec(t, dt)
        frame_index = Int(floor(t/dt)) + 1
        return AutoViz.render(rec[frame_index-nframes(rec)], env, cam=cam)
    end
    return duration, fps, render_rec
end

function animate_hist(pomdp::SingleOIPOMDP, hist::POMDPHistory{SingleOIState,SingleOIAction,SingleOIState, B}) where B
    duration = n_steps(hist)*pomdp.ΔT
    fps = Int(1/pomdp.ΔT)
    cam = FitToContentCamera(0.)
    function render_hist(t, dt)
        stateindex = Int(floor(t/dt)) + 1
        scene = state_to_scene(pomdp, state_hist(hist)[stateindex])
        return AutoViz.render(scene, pomdp.env, cam = cam)
    end
    return duration, fps, render_hist
end
