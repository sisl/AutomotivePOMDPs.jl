function AutoViz.render!(rendermodel::RenderModel, env::CrosswalkEnv)
    roadway = gen_straight_roadway(2, env.params.roadway_length)
    render!(rendermodel, roadway)

    curve = env.crosswalk.curve
    n = length(curve)
    pts = Array(Float64, 2, n)
    for (i,pt) in enumerate(curve)
        pts[1,i] = pt.pos.x
        pts[2,i] = pt.pos.y
    end

    add_instruction!(rendermodel, render_dashed_line, (pts, colorant"white", env.crosswalk.width, 1.0, 1.0, 0.0, Cairo.CAIRO_LINE_CAP_BUTT))
    obs = env.obstacles[1]
    for obs in env.obstacles
        pts = Array(Float64, 2, obs.npts)
        for (i, pt) in enumerate(obs.pts)
            pts[1,i] = pt.x
            pts[2,i] = pt.y
        end

        add_instruction!(rendermodel, render_fill_region, (pts, colorant"gray"))
    end

    return rendermodel
end

function animate_record(rec::SceneRecord, overlay::SceneOverlay)
    duration =rec.nframes*config.sim_dt
    fps = Int(1/config.sim_dt)
    function render_rec(t, dt)
        frame_index = Int(floor(t/dt)) + 1
        return render(rec[frame_index-nframes(rec)], env, [overlay], cam=cam)
    end
    return duration, fps, render_rec
end

type WaitAndGoOverlay <: SceneOverlay
    Δt::Float64
    speed::Float64
    # states
    policy::waitAndGo

    target_id::Int
    verbosity::Int
    color::Colorant
    font_size::Int

    function WaitAndGoOverlay( Δt::Float64, speed::Float64,  policy::waitAndGo,
        target_id::Int=1, verbosity::Int=1;
        color::Colorant=colorant"white",
        font_size::Int=20,
        )

        new(Δt, speed,policy, target_id, verbosity, color,font_size)
    end
end

function AutoViz.render!(rendermodel::RenderModel, overlay::WaitAndGoOverlay,
                        scene::Scene, env::CrosswalkEnv)

    # render overlay
    font_size = overlay.font_size
    text_y = font_size
    text_y_jump = round(Int, font_size*1.2)

    # display ego id
    add_instruction!( rendermodel, render_text, (@sprintf("id = %d", overlay.target_id), 10, text_y, font_size, overlay.color), incameraframe=false)
        text_y += text_y_jump
    # display ego speed
    ego = scene[findfirst(scene, overlay.target_id)]
    add_instruction!( rendermodel, render_text, (@sprintf("v = %10.3f m/s", ego.state.v), 10, text_y, font_size, overlay.color), incameraframe=false)
    text_y += text_y_jump
    # display acceleration
    acc = (ego.state.v - overlay.speed)/overlay.Δt
    overlay.speed = ego.state.v
    add_instruction!( rendermodel, render_text, (@sprintf("a = %10.3f m/s^2", acc), 10, text_y, font_size, overlay.color), incameraframe=false)
    text_y += text_y_jump
    # display ttc
    ttc_min = Inf
    for veh in scene
        if veh.id == overlay.target_id
            continue
        end
        ttc = (env.params.lane_width - ped.state.posG.y)/ped.state.v
        if ttc < ttc_min
            ttc_min = ttc
        end
    end
    add_instruction!( rendermodel, render_text, (@sprintf("TTC = %10.3f s", ttc_min), 10, text_y, font_size, overlay.color), incameraframe=false)
    text_y += text_y_jump
    if !overlay.policy.reaching
        if !overlay.policy.wait && (ego.state.v ≈ 0. && ego.state.posG.x > overlay.policy.initial_state.state.posG.x) # change state
            overlay.policy.reaching = false
            overlay.policy.wait = true
        end
    end

    if !overlay.policy.go
        # check
        if ttc_min > overlay.policy.threshold
            overlay.policy.wait = false
            overlay.policy.N += 1
        else
            overlay.policy.N = 0
        end
        if overlay.policy.N == overlay.policy.N0
            overlay.policy.go = true
        end
    end


    # display state
    if overlay.policy.reaching
        state = "reaching"
    elseif overlay.policy.wait
        state = "wait $(overlay.policy.N)"
    elseif overlay.policy.go
        state = "go"
    end
    add_instruction!( rendermodel, render_text, (state, 10, text_y, font_size, overlay.color), incameraframe=false)
    text_y += text_y_jump


    rendermodel
end
