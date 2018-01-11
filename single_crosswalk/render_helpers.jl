# additional method to render the crosswalk environment with obstacles
function AutoViz.render!(rendermodel::RenderModel, env::CrosswalkEnv)
    render!(rendermodel, env.roadway)

    curve = env.crosswalk.curve
    n = length(curve)
    pts = Array{Float64}(2, n)
    for (i,pt) in enumerate(curve)
        pts[1,i] = pt.pos.x
        pts[2,i] = pt.pos.y
    end

    add_instruction!(rendermodel, render_dashed_line, (pts, colorant"white", env.crosswalk.width, 1.0, 1.0, 0.0, Cairo.CAIRO_LINE_CAP_BUTT))
    obs = env.obstacles[1]
    for obs in env.obstacles
        pts = Array{Float64}(2, obs.npts)
        for (i, pt) in enumerate(obs.pts)
            pts[1,i] = pt.x
            pts[2,i] = pt.y
        end

        add_instruction!(rendermodel, render_fill_region, (pts, colorant"gray"))
    end

    return rendermodel
end

function animate(t, dt)
    frame_index = Int(floor(t/dt)) + 1
    return render(rec[frame_index-nframes(rec)], env.roadway, [CarFollowingStatsOverlay(1)], cam=cam)
end

function animate_scenes(scenes::Vector{Scene})
    duration =length(scenes)*config.sim_dt
    fps = Int(1/config.sim_dt)
    cam = FitToContentCamera(0.)
    function render_rec(t, dt)
        frame_index = Int(floor(t/dt)) + 1
        return render(scenes[frame_index], env, cam=cam)
    end
    return duration, fps, render_rec
end

function animate_scenes(scenes::Vector{Scene}, overlay::SceneOverlay)
    duration =length(scenes)*config.sim_dt
    fps = Int(1/config.sim_dt)
    cam = FitToContentCamera(0.)
    function render_rec(t, dt)
        frame_index = Int(floor(t/dt)) + 1
        return render(scenes[frame_index], env, [overlay], cam=cam)
    end
    return duration, fps, render_rec
end

function animate_states(pomdp::OCPOMDP, states::Vector{OCState}, overlay=nothing)
    duration = length(states)*pomdp.ΔT
    fps = Int(1/pomdp.ΔT)
    cam = FitToContentCamera(0.)
    function render_states(t, dt)
        state_index = Int(floor(t/dt)) + 1
        scene = state_to_scene(pomdp, states[state_index])
        return AutoViz.render(scene, pomdp.env, cam = cam)
    end
    return duration, fps, render_states
end

function animate_hist{B}(pomdp::OCPOMDP, hist::POMDPHistory{OCState,OCAction,OCState, B})
    duration = n_steps(hist)*pomdp.ΔT
    fps = Int(1/pomdp.ΔT)
    cam = FitToContentCamera(0.)
    function render_hist(t, dt)
        state_index = Int(floor(t/dt)) + 1
        scene = state_to_scene(pomdp, state_hist(hist)[state_index])
        return render(scene, pomdp.env, cam = cam)
    end
    return duration, fps, render_hist
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

mutable struct DecOverlay <: SceneOverlay
    up::Updater
    pomdp::OCPOMDP
    policy::Policy
    a::OCAction
    b::Dict{Int64, OCDistribution}
    o::Dict{Int64, OCObs}
    beliefs::Vector{DecBelief}
    ind::Int64
    verbosity::Int
    color::Colorant
    font_size::Int

    function DecOverlay(up::Updater, pomdp::OCPOMDP, policy::Policy, a::OCAction, b::Dict{Int64, OCDistribution},
        o::Dict{Int64, OCObs}, beliefs::Vector{DecBelief},
        ind::Int64 = 1,
        verbosity::Int=1,
        color::Colorant=colorant"white",
        font_size::Int=20,
        )

        new(up, pomdp, policy, a, b, o, beliefs, ind, verbosity, color,font_size)
    end
end

function AutoViz.render!(rendermodel::RenderModel, overlay::DecOverlay, scene::Scene, env::CrosswalkEnv)

    # render overlay
    font_size = overlay.font_size
    text_y = font_size
    text_y_jump = round(Int, font_size*1.2)

     # display acceleration
    add_instruction!( rendermodel, render_text, (@sprintf("a = %10.3f m/s^2", overlay.a.acc), 10, text_y, font_size, overlay.color), incameraframe=false)
    text_y += text_y_jump

    overlay.b = overlay.beliefs[overlay.ind]
#     println(overlay.b[2].it[1].ego)
    overlay.ind += 1
    belief = overlay.b
#     println(belief[2].it[1].ego)

    #Display state estimate
    for id in keys(belief)
        max, indmax = findmax(belief[id].p)
        for (i,s) in enumerate(belief[id].it)
            color=RGBA(105/255, 1., 1., 1 - belief[id].p[i])
        #             overlay.model.b[id].p[i]
#             println(overlay.model.b[id].p[i])
            p = belief[id].it[i].ped.posG
            add_instruction!(rendermodel, render_vehicle, (p.x, p.y, p.θ, pomdp.ped_type.length, pomdp.ped_type.width, color))
        end
    end

    # Update overlay
    ego = scene[findfirst(scene, 1)]
    if ego.state.posG.x != 5.0

        states = scene_to_states(scene, overlay.pomdp)
        overlay.o = generate_o(pomdp, states, overlay.a, states, rng)
#         overlay.b = update(up, overlay.b, overlay.a, overlay.o)
    end
    overlay.a = POMDPs.action(overlay.policy, belief)
    rendermodel
end


mutable struct DQNFeatures
    env::CrosswalkEnv
end

function AutoViz.render!(rendermodel::RenderModel, env_::DQNFeatures)
    env = env_.env
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
end

mutable struct InflateOverlay <: SceneOverlay
end

function AutoViz.render!(rendermodel::RenderModel, overlay::InflateOverlay, scene::Scene, _env::DQNFeatures)
    env = _env.env
    ego = scene[findfirst(scene, 1)]
    for veh in scene
        # increase pedestrian size for rendering
        if veh.def.class == AgentClass.PEDESTRIAN && is_observable(veh.state, ego.state, env)
            add_instruction!(rendermodel, render_vehicle, (veh.state.posG.x, veh.state.posG.y, veh.state.posG.θ,
                                              2,2,COLOR_CAR_OTHER))
        end
    end

    front = ego.state.posG + polar(VehicleDef().length/2, ego.state.posG.θ)
    # check if ego is in front of obstacle
    if front.x < env.params.roadway_length/2 - env.params.crosswalk_width/2
        obstacles = ConvexPolygon(4)
        obstacles.pts = [VecSE2(22, -1.5), VecSE2(28, -1.5), VecSE2(28, -13), VecSE2(22, -13)]
        obstacles.npts = 4; # a rectangle
        for obs in obstacles
            pts = Array{Float64}(2, obstacles.npts)
            for (i, pt) in enumerate(obstacles.pts)
                pts[1,i] = pt.x
                pts[2,i] = pt.y
            end

            add_instruction!(rendermodel, render_fill_region, (pts, colorant"black"))
        end
    end

    rendermodel
end
