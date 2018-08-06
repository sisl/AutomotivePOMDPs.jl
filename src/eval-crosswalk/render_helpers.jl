const PEDESTRIAN_DEF =  VehicleDef(AgentClass.PEDESTRIAN, 1.0, 1.0)

function AutoViz.render!(rendermodel::RenderModel, env::CrosswalkEnv)
    roadway = gen_straight_roadway(2, env.params.roadway_length)
    render!(rendermodel, roadway)

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

function animate_record(rec::SceneRecord, overlay::SceneOverlay = EmptyOverlay, save_index::Int64=-1)
    duration =rec.nframes*config.sim_dt
    fps = Int(1/config.sim_dt)
    function render_rec(t, dt)
        frame_index = Int(floor(t/dt)) + 1
        scene = rec[frame_index-nframes(rec)]
        a = render(scene, env, [overlay], cam=cam, car_colors = get_colors(scene))
        if save_index == frame_index
            write_to_png(a,"save$frame_index.png")
        end
        return a
    end
    return duration, fps, render_rec
end

function get_colors(scene::Scene, ego_id::Int64 = 1)
    colors = Dict{Int64, Colorant}()
    for veh in scene
        colors[veh.id] = veh.id == ego_id ?  COLOR_CAR_EGO : COLOR_CAR_OTHER
    end
    return colors
end

function animate_record(rec::SceneRecord)
    duration =rec.nframes*config.sim_dt
    fps = Int(1/config.sim_dt)
    function render_rec(t, dt)
        frame_index = Int(floor(t/dt)) + 1
        return render(rec[frame_index-nframes(rec)], env, cam=cam)
    end
    return duration, fps, render_rec
end

mutable struct WaitAndGoOverlay <: SceneOverlay
    Δt::Float64
    model::WaitAndGoModel
    ttc::Float64

    target_id::Int
    verbosity::Int
    color::Colorant
    font_size::Int

    function WaitAndGoOverlay( Δt::Float64, model::WaitAndGoModel, ttc::Float64 = Inf,
        target_id::Int=1, verbosity::Int=1;
        color::Colorant=colorant"white",
        font_size::Int=20,
        )

        new(Δt, model, ttc, target_id, verbosity, color,font_size)
    end
end

function reset_overlay!(overlay::WaitAndGoOverlay, ego::Vehicle)
    reset_model!(overlay.model, ego)
end


function AutoViz.render!(rendermodel::RenderModel, overlay::WaitAndGoOverlay,
                        scene::Scene, env::CrosswalkEnv)

    # render overlay
    font_size = overlay.font_size
    text_y = font_size
    text_y_jump = round(Int, font_size*1.2)

    ego = scene[findfirst(scene, overlay.target_id)]

    # display ego id
    add_instruction!( rendermodel, render_text, (@sprintf("id = %d", overlay.target_id), 10, text_y, font_size, overlay.color), incameraframe=false)
        text_y += text_y_jump
    # display x position
    add_instruction!( rendermodel, render_text, (@sprintf("x = %10.3f m", ego.state.posG.x), 10, text_y, font_size, overlay.color), incameraframe=false)
        text_y += text_y_jump
    # display ego speed
    add_instruction!( rendermodel, render_text, (@sprintf("v = %10.3f m/s", ego.state.v), 10, text_y, font_size, overlay.color), incameraframe=false)
    text_y += text_y_jump

    # display acceleration
    add_instruction!( rendermodel, render_text, (@sprintf("a = %10.3f m/s^2", overlay.model.a), 10, text_y, font_size, overlay.color), incameraframe=false)
    text_y += text_y_jump
    # display ttc
    ttc_min = Inf
    for veh in scene
        if veh.id != overlay.target_id
            ttc = (env.params.lane_width - veh.state.posG.y)/veh.state.v
            if 0 < ttc < ttc_min
                ttc_min = ttc
            end
        end
    end
    overlay.ttc = ttc_min


    add_instruction!( rendermodel, render_text, (@sprintf("TTC = %10.3f s", overlay.ttc), 10, text_y, font_size, overlay.color), incameraframe=false)
    text_y += text_y_jump

    # display state
    state = "$(overlay.model.policy.reaching)$(overlay.model.policy.wait)$(overlay.model.policy.go)"
    if overlay.model.policy.reaching
        state = "reaching"
    elseif overlay.model.policy.go
        state = "go"
    elseif overlay.model.policy.wait
        state = "wait $(overlay.model.policy.N)"
    end
    add_instruction!( rendermodel, render_text, (state, 10, text_y, font_size, overlay.color), incameraframe=false)
    text_y += text_y_jump

    #display observation
    for veh in overlay.model.o
        if veh.id == overlay.target_id
            continue
        end
        color=RGBA(75./255, 66./255, 244./255, 0.5)
        x, y, θ = veh.state.posG.x, veh.state.posG.y, veh.state.posG.θ
        add_instruction!(rendermodel, render_vehicle, (x, y, θ, PEDESTRIAN_DEF.length, PEDESTRIAN_DEF.width, color))
    end

    # Update overlay
    observe!(overlay.model, scene, env.roadway, overlay.target_id)
    rendermodel
end

mutable struct QMDPOverlay <: SceneOverlay
    model::CrosswalkDriver
    ttc::Float64
    target_id::Int
    verbosity::Int
    color::Colorant
    font_size::Int

    function QMDPOverlay(model::CrosswalkDriver, ttc::Float64 = Inf,
        target_id::Int=1, verbosity::Int=1;
        color::Colorant=colorant"white",
        font_size::Int=20,
        )

        new(model, ttc, target_id, verbosity, color,font_size)
    end
end

function reset_overlay!(overlay::QMDPOverlay, ego::Vehicle)
    reset_model!(overlay.model, ego)
end

function AutoViz.render!(rendermodel::RenderModel, overlay::QMDPOverlay, scene::Scene, env::CrosswalkEnv)

    # render overlay
    font_size = overlay.font_size
    text_y = font_size
    text_y_jump = round(Int, font_size*1.2)

    ego = scene[findfirst(scene, overlay.target_id)]

    # display ego id
    add_instruction!( rendermodel, render_text, (@sprintf("Ego car state"), 10, text_y, font_size, overlay.color), incameraframe=false)
        text_y += text_y_jump
    # display x position
    add_instruction!( rendermodel, render_text, (@sprintf("x = %10.3f m", ego.state.posG.x), 10, text_y, font_size, overlay.color), incameraframe=false)
        text_y += text_y_jump
    # display ego speed

    add_instruction!( rendermodel, render_text, (@sprintf("v = %10.3f m/s", ego.state.v), 10, text_y, font_size, overlay.color), incameraframe=false)
    text_y += text_y_jump
    # display acceleration
    add_instruction!( rendermodel, render_text, (@sprintf("a = %10.3f m/s^2", overlay.model.a), 10, text_y, font_size, overlay.color), incameraframe=false)
    text_y += text_y_jump
    # # display ttc
    # ttc_min = Inf
    # for veh in scene
    #     if veh.id != overlay.target_id
    #         ttc = (env.params.lane_width - veh.state.posG.y)/veh.state.v
    #         if 0 < ttc < ttc_min
    #             ttc_min = ttc
    #         end
    #     end
    # end
    # overlay.ttc = ttc_min
    # add_instruction!( rendermodel, render_text, (@sprintf("TTC = %10.3f s", overlay.ttc), 10, text_y, font_size, overlay.color), incameraframe=false)
    # text_y += text_y_jump

    #Display ray
    for oid in keys(overlay.model.o)
        if oid == OFF_KEY; continue; end
        oped = overlay.model.o[oid].ped
        ego_front = get_front(ego)
        line_pts = [oped.posG.x oped.posG.y; ego_front.x ego_front.y]'
        add_instruction!( rendermodel, render_line, (line_pts, COLOR_CAR_EGO, 0.1))
    end

    for veh in scene
        if veh.id == 1 || veh.id ∈ keys(overlay.model.o); continue; end
        ego_front = get_front(ego)
        line_pts = [veh.state.posG.x veh.state.posG.y; ego_front.x ego_front.y]'
        add_instruction!( rendermodel, render_line, (line_pts, RGB(1.0, 0., 0.), 0.1))
    end

    #Display state estimate
    for id in keys(overlay.model.b)
        max, indmax = findmax(overlay.model.b[id].p)
        for (i,s) in enumerate(overlay.model.b[id].it)
            prob = overlay.model.b[id].p[i]
            if prob > 1e-3
                color=RGBA(75./255, 66./255, 244./255, overlay.model.b[id].p[i])

#             overlay.model.b[id].p[i]
#             println(overlay.model.b[id].p[i])
                p = overlay.model.b[id].it[i].ped.posG
                if haskey(overlay.model.o, id)
                    px = overlay.model.o[id].ped.posG.x
                else
                    px = p.x
                end
                add_instruction!(rendermodel, render_vehicle, (px, p.y, p.θ, PEDESTRIAN_DEF.length, PEDESTRIAN_DEF.width, color))
            end
        end
    end

    # display labels
    x_ego_lab = 2.5
    y_ego_lab = -5.
    θ_ego_lab = 0.
    add_instruction!(rendermodel, render_vehicle, (x_ego_lab, y_ego_lab, θ_ego_lab,
                                                   VehicleDef().length, VehicleDef().width, COLOR_CAR_EGO))
    text_y += 12*text_y_jump
    add_instruction!(rendermodel, render_text, (@sprintf("Ego car"), 120, text_y, font_size, overlay.color), incameraframe=false)
    x_other_lab = 2.5
    y_other_lab = -8
    θ_other_lab = 0.
    add_instruction!(rendermodel, render_vehicle, (x_other_lab, y_other_lab, θ_other_lab,
                                                  PEDESTRIAN_DEF.length, PEDESTRIAN_DEF.width, COLOR_CAR_OTHER))
    text_y += 2.*text_y_jump
    add_instruction!(rendermodel, render_text, (@sprintf("Pedestrian"), 120, text_y, font_size, overlay.color), incameraframe=false)
    text_y += text_y_jump
    add_instruction!(rendermodel, render_text, (@sprintf("ground truth"), 120, text_y, font_size, overlay.color), incameraframe=false)
    x_belief_lab = 2.5
    y_belief_lab = -11
    θ_belief_lab = 0.
    add_instruction!(rendermodel, render_vehicle, (x_belief_lab, y_belief_lab, θ_belief_lab,
                                                 PEDESTRIAN_DEF.length, PEDESTRIAN_DEF.width, RGB(75./255, 66./255, 244./255)))
    text_y += 2.*text_y_jump
    add_instruction!(rendermodel, render_text, (@sprintf("Belief"), 120, text_y, font_size, overlay.color), incameraframe=false)

    add_instruction!(rendermodel, render_text, (@sprintf("Absent state"), 400, 550, font_size, overlay.color), incameraframe=false)

    add_instruction!( rendermodel, render_line, ([35. -5; 36 -5]', RGB(1.0, 0., 0.), 0.1))
    add_instruction!(rendermodel, render_text, (@sprintf("occluded target"), 730, 405, font_size, overlay.color), incameraframe=false)

    add_instruction!( rendermodel, render_line, ([35. -7; 36 -7]', COLOR_CAR_EGO, 0.1))
    add_instruction!(rendermodel, render_text, (@sprintf("visible target"), 730, 445, font_size, overlay.color), incameraframe=false)

    # Display everyones ID
    # for veh in scene
    #     add_instruction!(rendermodel, render_text, ("$(veh.id)", veh.state.posG.x, veh.state.posG.y, font_size, colorant"black"), incameraframe=true)
    # end
    # Update overlay
    observe!(overlay.model, scene, env.roadway, overlay.target_id)
    rendermodel
end
