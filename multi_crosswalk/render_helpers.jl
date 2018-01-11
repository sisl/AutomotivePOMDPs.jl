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

function animate_hist(pomdp::OCPOMDP, hist)
    duration = n_steps(hist)*pomdp.ΔT
    fps = Int(1/pomdp.ΔT)
    cam = FitToContentCamera(0.)
    function render_hist(t, dt)
        state_index = Int(floor(t/dt)) + 1
        scene = state_hist(hist)[state_index]
        return AutoViz.render(scene, pomdp.env, cam = cam)
    end
    return duration, fps, render_hist
end


function animate_record(rec::SceneRecord, pomdp::OCPOMDP, overlay::SceneOverlay, cam=FitToContentCamera(0.0))
    duration =rec.nframes*pomdp.ΔT
    fps = Int(1/pomdp.ΔT)
    function render_rec(t, dt)
        frame_index = Int(floor(t/dt)) + 1
        return render(rec[frame_index-nframes(rec)], pomdp.env, [overlay], cam=cam)
    end
    return duration, fps, render_rec
end

struct EmptyOverlay <: SceneOverlay end

function AutoViz.render!(rendermodel::RenderModel, overlay::EmptyOverlay, scene::Scene, env::CrosswalkEnv)
    rendermodel
end

mutable struct LidarOverlay <: SceneOverlay
    sensor::LidarSensor
end


function AutomotiveDrivingModels.observe!(lidar::LidarSensor, scene::Scene, env::CrosswalkEnv, egoid::Int64)
    state_ego = scene[findfirst(scene, egoid)].state
    ego_vel = polar(state_ego.v, state_ego.posG.θ)
    for (i,angle) in enumerate(lidar.angles)
        ray_angle = state_ego.posG.θ + angle
        ray_vec = polar(1.0, ray_angle)
        ray = VecSE2(state_ego.posG.x, state_ego.posG.y, ray_angle)

        range = lidar.max_range
        range_rate = 0.0
        for veh in scene
            if veh.id != egoid
                to_oriented_bounding_box!(lidar.poly, veh)

                range2 = AutomotiveDrivingModels.get_collision_time(ray, lidar.poly, 1.0)
                if !isnan(range2) && range2 < range
                    range = range2
                    relative_speed = polar(veh.state.v, veh.state.posG.θ) - ego_vel
                    range_rate = proj(relative_speed, ray_vec, Float64)
                end
            end
        end
        range_obs = lidar.max_range
        for obs in env.obstacles
            range_obs2 = AutomotiveDrivingModels.get_collision_time(ray, obs, 1.0)
            if !isnan(range_obs2) && range_obs2 < range_obs
                range_obs = range_obs2
                relative_speed = polar(0., 0.) - ego_vel
                range_rate = proj(relative_speed, ray_vec, Float64)
            end
        end

        if range_obs < range
            lidar.ranges[i] = range_obs
            lidar.range_rates[i] = range_rate
        else
            lidar.ranges[i] = range
            lidar.range_rates[i] = range_rate
        end
    end

    lidar
end

function render_ray!(rendermodel::RenderModel, ray::VecSE2;
    color::Colorant=convert(RGB, HSV(rand(1:360), 0.8, 0.8)),
    origin_radius::Float64 = 0.1,
    length::Float64 = 2.0,
    line_width::Float64 = 0.05,
    )

    pos2 = ray + polar(length, ray.θ)
    add_instruction!(rendermodel, render_line_segment, (ray.x, ray.y, pos2.x, pos2.y, color, line_width))
    add_instruction!(rendermodel, render_circle, (ray.x, ray.y, origin_radius, color, color, 0.0))
    rendermodel
end

function render_lidar!(rendermodel::RenderModel, lidar::LidarSensor, posG::VecSE2;
    color::Colorant=colorant"white",
    line_width::Float64 = 0.05,
    )
    for (angle, range, range_rate) in zip(lidar.angles, lidar.ranges, lidar.range_rates)
        ray = VecSE2(posG.x, posG.y, posG.θ + angle)
#         println(ray, range)
        t = 2 / (1 + exp(-range_rate))
        ray_color = t > 1.0 ? lerp(RGBA(1.0,1.0,1.0,0.5), RGBA(0.2,0.2,1.0,0.5), t/2) :
                              lerp(RGBA(1.0,0.2,0.2,0.5), RGBA(1.0,1.0,1.0,0.5), t)
        render_ray!(rendermodel::RenderModel, ray, color=ray_color, length=range, line_width=line_width)
    end
    rendermodel
end

function AutoViz.render!(rendermodel::RenderModel, overlay::LidarOverlay, scene::Scene, env::CrosswalkEnv)
    ego = scene[findfirst(scene, EGO_INDEX)]
    observe!(overlay.sensor, scene, env, ego.id)
    return render_lidar!(rendermodel, overlay.sensor, ego.state.posG)
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
