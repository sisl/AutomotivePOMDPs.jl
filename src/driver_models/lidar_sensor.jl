function AutomotiveDrivingModels.observe!(lidar::LidarSensor, scene::Scene, env::OccludedEnv, egoid::Int64)
    state_ego = scene[findfirst(isequal(egoid), scene)].state
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

mutable struct LidarOverlay <: SceneOverlay
    sensor::LidarSensor
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

function AutoViz.render!(rendermodel::RenderModel, overlay::LidarOverlay, scene::Scene, env::OccludedEnv)
    ego = scene[findfirst(isequal(EGO_ID), scene)]
    observe!(overlay.sensor, scene, env, ego.id)
    return render_lidar!(rendermodel, overlay.sensor, ego.state.posG)
end
