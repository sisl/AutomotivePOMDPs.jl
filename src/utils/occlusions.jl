

"""
    is_observable_fixed(ego::VehicleState, car::VehicleState, env::OccludedEnv)
    is_observable_fixed(x::Float64, y::Float64, ego::VehicleState, env::OccludedEnv)
Occlusion checker considering only fixed obstacles in the environment:
- is_observable_fixed(ego::VehicleState, car::VehicleState, env::OccludedEnv) check if car is observable from ego
- is_observable_fixed(x::Float64, y::Float64, ego::VehicleState, env::OccludedEnv) check if x, y is observable from ego
"""
function is_observable_fixed(ego::VehicleState, car::VehicleState, env::OccludedEnv)
    m = length(env.obstacles)
    front = ego.posG + polar(VehicleDef().length/2, ego.posG.θ)
    angle = atan(car.posG.y - front.y, car.posG.x - front.x)
    ray = Projectile(VecSE2(front.x, front.y, angle), 1.0)
    for i = 1:m
        if is_colliding(ray, env.obstacles[i], car.posG)
            return false
        end
    end
    return true
end

# check if a point in space is observable
function is_observable_fixed(x::Float64, y::Float64, ego::VehicleState, env::OccludedEnv)
    m = length(env.obstacles)
    front = ego.posG + polar(VehicleDef().length/2, ego.posG.θ)
    angle = atan(y - front.y, x - front.x)
    ray = Projectile(VecSE2(front.x, front.y, angle), 1.0)
    for i = 1:m
        if is_colliding(ray, env.obstacles[i], VecSE2(x,y,0.))
            return false
        end
    end
    return true
end

"""
    is_observable_dyna(ego::Vehicle, car::Vehicle, scene::Scene)
Occlusion checker verifying is `car` is observable from `ego`
It considers only other vehicles (not pedestrians, not fixed obstacles)
"""
function is_observable_dyna(ego::Vehicle, car::Vehicle, scene::Scene)
    front = ego.state.posG + polar(ego.def.length/2, ego.state.posG.θ)
    angle = atan(car.state.posG.y - front.y, car.state.posG.x - front.x)
    ray = Projectile(VecSE2(front.x, front.y, angle), 1.0)
    for veh in scene
        if veh.id == car.id || veh.id == ego.id || veh.def == PEDESTRIAN_DEF
            continue
        end
        box = get_oriented_bounding_box(veh)
        if is_colliding(ray, box, car.state.posG)
            return false
        end
    end
    return true
end



"""
    is_observable(ego::Vehicle, car::Vehicle, scene::Scene, env::OccludedEnv)
Check if car is observable from ego, by considering both fixed and dynamic obstacles
"""
function is_observable(ego::Vehicle, car::Vehicle, scene::Scene, env::OccludedEnv)
    return is_observable_dyna(ego, car, scene) && is_observable_fixed(ego.state, car.state, env)
end

function AutomotiveDrivingModels.is_colliding(P::Projectile, poly::ConvexPolygon, A::VecSE2)
    # collides if at least one of the segments collides with the ray
    point_time = norm(VecE2(A - P.pos))
    for i in 1 : length(poly)
        seg = get_edge(poly, i)
        obs_time = get_intersection_time(P, seg)
        if !isinf(obs_time) && obs_time < point_time
            return true
        end
    end
    false
end

# check if x,y is in veh
function AutomotiveDrivingModels.is_colliding(x::Float64, y::Float64, veh::VehicleState)
    v = VecSE2(x,y, 0.)
    poly = get_oriented_bounding_box(Vehicle(veh, VehicleDef(), -1))
    return contains(poly, v)
end
