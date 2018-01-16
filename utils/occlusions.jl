
function is_observable(ego::VehicleState, car::VehicleState, env::OccludedEnv)
    m = length(env.obstacles)
    front = ego.posG + polar(VehicleDef().length/2, ego.posG.θ)
    angle = atan2(car.posG.y - front.y, car.posG.x - front.x)
    ray = Projectile(VecSE2(front.x, front.y, angle), 1.0)
    for i = 1:m
        if is_colliding(ray, env.obstacles[i], car.posG)
            return false
        end
    end
    return true
end

function AutomotiveDrivingModels.is_colliding(P::Projectile, poly::ConvexPolygon, A::VecSE2)
    # collides if at least one of the segments collides with the ray
    point_time = sqrt(abs2(A - P.pos))
    for i in 1 : length(poly)
        seg = get_edge(poly, i)
        obs_time = get_intersection_time(P, seg)
        if !isinf(obs_time) && obs_time < point_time
            return true
        end
    end
    false
end
