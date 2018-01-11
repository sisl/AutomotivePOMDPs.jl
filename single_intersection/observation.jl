### Implementation of observation model


function POMDPs.observation(pomdp::OIPOMDP, sp::OIState)
   # return OIDistribution([1.0], [sp])
    if !is_observable(sp, pomdp.env) || off_the_grid(pomdp, sp.car)
        o = OIObs(false, sp.ego, get_off_the_grid(pomdp))
        return OIDistribution([1.0], [o])
    elseif is_crash(pomdp, sp.ego, sp.car)
        return OIDistribution([1.0], [sp])
    end
    ego = sp.ego
    car = sp.car
    lane = pomdp.env.roadway[car.posF.roadind.tag]
    neighbors = Vector{VehicleState}(9)
    neighbors[1] = car_state(pomdp, lane, car.posF.s - pomdp.pos_res, car.v)
    neighbors[2] = car_state(pomdp, lane, car.posF.s + pomdp.pos_res, car.v)
    neighbors[3] = car_state(pomdp, lane, car.posF.s - pomdp.pos_res, car.v - pomdp.vel_res)
    neighbors[4] = car_state(pomdp, lane, car.posF.s + pomdp.pos_res, car.v - pomdp.vel_res)
    neighbors[5] = car_state(pomdp, lane, car.posF.s - pomdp.pos_res, car.v + pomdp.vel_res)
    neighbors[6] = car_state(pomdp, lane, car.posF.s + pomdp.pos_res, car.v + pomdp.vel_res)
    neighbors[7] = car_state(pomdp, lane, car.posF.s, car.v - pomdp.vel_res)
    neighbors[8] = car_state(pomdp, lane, car.posF.s, car.v + pomdp.vel_res)
    neighbors[9] = car_state(pomdp, lane, car.posF.s, car.v)

    states = OIObs[]
    sizehint!(states, 9)
    for neighbor in neighbors
        if in_bounds_car(pomdp, neighbor) && !is_crash(pomdp, ego, neighbor)
            push!(states, OIObs(false, ego, neighbor))
        end
    end
    probs = ones(length(states))
    probs = normalize!(probs, 1)
    return OIDistribution(probs, states)
end
function POMDPs.observation(pomdp::OIPOMDP, a::OIAction, sp::OIState)
    return observation(pomdp, sp)
end


"""
Check if a car in s is observable or not
"""
function is_observable(s::OIState, env::IntersectionEnv)
    m = length(env.obstacles)
    car = s.car
    ego = s.ego
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
function is_observable(ego::VehicleState, car::VehicleState, env::IntersectionEnv)
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

"""
Check if a state is in bound
"""
function in_bounds_car(pomdp::OIPOMDP, veh::VehicleState)
    lane = pomdp.env.roadway[veh.posF.roadind.tag]
    s = get_end(lane)
    return (veh.posF.s <= s) && (veh.v <= pomdp.env.params.speed_limit) && (veh.v >= 0.) && (veh.posF.s >= 0.)
end
