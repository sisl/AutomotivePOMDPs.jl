### Implementation of observation model


function POMDPs.observation(pomdp::SingleOIPOMDP, sp::SingleOIState)
   # return SingleOIDistribution([1.0], [sp])
    if !is_observable_fixed(sp, pomdp.env) || off_the_grid(pomdp, sp.car)
        o = SingleOIObs(false, sp.ego, get_off_the_grid(pomdp))
        return SingleOIDistribution([1.0], [o])
    elseif is_crash(pomdp, sp.ego, sp.car)
        return SingleOIDistribution([1.0], [sp])
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

    states = SingleOIObs[]
    sizehint!(states, 9)
    for neighbor in neighbors
        if in_bounds_car(pomdp, neighbor) && !is_crash(pomdp, ego, neighbor)
            push!(states, SingleOIObs(false, ego, neighbor))
        end
    end
    probs = ones(length(states))
    probs = normalize!(probs, 1)
    return SingleOIDistribution(probs, states)
end
function POMDPs.observation(pomdp::SingleOIPOMDP, a::SingleOIAction, sp::SingleOIState)
    return observation(pomdp, sp)
end


"""
Check if a car in s is observable or not
"""
function is_observable_fixed(s::SingleOIState, env::SimpleInterEnv)
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

"""
Check if a state is in bound
"""
function in_bounds_car(pomdp::SingleOIPOMDP, veh::VehicleState)
    lane = pomdp.env.roadway[veh.posF.roadind.tag]
    s = get_end(lane)
    return (veh.posF.s <= s) && (veh.v <= pomdp.env.params.speed_limit) && (veh.v >= 0.) && (veh.posF.s >= 0.)
end
