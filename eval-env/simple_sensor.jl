# Simple sensor model for the ego car

"""
Sensor model with gaussian noise in position and velocity measurement
"""
type SimpleSensor
    pos_noise::Float64
    vel_noise::Float64
end

"""
    measure(ego::VehicleState, cars::EntityFrame{VehicleState, VehicleDef, Int64}, model::SimpleSensor, env::CrosswalkEnv)
return a measurement of a given scene from the point of view of the ego car
"""
function measure(ego::VehicleState, cars::Vector{Entity{VehicleState, VehicleDef, Int64}}, model::SimpleSensor, env::CrosswalkEnv)
    observed = Vehicle[] # sizehint!(Vector{Vehicle(0)}, 5)
    for veh in cars
        car = veh.state
        if is_observable(car, ego, cars, env)
            obs_state = VehicleState(VecSE2(car.posG.x + model.pos_noise*randn(),
                                                      car.posG.y + model.pos_noise*randn(),
                                                      car.posG.θ),
                                                      env.roadway,
                                                       car.v + model.vel_noise*randn())
            push!(observed, Vehicle(obs_state, veh.def, veh.id))
        end
    end
    return observed
end

"""
Return true if a car is observable
"""
function is_observable(car::VehicleState, ego::VehicleState, cars::Vector{Entity{VehicleState, VehicleDef, Int64}}, env::CrosswalkEnv)
    @assert !(car in cars)
    n = length(cars)
    m = length(env.obstacles)
    angle = atan2(car.posG.y - ego.posG.y, car.posG.x - ego.posG.x)
    ray = VecSE2(ego.posG.x, ego.posG.y, angle)
    for i= 1:m
        if is_colliding(ray, env.obstacles[i])
            return false
        end
    end
    for i= 1:n
        if cars[i].state != car
            Pcar = ConvexPolygon(4)
            to_oriented_bounding_box!(Pcar, cars[i])
            if is_colliding(ray, Pcar)
                return false
            end
        end
    end
    return true
end
