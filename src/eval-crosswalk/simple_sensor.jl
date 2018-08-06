# Simple sensor model for the ego car

# """
# Sensor model with gaussian noise in position and velocity measurement
# """
# type SimpleSensor
#     pos_noise::Float64
#     vel_noise::Float64
# end
#
# """
#     measure(ego::VehicleState, cars::EntityFrame{VehicleState, VehicleDef, Int64}, model::SimpleSensor, env::CrosswalkEnv)
# return a measurement of a given scene from the point of view of the ego car
# """
# function measure(ego::Vehicle, cars::Vector{Entity{VehicleState, VehicleDef, Int64}}, model::SimpleSensor, env::CrosswalkEnv)
#     observed = Vehicle[] # sizehint!(Vector{Vehicle(0)}, 5)
#     for veh in cars
#         car = veh.state
#         if is_observable(car, ego.state, env)
#             obs_state = VehicleState(VecSE2(car.posG.x + model.pos_noise*randn(),
#                                                       car.posG.y + model.pos_noise*randn(),
#                                                       car.posG.θ),
#                                                       env.roadway,
#                                                       car.v + model.vel_noise*randn())
#             push!(observed, Vehicle(obs_state, veh.def, veh.id))
#         end
#     end
#     insert!(observed, 1, ego)
#     return observed
# end

# """
# Return true if a car is observable
# """
# function is_observable(car::VehicleState, ego::VehicleState, cars::Vector{Entity{VehicleState, VehicleDef, Int64}}, env::CrosswalkEnv)
#     @assert !(car in cars)
#     n = length(cars)
#     m = length(env.obstacles)
#     front = ego.posG + polar(VehicleDef().length/2, ego.posG.θ)
#     angle = atan2(car.posG.y - front.y, car.posG.x - front.x)
#     ray = VecSE2(front.x, front.y, angle)
#     ray_length = abs2(ray - car.posG)
#     if isinf(car.v)
#         return false
#     end
#     for i= 1:m
#         if is_colliding(ray, env.obstacles[i], ray_length)
#             return false
#         end
#     end
#     # for i= 1:n
#     #     if cars[i].state != car
#     #         Pcar = ConvexPolygon(4)
#     #         to_oriented_bounding_box!(Pcar, cars[i])
#     #         if is_colliding(ray, Pcar)
#     #             return false
#     #         end
#     #     end
#     # end
#     return true
# end

function is_observable(car::VehicleState, ego::VehicleState, env::CrosswalkEnv)
    m = length(env.obstacles)
    front = ego.posG + polar(VehicleDef().length/2, ego.posG.θ)
    angle = atan2(car.posG.y - front.y, car.posG.x - front.x)
    ray = Projectile(VecSE2(front.x, front.y, angle), 1.0)
    if isinf(car.v)
        return false
    end
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
