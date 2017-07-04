### Function to spawn a flow of pedestrian
using Distributions

"""
    initial_pedestrian(env::CrosswalkEnv, scene::Scene, rng::AbstractRNG)
Create a new pedestrian entity at its initial state
"""
function initial_pedestrian(env::CrosswalkEnv, rng::AbstractRNG)
    # lateral position
    d_lat = Uniform(env.params.roadway_length - env.params.crosswalk_width, env.params.roadway_length + env.params.crosswalk_width)
    x0 = 0.5*rand(rng, d_lat)

    # vertical position
    y0 = -env.params.crosswalk_length/4

    # orientation
    θ = π/2

    #velocity
    v0 = rand(rng, Uniform(0., env.params.ped_max_speed))
    cw_roadway = Roadway([RoadSegment(2, [env.crosswalk])]);
    ped_initial_state = VehicleState(VecSE2(x0, y0, θ), env.crosswalk, cw_roadway, v0);
    # ped_initial_state = VehicleState(VecSE2(x0, y0, θ), env.crosswalk, env.roadway, v0)

    # new id, increment last id by 1
    id = scene.n + 1
    return Vehicle(ped_initial_state, PEDESTRIAN_DEF, id)
end
