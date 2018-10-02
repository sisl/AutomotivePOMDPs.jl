# Define several base scenarios derived from the large urban environment

# ego + 2 cars
const TwoCars = UrbanPOMDP(max_cars=2, max_peds=0, max_obstacles=0, car_birth=0.9, ped_birth=0.9)

# ego + 1 ped + 1 car
const PedCar = UrbanPOMDP(max_cars=1, max_peds=1, max_obstacles=0, car_birth=0.9, ped_birth=0.9)

# ego + 1 ped + 1 obstacle
const ObsPed = UrbanPOMDP(max_cars=0, max_peds=1, max_obstacles=1, car_birth=0.9, ped_birth=0.9)

# ego + 1 car + 1 obstacle
const ObsCar = UrbanPOMDP(max_cars=1, max_peds=0, max_obstacles=1, car_birth=0.9, ped_birth=0.9)

# Wrappers
@with_kw struct TwoCarsScenario
    problem::UrbanPOMDP = TwoCars
end

@with_kw struct PedCarScenario
    problem::UrbanPOMDP = PedCar
end

@with_kw struct ObsPedScenario
    problem::UrbanPOMDP = ObsPed
end

@with_kw struct ObsCarScenario
    problem::UrbanPOMDP = ObsCar
end


### Extract input

# return a list of tuple (pb_id, pb_input)
function get_problem_input(ego::Vector{Float64},
                           car_map::OrderedDict{String, Vector{Float64}},
                           ped_map::OrderedDict{String, Vector{Float64}},
                           obs_map::OrderedDict{String, Vector{Float64}})
    inputs = Tuple{Symbol, Vector{Float64}}[]
    # build obs ped
    for (ped, ped_state) in ped_map
        obs_states = []
        for (obs, obs_state) in obs_map
            push!(obs_states, obs_state)

        end
        feature_vec = vcat(ego, ped_state, obs_states...)
        push!(inputs, (:obsped, feature_vec))
    end

    # build obs car
    for (car, car_state) in car_map
        obs_states = []
        for (obs, obs_state) in obs_map
            push!(obs_states, obs_state)
        end
        feature_vec = vcat(ego, car_state, obs_states...)
        push!(inputs, (:obscar, feature_vec))
    end

    # build ped car
    for (ped, ped_state) in ped_map
        for (car, car_state) in car_map
            feature_vec = vcat(ego, car_state, ped_state)
            push!(inputs, (:pedcar, feature_vec))
        end
    end

    # build twocars
     for (car, car_state) in car_map
        for (car, car_state) in car_map
            feature_vec = vcat(ego, car_state, car_state)
            push!(inputs, (:twocars, feature_vec))
        end
    end
    return inputs
end

function decompose_input(pomdp::UrbanPOMDP, o::Vector{Float64})
    ego, car_map, ped_map, obs_map = split_o(o, pomdp)
    inputs = get_problem_input(ego, car_map, ped_map, obs_map)
    return inputs
end
