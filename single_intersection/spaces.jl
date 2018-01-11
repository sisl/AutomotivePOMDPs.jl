#### STATE SPACE ############################

const LANE_ID_LIST = ["right_from_left", "straight_from_right", "left_from_right", "straight_from_left"]

function POMDPs.states(pomdp::OIPOMDP)
    env = pomdp.env
    V = linspace(0, pomdp.env.params.speed_limit, Int(floor(pomdp.env.params.speed_limit/pomdp.vel_res)) + 1)
    S_ego = linspace(pomdp.s_start, pomdp.s_goal, Int(floor((pomdp.s_goal - pomdp.s_start)/pomdp.pos_res)) + 1)
    state_space = Vector{OIState}()
    for lane_id in LANE_ID_LIST
        s_end = get_end(env.lane_map[lane_id])
        S = linspace(0, s_end, Int(floor(s_end/pomdp.pos_res)) + 1)
        for s_ego in S_ego
            for v_ego in V
                for s in S
                    for v in V
                        ego = ego_state(pomdp, s_ego, v_ego)
                        car = car_state(pomdp, lane_id, s, v)
                        push!(state_space, OIState(is_crash(pomdp, ego, car), ego, car))
                    end
                end
            end
        end
    end
    for s_ego in S_ego
        for v_ego in V
            ego = ego_state(pomdp, s_ego, v_ego)
            car = get_off_the_grid(pomdp)
            push!(state_space, OIState(false, ego, car))
        end
    end
    return state_space
end

function POMDPs.n_states(pomdp)
    N = 0
    n_vel = Int(floor(pomdp.env.params.speed_limit/pomdp.vel_res)) + 1
    n_s_ego = Int(floor((pomdp.s_goal - pomdp.s_start)/pomdp.pos_res)) + 1
    for lane_id in LANE_ID_LIST
        s_end = get_end(pomdp.env.lane_map[lane_id])
        N += (Int(floor(s_end/pomdp.pos_res)) + 1)*n_vel*n_vel*n_s_ego
    end
    # add the off the grid state
    N += n_vel*n_s_ego
    return N
end

function POMDPs.state_index(pomdp::OIPOMDP, s::OIState)
    size_V = Int(floor(pomdp.env.params.speed_limit/pomdp.vel_res)) + 1
    size_s_ego = Int(floor((pomdp.s_goal - pomdp.s_start)/pomdp.pos_res)) + 1

    s_ego = s.ego.posF.s
    v_ego = s.ego.v
    car_lane_tag = s.car.posF.roadind.tag
    @assert car_lane_tag.segment < 5 #5-6 are reserved for the ego car
    s_end = get_end(pomdp.env.roadway[car_lane_tag])
    size_s_car = Int(floor(s_end/pomdp.pos_res)) + 1
    s_car = s.car.posF.s
    v_car = s.car.v

    s_ego_ind = Int(ceil((s_ego - pomdp.s_start)/pomdp.pos_res)) + 1
    v_ego_ind = Int(ceil(v_ego/pomdp.vel_res)) + 1
    s_car_ind = Int(ceil(s_car/pomdp.pos_res)) + 1


    if !off_the_grid(pomdp, s.car)
        v_car_ind = Int(ceil(v_car/pomdp.vel_res)) + 1
        ind = sub2ind((size_V, size_s_car, size_V, size_s_ego), v_car_ind, s_car_ind, v_ego_ind, s_ego_ind)
        for i=1:car_lane_tag.segment-1
            lane_tag = pomdp.env.lane_map[LANE_ID_LIST[i]].tag
            s_end = get_end(pomdp.env.roadway[lane_tag])
            size_s = Int(floor(s_end/pomdp.pos_res)) + 1
            ind += size_s*size_V*size_s_ego*size_V
        end
    else
        ind = sub2ind((size_V, size_s_ego), v_ego_ind, s_ego_ind)
        for i=1:length(LANE_ID_LIST)
            lane_tag = pomdp.env.lane_map[LANE_ID_LIST[i]].tag
            s_end = get_end(pomdp.env.roadway[lane_tag])
            size_s = Int(floor(s_end/pomdp.pos_res)) + 1
            ind += size_s*size_V*size_s_ego*size_V
        end
    end
    return ind
end

### ACTION SPACE
POMDPs.actions(pomdp::OIPOMDP) = [OIAction(-4.0),
                           OIAction(-2.0),
                           OIAction(0.),
                           OIAction(1.5),
                           OIAction(3.0)]
POMDPs.n_actions(pomdp::OIPOMDP) = 5

function POMDPs.action_index(pomdp::OIPOMDP, a::OIAction)
    if a.acc == -4.0
        return 1
    elseif a.acc == -2
        return 2
    elseif a.acc == 0.
        return 3
    elseif a.acc == 1.5
        return 4
    else
        return 5
    end
end


### OBSRVATION SPACE

function POMDPs.observations(pomdp::OIPOMDP)
    return states(pomdp)
end

function POMDPs.obs_index(pomdp::OIPOMDP, o::OIObs)
    return state_index(pomdp, o)
end

function POMDPs.n_observations(pomdp::OIPOMDP)
    return n_states(pomdp)
end


#### HELPERS ####

function Base.show(io::IO, s::OIState)
    print(io, "OIState(", s.ego.posF.s, ", ", s.ego.v, ", ", s.car.posF.roadind.tag, ", ",
                          s.car.posF.s ,", ", s.car.v, ")")
end

"""
    ego_state(pomdp:ICPOMDP, s::Float64, v::Float64)
Helper that returns the ego car state given its position on the lane and the velocity
"""
function ego_state(pomdp::OIPOMDP, s::Float64, v::Float64)
    ego_lane = pomdp.env.lane_map["ego_left"]
    posF = Frenet(ego_lane, s)
    return VehicleState(posF, pomdp.env.roadway, v)
end

"""
    car_state(pomdp::OIPOMDP, lane_id::String, s::Float64, v::Float64)
Helper that returns the state of a car given the lane and the position on the lane
"""
function car_state(pomdp::OIPOMDP, lane_id::String, s::Float64, v::Float64)
    lane = pomdp.env.lane_map[lane_id]
    posF = Frenet(lane, s)
    return VehicleState(posF, pomdp.env.roadway, v)
end
function car_state(pomdp::OIPOMDP, lane::Lane, s::Float64, v::Float64)
    posF = Frenet(lane, s)
    return VehicleState(posF, pomdp.env.roadway, v)
end

"""
    is_crash(pomdp::OCPOMDP, ego::VehicleState, car::VehicleState)
use ADM collision routine to check if a combination of ego car state and car state
results in a collision
"""
function is_crash(pomdp::OIPOMDP, ego::VehicleState, car::VehicleState)
    return is_colliding(Vehicle(ego, pomdp.ego_type, 0), Vehicle(car, pomdp.car_type, 1))
end

"""
    get_off_the_grid(pomdp::OCPOMDP)
return the off the grid state
"""
function get_off_the_grid(pomdp::OIPOMDP)
    posG = pomdp.off_grid
    return VehicleState(posG, pomdp.env.roadway, 0.)
end

"""
    off_the_grid(pomdp::OCPOMDP, car::VehicleState)
Check if the current state of the car is in the grid
"""
function off_the_grid(pomdp::OIPOMDP, car::VehicleState)
    return car.posG == pomdp.off_grid
end

"""
    get_ego_s(pomdp::OIPOMDP)
returns all the possible position of the ego car along its lane as a linspace
"""
function get_ego_s_grid(pomdp::OIPOMDP, res::Float64 = pomdp.pos_res)
    return linspace(pomdp.s_start, pomdp.s_goal,
                    Int(floor((pomdp.s_goal - pomdp.s_start)/pomdp.pos_res)) + 1)
end

"""
    get_v_grid(pomdp::OIPOMDP)
returns all the possible velocities as a linspace
"""
function get_v_grid(pomdp::OIPOMDP, res::Float64 = pomdp.vel_res)
    return linspace(0, pomdp.env.params.speed_limit,
                    Int(floor(pomdp.env.params.speed_limit/pomdp.vel_res)) + 1)
end

"""
    get_lane_s(pomdp::OIPOMDP, lane_id::String)
returns all the possible position on the corresponding lane
"""
function get_lane_s(pomdp::OIPOMDP, lane_id::String, res::Float64 = pomdp.pos_res)
    s_end = get_end(pomdp.env.lane_map[lane_id])
    S = linspace(0, s_end, Int(floor(s_end/res)) + 1)
    return S
end
function get_lane_s(pomdp::OIPOMDP, lane::Lane, res::Float64 = pomdp.pos_res)
    s_end = get_end(lane)
    S = linspace(0, s_end, Int(floor(s_end/res)) + 1)
    return S
end

function get_end(lane::Lane)
    s_end = round(lane.curve[end].s)
    if s_end % 2 == 1
        s_end -= 1
    end
    return s_end
end

"""
    car_states(pomdp::OIPOMDP)
returns a vector of all the states that the other car can occupy
"""
function car_states(pomdp::OIPOMDP)
    env = pomdp.env
    V = linspace(0, pomdp.env.params.speed_limit, Int(floor(pomdp.env.params.speed_limit/pomdp.vel_res)) + 1)
    space = Vector{VehicleState}()
    for lane_id in LANE_ID_LIST
        s_end = get_end(env.lane_map[lane_id])
        S = linspace(0, s_end, Int(floor(s_end/pomdp.pos_res)) + 1)
        for s in S
            for v in V
                car = car_state(pomdp, lane_id, s, v)
                push!(space, car)
            end
        end
    end
    car = get_off_the_grid(pomdp)
    push!(space, car)
    return space
end

"""
    n_car_states(pomdp::OIPOMDP)
returns the number of states a car can occupy
"""
function n_car_states(pomdp::OIPOMDP)
    N = 0
    n_vel = Int(floor(pomdp.env.params.speed_limit/pomdp.vel_res)) + 1
    for lane_id in LANE_ID_LIST
        s_end = get_end(pomdp.env.lane_map[lane_id])
        N += (Int(floor(s_end/pomdp.pos_res)) + 1)*n_vel
    end
    return N+1
end

function n_ego_states(pomdp::OIPOMDP)
    n_vel = length(get_v_grid(pomdp))
    n_s_ego = length(get_ego_s_grid(pomdp))
    return n_vel*n_s_ego
end
