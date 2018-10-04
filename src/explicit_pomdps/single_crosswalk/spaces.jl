####### STATE SPACE ##############################

function POMDPs.states(pomdp::SingleOCPOMDP)
    env = pomdp.env
    rl = env.params.roadway_length
    cw = env.params.crosswalk_width
    lw = env.params.lane_width
    cl = env.params.crosswalk_length
    x_ped = 0.5*(env.params.roadway_length - env.params.crosswalk_width + 1)
    Y = LinRange(-cl/4, cl/4, Int(floor(cl/2/pomdp.pos_res)) + 1)
    V_ped = LinRange(0, env.params.ped_max_speed, Int(floor(env.params.ped_max_speed/pomdp.vel_res)) + 1)
    X = LinRange(5., rl/2 + 2*cw , Int(floor((rl/2 + 2*cw -5)/pomdp.pos_res)) + 1)
    V = LinRange(0., env.params.speed_limit, Int(floor(env.params.speed_limit/pomdp.vel_res)) + 1)
    y_ego = 0. #XXX might need to be a parameter
    space = SingleOCState[]
    for x in X
        for v in V
            for y in Y
                for v_ped in V_ped
                    ego = VehicleState(VecSE2(x, y_ego, 0.), env.roadway, v)
                    ped = VehicleState(VecSE2(x_ped, y, pi/2), env.roadway, v_ped)
                    push!(space, SingleOCState(is_crash(pomdp, ego, ped), ego, ped))
                end
            end
        end
    end
    for x in X
        for v in V
            ego = VehicleState(VecSE2(x, y_ego, 0.), env.roadway, v)
            ped =  get_off_the_grid(pomdp)
            push!(space, SingleOCState(is_crash(pomdp, ego, ped), ego, ped))
        end
    end
    return space
end

function POMDPs.state_index(pomdp::SingleOCPOMDP, s::SingleOCState)
    env = pomdp.env
    rl = env.params.roadway_length
    cw = env.params.crosswalk_width
    lw = env.params.lane_width
    cl = env.params.crosswalk_length
    size_x = Int(floor((rl/2 + 2*cw -5)/pomdp.pos_res) + 1)
    size_v = Int(floor(env.params.speed_limit/pomdp.vel_res) + 1)
    size_y = Int(floor(cl/2/pomdp.pos_res) + 1)
    size_v_ped = Int(floor(env.params.ped_max_speed/pomdp.vel_res) + 1)


    x = s.ego.posG.x
    v = s.ego.v
    y = s.ped.posG.y
    v_ped = s.ped.v

    y_0 = -env.params.crosswalk_length/4
    x_0 = 5
    x_ind = Int(ceil((x - x_0)/pomdp.pos_res)) + 1
    v_ind = Int(ceil(v/pomdp.vel_res)) + 1
    y_ind = Int(ceil((y - y_0)/pomdp.pos_res)) + 1

    if !off_the_grid(pomdp, s.ped)
        v_ped_ind = Int(ceil(v_ped/pomdp.vel_res)) + 1
        i = LinearIndices((size_v_ped, size_y, size_v, size_x))[v_ped_ind, y_ind, v_ind, x_ind]
    else
        off_grid_ind = LinearIndices((size_v, size_x))[v_ind, x_ind]
        i = size_x*size_v*size_y*size_v_ped + off_grid_ind
    end
    return i
end


function POMDPs.n_states(pomdp::SingleOCPOMDP)
    env = pomdp.env
    rl = env.params.roadway_length
    cw = env.params.crosswalk_width
    lw = env.params.lane_width
    cl = env.params.crosswalk_length
    size_x = floor((rl/2 + 2*cw -5)/pomdp.pos_res) + 1
    size_v = floor(env.params.speed_limit/pomdp.vel_res) + 1
    size_y = floor(cl/2/pomdp.pos_res) + 1
    size_v_ped = floor(env.params.ped_max_speed/pomdp.vel_res) + 1
    return Int(size_x*size_v*(size_y*size_v_ped + 1))
end


### OBSERVATION SPACE


function POMDPs.observations(pomdp::SingleOCPOMDP)
    return states(pomdp)
end

function POMDPs.obs_index(pomdp::SingleOCPOMDP, o::SingleOCObs)
    return state_index(pomdp, o)
end

function POMDPs.n_observations(pomdp::SingleOCPOMDP)
    return n_states(pomdp)
end



#### ACTION SPACE
function POMDPs.actions(pomdp::SingleOCPOMDP)
    return [SingleOCAction(-2*pomdp.max_acc), SingleOCAction(-pomdp.max_acc), SingleOCAction(0.0), SingleOCAction(pomdp.max_acc)]
end

function POMDPs.n_actions(pomdp::SingleOCPOMDP)
    return 4
end

function POMDPs.action_index(pomdp::SingleOCPOMDP, a::SingleOCAction)
    if a.acc < -2.
        return 1
    elseif -2. <= a.acc < 0.
        return 2
    elseif a.acc == 0.
        return 3
    else
        return 4
    end
end

######################### HELPERS ##################################################################

"""
    is_crash(pomdp::SingleOCPOMDP, ego::VehicleState, ped::VehicleState)
use ADM collision routine to check if a combination of ego car state and pedestrian state
results in a collision
"""
function is_crash(pomdp::SingleOCPOMDP, ego::VehicleState, ped::VehicleState)
    return is_colliding(Vehicle(ego, pomdp.ego_type, EGO_ID), Vehicle(ped, pomdp.ped_type, PED_ID))
end
"""
    is_crash(pomdp::SingleOCPOMDP, ego::VehicleState, ped::VehicleState)
use ADM collision routine to check if a state results in a collision
"""
function is_crash(pomdp::SingleOCPOMDP, s::SingleOCState)
    return is_crash(pomdp, s.ego, s.ped)
end

"""
    off_the_grid(pomdp::SingleOCPOMDP, ped::VehicleState)
Check if the current state of the pedestrian is in the grid
"""
function off_the_grid(pomdp::SingleOCPOMDP, ped::VehicleState)
    return ped == get_off_the_grid(pomdp)
end

"""
    get_off_the_grid(pomdp::SingleOCPOMDP)
return the off the grid pedestrian state
"""
function get_off_the_grid(pomdp::SingleOCPOMDP)
    env = pomdp.env
    obstacle = env.obstacles[1]
    obs_center = get_center(obstacle)
    return VehicleState(VecSE2(obs_center.x, obs_center.y, 0.), pomdp.env.roadway, 0.0)
end

"""
    get_X_grid(pomdp::SingleOCPOMDP)
Helper function to return the X discretization
"""
function get_X_grid(pomdp::SingleOCPOMDP)
    env = pomdp.env
    rl = env.params.roadway_length
    cw = env.params.crosswalk_width
    lw = env.params.lane_width
    cl = env.params.crosswalk_length
    return LinRange(5., rl/2 + 2*cw , Int(floor((rl/2 + 2*cw -5)/pomdp.pos_res)) + 1)
end

"""
    get_V_grid(pomdp::SingleOCPOMDP)
Helper function to return the ego velSingleOCity discretization
"""
function get_V_grid(pomdp::SingleOCPOMDP)
    env = pomdp.env
    V = LinRange(0., env.params.speed_limit, Int(floor(env.params.speed_limit/pomdp.vel_res)) + 1)
end

"""
    get_Y_grid(pomdp::SingleOCPOMDP)
Helper function to return the Y discretization
"""
function get_Y_grid(pomdp::SingleOCPOMDP)
    cl = pomdp.env.params.crosswalk_length
    return LinRange(-cl/4, cl/4, Int(floor(cl/2/pomdp.pos_res)) + 1)
end

"""
    get_V_ped_grid(pomdp::SingleOCPOMDP)
Helper funcion to return the VelSingleOCity discretization for the pedestrian
"""
function get_V_ped_grid(pomdp::SingleOCPOMDP)
    env = pomdp.env
    V_ped = LinRange(0, env.params.ped_max_speed, Int(floor(env.params.ped_max_speed/pomdp.vel_res)) + 1)
end

"""
    get_y_index(pomdp::SingleOCPOMDP, y::Float64)
return the index of y in the Y grid
"""
function get_y_index(pomdp::SingleOCPOMDP, y::Float64)
    y_0 = -pomdp.env.params.crosswalk_length/4
    y_ind = Int(ceil((y - y_0)/pomdp.pos_res)) + 1
    return y_ind
end

"""
    get_v_ped_index(pomdp::SingleOCPOMDP, v_ped::Float64)
return the index of v_ped in the V_ped grid
"""
function get_v_ped_index(pomdp::SingleOCPOMDP, v_ped::Float64)
    v_ped_ind = Int(ceil(v_ped/pomdp.vel_res)) + 1
    return v_ped_ind
end
