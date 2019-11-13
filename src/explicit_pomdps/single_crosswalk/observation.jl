### Observation model  ############################################################################

function POMDPs.observation(pomdp::SingleOCPOMDP, a::SingleOCAction, sp::SingleOCState)
    if !is_observable_fixed(sp, pomdp.env) || off_the_grid(pomdp, sp.ped)
        o = SingleOCObs(false, sp.ego, get_off_the_grid(pomdp))
        return Deterministic(o)
    elseif collision_checker(sp.ego, sp.ped, pomdp.ego_type, pomdp.ped_type)
        return Deterministic(sp)
    elseif isterminal(pomdp, sp)
        return Deterministic(sp)
    end
    ego = sp.ego
    ped = sp.ped

    neighbors = Vector{VehicleState}(undef, 9)
    neighbors[1] = yv_to_state(pomdp, ped.posG.y - pomdp.pos_res, ped.v)
    neighbors[2] = yv_to_state(pomdp, ped.posG.y + pomdp.pos_res, ped.v)
    neighbors[3] = yv_to_state(pomdp, ped.posG.y - pomdp.pos_res, ped.v - pomdp.vel_res)
    neighbors[4] = yv_to_state(pomdp, ped.posG.y + pomdp.pos_res, ped.v - pomdp.vel_res)
    neighbors[5] = yv_to_state(pomdp, ped.posG.y - pomdp.pos_res, ped.v + pomdp.vel_res)
    neighbors[6] = yv_to_state(pomdp, ped.posG.y + pomdp.pos_res, ped.v + pomdp.vel_res)
    neighbors[7] = yv_to_state(pomdp, ped.posG.y, ped.v - pomdp.vel_res)
    neighbors[8] = yv_to_state(pomdp, ped.posG.y, ped.v + pomdp.vel_res)
    neighbors[9] = yv_to_state(pomdp, ped.posG.y, ped.v)

    grid = RectangleGrid(get_Y_grid(pomdp), get_V_ped_grid(pomdp)) #XXX preallocate

    states = SingleOCObs[]
    sizehint!(states, 9)
    for neighbor in neighbors
        if in_bounds_ped(pomdp, neighbor) && !collision_checker(ego, neighbor, pomdp.ego_type, pomdp.ped_type)
            # make sure neighbor is in the grid
            neigh_yv_ind, neigh_yv_weights = interpolants(grid, [neighbor.posG.y, neighbor.v])
            neigh_yv = neigh_yv_ind[argmax(neigh_yv_weights)]
            neighbor = yv_to_state(pomdp, ind2x(grid, neigh_yv)...)
            push!(states, SingleOCObs(false, ego, neighbor))
        end
    end
    probs = zeros(length(states))
    for (i, o) in enumerate(states)
        probs[i] = obs_weight(o, sp, pomdp)
    end
    probs = normalize!(probs, 1)
    @assert length(probs) == length(states)
    return SparseCat(states, probs)
end

"""
    function obs_weight(o::SingleOCState, s::SingleOCState, pomdp::SingleOCPOMDP)
Given a continuous observation returns a weight proportional to the probability of observing o
while being in the state s
"""
function obs_weight(o::SingleOCState, s::SingleOCState, pomdp::SingleOCPOMDP)
    weight = 1.0
    if !is_observable_fixed(s.ego, s.ped, pomdp.env)
        if off_the_grid(pomdp, o.ped)
            return weight
        else
            return 0.
        end
    else
        if off_the_grid(pomdp, o.ped) || !is_observable_fixed(o.ego, o.ped, pomdp.env)
            return 0.
        else
            pos_noise = pomdp.pos_obs_noise
            vel_noise = pomdp.vel_obs_noise
            weight *= 1/(sqrt(2*pi*pos_noise*vel_noise))*exp(-1/pos_noise*(s.ped.posG.y - o.ped.posG.y)^2 - 1/vel_noise*(s.ped.v - o.ped.v)^2)
            return weight
        end
    end
end


"""
Check if a pedestrian in s is observable or not
"""
function is_observable_fixed(s::SingleOCState, env::CrosswalkEnv)
    m = length(env.obstacles)
    ped = s.ped
    ego = s.ego
    front = ego.posG + polar(VehicleDef().length/2, ego.posG.θ)
    angle = atan(ped.posG.y - front.y, ped.posG.x - front.x)
    ray = Projectile(VecSE2(front.x, front.y, angle), 1.0)
    if isinf(ped.v)
        return false
    end
    for i = 1:m
        if is_colliding(ray, env.obstacles[i], ped.posG)
            return false
        end
    end
    return true
end


"""
Check if a state is in bound
"""
function in_bounds_ped(pomdp::SingleOCPOMDP, veh::VehicleState)
    cl = pomdp.env.params.crosswalk_length
    y_min = -cl/4
    y_max = cl/4
    return (y_min <= veh.posG.y <= y_max) && (0. <= veh.v <= pomdp.env.params.ped_max_speed)
end

### Helpers for converting SingleOCObs to NN input

# """
#     extract_gray!(s::Array{UInt32, 2}, dest::Array{Float64, 2})
# Extract a gray scale image from a CairoSurface object
# """
# function extract_gray!(s::Array{UInt32, 2}, dest::Array{Float64, 2})
#     for (i,s) in enumerate(s)
#         dest[i] = 0.2989*convert(Float64, (s >> 16) & 0xFF) +
#                   0.5870*convert(Float64, (s >> 8) & 0xFF) +
#                   0.1140*convert(Float64, s & 0xFF)
#     end
#     return dest'
# end

# uncomment for image representation
# /!\ do not forget to change the observation space size in the python wrapper
# function POMDPs.convert_o(o::SingleOCObs, pomdp::SingleOCPOMDP)
#     scene = state_to_scene(pomdp, o)
#     img = AutoViz.render(scene, DQNFeatures(pomdp.env), [InflateOverlay()], cam=StaticCamera(VecE2(25, 0.), 20.))
#     gray_img = Array{Float64, 2}(Int(img.width), Int(img.height))
#     extract_gray!(img.data, gray_img)
#     gray_img = imresize(gray_img, (84,84))
#     gray_img /= 255.0
# end

# uncomment for engineered feature representation with SingleOCclusion map
# function POMDPs.convert_o(::Type{Array{Float64, 1}}, o::SingleOCObs, pomdp::SingleOCPOMDP)
#     X = get_X_grid(pomdp)
#     Y = get_Y_grid(pomdp)
#     im_w = length(X)
#     im_h = length(Y)
#     max_speed = get_V_grid(pomdp)[end]
#     features = zeros(im_w, im_h, 4)
#     for (i,x) in enumerate(X)
#         for (j,y) in enumerate(Y)
#             features[i,j,1] = is_colliding(x, y, o.ego) ? 1.0 : 0.0
#             features[i,j,2] = (is_colliding(x, y, o.ped) && is_observable(x, y, o.ego, pomdp.env)) ? 1.0 : 0.0
#             features[i,j,1] = is_observable(x, y, o.ego, pomdp.env) ? 1.0 : 0.0
#             if is_colliding(x, y, o.ego)
#                 features[i,j,2] = o.ego.v/max_speed
#             elseif is_colliding(x,  y, o.ped) && is_observable(x, y, o.ego, pomdp.env)
#                 features[i,j,2] = o.ped.v/max_speed
#             end
#         end
#     end
#     return features
# end

# uncomment for vector representation
# /!\ do not forget to change the observation space size in the python wrapper
function POMDPs.convert_o(::Type{Array{Float64, 1}}, o::SingleOCObs, pomdp::SingleOCPOMDP)
    o_vec = Vector{Float64}(4)
    o_vec[1], o_vec[2] = o.ego.posG.x/pomdp.x_goal, o.ego.v/pomdp.env.params.speed_limit
    o_vec[3] = o.ped.posG.y/pomdp.y_goal
    o_vec[4] = o.ped.v
    if isinf(o_vec[4])
        o_vec[4] = 0.
    end
    o_vec[4] /= 2.
    return reshape(o_vec, (4,1,1))
end
