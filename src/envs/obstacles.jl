#### Obstacle generation ####

function empty_obstacles!(env::OccludedEnv)
    env.obstacles = Vector{ConvexPolygon}[]
end

function add_obstacle!(env::OccludedEnv, pos::VecSE2, width::Float64, height::Float64)
    obstacle = get_oriented_bounding_box(pos, width, height)
    push!(env.obstacles, obstacle)
end

function left_obstacle_space(env::OccludedEnv; y_min=env.params.y_min/4)
    x_min = env.params.x_min
    x_max = env.params.inter_x - env.params.nlanes*env.params.lane_width - maximum(env.params.crosswalk_width/2)
    y_max = env.params.inter_y-env.params.nlanes_main*env.params.lane_width
    return x_min, x_max, y_min, y_max
end

function get_width(poly::ConvexPolygon)
    width_seg = get_edge(poly, 1)
    width = dist(width_seg.B, width_seg.A)
    return width
end

function get_height(poly::ConvexPolygon)
    height_seg = get_edge(poly, 2)
    height = dist(height_seg.B, height_seg.A)
    return height
end

function right_obstacle_space(env::UrbanEnv; delta::Float64=1.0, y_min=env.params.y_min/4)
    x_min = env.params.inter_x + env.params.nlanes*env.params.lane_width + maximum(env.params.crosswalk_width/2)
    x_max = env.params.x_max
    y_max = env.params.inter_y-env.params.nlanes_main*env.params.lane_width
    return x_min, x_max, y_min, y_max
end

function upper_obstacle_space(env::UrbanEnv; max_width = 2.0)
    # x_min = env.params.x_min
    x_min = minimum([pos.x for pos in env.params.crosswalk_pos]) + maximum(env.params.crosswalk_width/2)
    # x_max = env.params.crosswalk_pos.x - env.params.crosswalk_width/2
    x_max = maximum([pos.x for pos in env.params.crosswalk_pos]) - maximum(env.params.crosswalk_width/2)
    y_min = env.params.inter_y + env.params.nlanes_main*env.params.lane_width
    y_max = y_min + max_width
    return x_min, x_max, y_min, y_max
end

function max_left_obstacle(env::UrbanEnv)
    x_min, x_max, y_min, y_max = left_obstacle_space(env)
    obstacle = get_oriented_bounding_box(VecSE2(0.5*(x_max + x_min), 0.5*(y_max + y_min)), x_max - x_min, y_max-y_min)
    return obstacle
end
function max_right_obstacle(env::UrbanEnv)
    x_min, x_max, y_min, y_max = right_obstacle_space(env)
    obstacle = get_oriented_bounding_box(VecSE2(0.5*(x_max + x_min), 0.5*(y_max + y_min)), x_max - x_min, y_max-y_min)
    return obstacle
end
function max_upper_obstacle(env::UrbanEnv)
    x_min, x_max, y_min, y_max = upper_obstacle_space(env)
    obstacle = get_oriented_bounding_box(VecSE2(0.5*(x_max + x_min), 0.5*(y_max + y_min)), x_max - x_min, y_max-y_min)
    return obstacle
end


struct LeftObsDist
    pos::VecSE2
    widths::Vector{Float64}
end
function LeftObsDist(env::UrbanEnv)
    xmin, xmax, ymin, ymax = left_obstacle_space(env)
    x_pos = 0.5*xmin + 0.5*xmax
    obs_pos = VecSE2(x_pos,0.5*ymax+0.5*ymin)
    max_width = xmax - xmin
    widths = [max_width, max_width/2, max_width/4, 3*max_width/4]
    return LeftObsDist(obs_pos, widths)
end
struct RightObsDist
    pos::VecSE2
    widths::Vector{Float64}
end
function RightObsDist(env::UrbanEnv)
    xmin, xmax, ymin, ymax = right_obstacle_space(env)
    x_pos = 0.5*xmin + 0.5*xmax
    obs_pos = VecSE2(x_pos,0.5*ymax+0.5*ymin)
    max_width = xmax - xmin
    widths = [max_width, max_width/2, max_width/4, 3*max_width/4]
    return RightObsDist(obs_pos, widths)
end
struct UpperObsDist
    pos::VecSE2
    widths::Vector{Float64}
end
function UpperObsDist(env::UrbanEnv)
    xmin, xmax, ymin, ymax = upper_obstacle_space(env)
    x_pos = 0.5*xmin + 0.5*xmax
    obs_pos = VecSE2(x_pos,0.5*ymax+0.5*ymin)
    max_width = xmax - xmin
    widths = [max_width, max_width/2, max_width/4, 3*max_width/4]
    return UpperObsDist(obs_pos, widths)
end

"""
structure with obstacle distribution parameters
"""
struct ObstacleDistribution
    upper_obs_pres_prob::Float64
    left_obs_pres_prob::Float64
    right_obs_pres_prob::Float64
    upper_obs_dist::UpperObsDist
    left_obs_dist::LeftObsDist
    right_obs_dist::RightObsDist

    # obs_widths::Vector{Float64}
    # obs_upper::Vector{Float64}
    # obs_left::Vector{Float64}
    # obs_right::Vector{Float64}
end

function ObstacleDistribution(env::UrbanEnv ;
                              upper_obs_pres_prob::Float64 = 0.75,
                              left_obs_pres_prob::Float64 = 0.75,
                              right_obs_pres_prob::Float64 = 0.75)
    left_obs_dist = LeftObsDist(env)
    right_obs_dist = RightObsDist(env)
    upper_obs_dist = UpperObsDist(env)
    return ObstacleDistribution(upper_obs_pres_prob, left_obs_pres_prob, right_obs_pres_prob,
                                upper_obs_dist, left_obs_dist, right_obs_dist)
end

"""
sample obstacle configuration from obs_dist and add them to the environment
"""
function sample_obstacles!(env::UrbanEnv, obs_dist::ObstacleDistribution, rng::AbstractRNG)
    empty_obstacles!(env)
    # sample upper_obstacle
    upper_xmin, upper_xmax, upper_ymin, upper_ymax = upper_obstacle_space(env)
    if rand(rng) < obs_dist.upper_obs_pres_prob
        upper_pos = obs_dist.upper_obs_dist.pos
        upper_width = clamp(rand(rng, obs_dist.upper_obs_dist.widths), 0., 2*min(upper_pos.x - upper_xmin, upper_xmax - upper_pos.x))
        # upper_width = rand(rng, obs_dist.upper_obs_dist.widths)
        add_obstacle!(env, upper_pos , upper_width , upper_ymax - upper_ymin)
    end

    # sample left_obstacle
    left_xmin, left_xmax, left_ymin, left_ymax = left_obstacle_space(env)
    if rand(rng) < obs_dist.left_obs_pres_prob
        left_pos = obs_dist.left_obs_dist.pos
        left_width = clamp(rand(rng, obs_dist.left_obs_dist.widths), 0., 2*min(left_pos.x - left_xmin, left_xmax - left_pos.x))
        add_obstacle!(env, left_pos, left_width , left_ymax - left_ymin)
    end

    # sample right_obstacle
    right_xmin, right_xmax, right_ymin, right_ymax = right_obstacle_space(env)
    if rand(rng) < obs_dist.right_obs_pres_prob
        right_pos = obs_dist.right_obs_dist.pos
        # right_width = rand(rng, obs_dist.right_obs_dist.widths)
        right_width = clamp(rand(rng, obs_dist.right_obs_dist.widths), 0., 2*min(right_pos.x - right_xmin, right_xmax - right_pos.x))
        add_obstacle!(env, right_pos , right_width , right_ymax - right_ymin)
    end
end

"""
sample one obstacle from obs_dist and add it to the environment
"""
function sample_obstacle!(env::UrbanEnv, obs_dist::ObstacleDistribution, rng::AbstractRNG)
    # choose from distribution 
    dist_ind = rand(1:3)
    presence = rand(rng)
    if dist_ind == 1 && presence < obs_dist.left_obs_pres_prob
        left_xmin, left_xmax, left_ymin, left_ymax = left_obstacle_space(env)
        left_pos = obs_dist.left_obs_dist.pos
        left_width = clamp(rand(rng, obs_dist.left_obs_dist.widths), 0., 2*min(left_pos.x - left_xmin, left_xmax - left_pos.x))
        add_obstacle!(env, left_pos, left_width , left_ymax - left_ymin)
    elseif dist_ind == 2  && presence < obs_dist.upper_obs_pres_prob
        upper_xmin, upper_xmax, upper_ymin, upper_ymax = upper_obstacle_space(env)
        upper_pos = obs_dist.upper_obs_dist.pos
        upper_width = clamp(rand(rng, obs_dist.upper_obs_dist.widths), 0., 2*min(upper_pos.x - upper_xmin, upper_xmax - upper_pos.x))
        # upper_width = rand(rng, obs_dist.upper_obs_dist.widths)
        add_obstacle!(env, upper_pos , upper_width , upper_ymax - upper_ymin)
    elseif dist_ind == 3 && presence < obs_dist.right_obs_pres_prob
        right_xmin, right_xmax, right_ymin, right_ymax = right_obstacle_space(env)
        right_pos = obs_dist.right_obs_dist.pos
        # right_width = rand(rng, obs_dist.right_obs_dist.widths)
        right_width = clamp(rand(rng, obs_dist.right_obs_dist.widths), 0., 2*min(right_pos.x - right_xmin, right_xmax - right_pos.x))
        add_obstacle!(env, right_pos , right_width , right_ymax - right_ymin)
    end      
end
