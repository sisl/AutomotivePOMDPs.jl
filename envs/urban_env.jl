"""
Container type for the environment parameters
"""
@with_kw mutable struct UrbanParams
    # geometry
    x_min::Float64 = -30.0
    x_max::Float64 = 30.0
    y_min::Float64 = -30.0
    inter_x::Float64 = 0.
    inter_y::Float64 = 0.
    inter_width::Float64 = 6.0
    inter_height::Float64 = 1.0
    lane_width::Float64 = 3.0
    nlanes_main::Int64 = 2
    nlanes::Int64 = 1
    # crosswalk
    crosswalk_pos::Vector{VecSE2} = [VecSE2(6, 0., pi/2), VecSE2(-6, 0., pi/2), VecSE2(0., -8., 0.)]
    crosswalk_length::Vector{Float64}  = [20.0, 20., 10.0]
    crosswalk_width::Vector{Float64} = [4.0, 4.0, 3.1]

    stop_line::Float64 = 19. # in m along the ego car lane
    # cars
    speed_limit::Float64 = 10.0 # in m/s
    car_rate::Float64 = 0.3 # probability of a car appearing every time steps
    # peds
    ped_rate::Float64 = 0.3
    ped_max_speed::Float64 = 2.0

end


"""
Urban Environment with intersections and crosswalk
"""
mutable struct UrbanEnv <: OccludedEnv
    roadway::Roadway
    crosswalks::Vector{Lane}
    obstacles::Vector{ConvexPolygon}
    params::UrbanParams
    priorities::Dict{Tuple{LaneTag, LaneTag}, Bool}
    directions::Dict{Symbol, Tuple{LaneTag, LaneTag}}
end

function UrbanEnv(;params=UrbanParams())
    intersection_params = TInterParams(params.x_min, params.x_max, params.y_min, params.inter_x,
                                       params.inter_y, params.inter_width, params.inter_height,
                                       params. lane_width, params.nlanes_main, params.nlanes,
                                       params. stop_line, params.speed_limit, params.car_rate)
    roadway = gen_T_roadway(intersection_params)
    crosswalks = Lane[]
    cw_id = length(roadway.segments)+1
    for i =1:length(params.crosswalk_pos)
        cw_tag = LaneTag(cw_id, 1)

        crosswalk = Lane(cw_tag, gen_straight_curve(VecE2(params.crosswalk_pos[i]) - polar(params.crosswalk_length[i]/2, params.crosswalk_pos[i].θ),
                                                              VecE2(params.crosswalk_pos[i]) + polar(params.crosswalk_length[i]/2, params.crosswalk_pos[i].θ),
                                                               2), width = params.crosswalk_width[i])
        cw_segment = RoadSegment(length(roadway.segments)+1, [crosswalk])
        push!(crosswalks, crosswalk)
        push!(roadway.segments, cw_segment)
        cw_id += 1
    end
    obstacles = Vector{ConvexPolygon}[]
    priorities, directions = priority_map(roadway)
    return UrbanEnv(roadway, crosswalks, obstacles, params, priorities, directions)
end

#### Obstacle generation ####


function empty_obstacles!(env::UrbanEnv)
    env.obstacles = Vector{ConvexPolygon}[]
end

function add_obstacle!(env::UrbanEnv, pos::VecSE2, width::Float64, height::Float64)
    obstacle = get_oriented_bounding_box(pos, width, height)
    push!(env.obstacles, obstacle)
end

function left_obstacle_space(env::UrbanEnv; y_min=env.params.y_min/2)
    x_min = env.params.x_min
    x_max = env.params.inter_x - env.params.nlanes*env.params.lane_width
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

function right_obstacle_space(env::UrbanEnv; delta::Float64=1.0, y_min=env.params.y_min/2)
    x_min = env.params.inter_x + env.params.nlanes*env.params.lane_width
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



# function ObstacleDistribution(env::UrbanEnv ;
#                                upper_obs_pres_prob::Float64 = 0.75,
#                                left_obs_pres_prob::Float64 = 0.75,
#                                right_obs_pres_prob::Float64 = 0.75,
#                                obs_widths::Vector{Float64} = [5., 10., 17.])
#     # upper_xmin, upper_xmax, upper_ymin, upper_ymax = upper_obstacle_space(env)
#     # obs_upper = [upper_xmin + minimum(obs_widths)/2, 0.5*upper_xmin + 0.5*upper_xmax, upper_xmax - minimum(obs_widths)/2]
#
#     upper_xmin, upper_xmax, upper_ymin, upper_ymax = upper_obstacle_space(env)
#     obs_upper = [0.5*upper_xmin + 0.5*upper_xmax]
#
#
#     left_xmin, left_xmax, left_ymin, left_ymax = left_obstacle_space(env)
#     obs_left = [left_xmin + minimum(obs_widths)/2, 0.5*left_xmin + 0.5*left_xmax, left_xmax - minimum(obs_widths)/2]
#
#     right_xmin, right_xmax, right_ymin, right_ymax = right_obstacle_space(env)
#     obs_right = [right_xmin + minimum(obs_widths)/2, 0.5*right_xmin + 0.5*right_xmax, right_xmax - minimum(obs_widths)/2]
#     return ObstacleDistribution(upper_obs_pres_prob, left_obs_pres_prob, right_obs_pres_prob, obs_widths, obs_upper, obs_left, obs_right)
# end


# """
# sample obstacle configuration from obs_dist and add them to the environment
# """
# function sample_obstacles!(env::UrbanEnv, obs_dist::ObstacleDistribution, rng::AbstractRNG)
#     empty_obstacles!(env)
#     # sample upper_obstacle
#     upper_xmin, upper_xmax, upper_ymin, upper_ymax = upper_obstacle_space(env)
#     if rand(rng) < obs_dist.upper_obs_pres_prob
#         upper_pos = VecSE2(rand(rng, obs_dist.obs_upper),0.5*upper_ymax+0.5*upper_ymin)
#         upper_width = clamp(rand(rng, obs_dist.obs_widths), 0., 2*min(upper_pos.x - upper_xmin, upper_xmax - upper_pos.x))
#         add_obstacle!(env, upper_pos , upper_width , upper_ymax - upper_ymin)
#     end
#
#     # sample left_obstacle
#     left_xmin, left_xmax, left_ymin, left_ymax = left_obstacle_space(env)
#     if rand(rng) < obs_dist.left_obs_pres_prob
#         left_pos = VecSE2(rand(rng, obs_dist.obs_left),0.5*left_ymax+0.5*left_ymin)
#         left_width = clamp(rand(rng, obs_dist.obs_widths), 0., 2*min(left_pos.x - left_xmin, left_xmax - left_pos.x))
#         add_obstacle!(env, left_pos, left_width , left_ymax - left_ymin)
#     end
#
#     # sample right_obstacle
#     right_xmin, right_xmax, right_ymin, right_ymax = right_obstacle_space(env)
#     if rand(rng) < obs_dist.right_obs_pres_prob
#         right_pos = VecSE2(rand(rng, obs_dist.obs_right),0.5*right_ymax+0.5*right_ymin)
#         right_width = clamp(rand(rng, obs_dist.obs_widths), 0., 2*min(right_pos.x - right_xmin, right_xmax - right_pos.x))
#         add_obstacle!(env, right_pos , right_width , right_ymax - right_ymin)
#     end
# end
