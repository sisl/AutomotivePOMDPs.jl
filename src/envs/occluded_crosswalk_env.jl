### CONFIGURATION OF THE OCCLUDED CROSSWALK ENVIRONMENT

""""
Container type for the environment parameters
"""
mutable struct CrosswalkParams
    # geometry
    n_lanes::Int64
    roadway_length::Float64
    lane_width::Float64
    crosswalk_length::Float64
    crosswalk_width::Float64
    # cars
    start_pos::Float64
    end_pos::Float64
    speed_limit::Float64

    # pedestrians
    n_max_ped::Int64
    ped_rate::Float64
    ped_max_speed::Float64
    ped_max_y::Float64
end

function CrosswalkParams(;n_lanes::Int64 = 2,
                    roadway_length::Float64 = 50.,
                    lane_width::Float64 = 3.,
                    crosswalk_length::Float64 = 20.,
                    crosswalk_width::Float64 = 6.0,
                    start_pos::Float64 = 5.0,
                    end_pos::Float64 = roadway_length/2 + 2*crosswalk_width,
                    speed_limit::Float64 = 8.,
                    n_max_ped::Int64 = 100,
                    ped_rate::Float64 = 0.5,
                    ped_max_speed::Float64 = 2.,
                    ped_max_y::Float64 = crosswalk_length/2)
    return CrosswalkParams(n_lanes, roadway_length, lane_width, crosswalk_length, crosswalk_width,
                     start_pos,end_pos, speed_limit, n_max_ped, ped_rate, ped_max_speed, ped_max_y)
end

"""
Define the crosswalk environment
"""
mutable struct CrosswalkEnv <: OccludedEnv
    roadway::Roadway
    crosswalk::Lane
    obstacles::Vector{ConvexPolygon}
    params::CrosswalkParams
end

"""
    CrosswalkEnv(params::CrosswalkParams)
constructor for the crosswalk environment
"""
function CrosswalkEnv(params::CrosswalkParams = CrosswalkParams())
    roadway = gen_straight_roadway(2, params.roadway_length)
    crosswalk_pos = params.roadway_length/2
    n_samples = 2 # for curve generation
    crosswalk = Lane(LaneTag(2,1), gen_straight_curve(VecE2(crosswalk_pos, -params.crosswalk_length/2),
                                                      VecE2(crosswalk_pos, params.crosswalk_length/2),
                                                       n_samples), width = params.crosswalk_width)
    cw_segment = RoadSegment(2, [crosswalk])
    push!(roadway.segments, cw_segment)
    obstacles = ConvexPolygon(4)
    #TODO parameterize the obstacle
    obstacles.pts = [VecE2(15, -1.5), VecE2(15, -4.5), VecE2(21.5, -4.5), VecE2(21.5, -1.5)]
    obstacles.npts = 4; # a rectangle
    env = CrosswalkEnv(roadway, crosswalk, [obstacles], params)
    return env
end
