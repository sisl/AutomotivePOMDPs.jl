### Multi-Lane intersection with fixed obstacles

"""
Container type for the environment parameters
"""
@with_kw mutable struct TInterParams
    # geometry
    x_min::Float64 = -20.0
    x_max::Float64 = 20.0
    y_min::Float64 = -20.0
    inter_x::Float64 = 0.
    inter_y::Float64 = 0.
    inter_width::Float64 = 6.0
    inter_height::Float64 = 1.0
    lane_width::Float64 = 3.0
    nlanes_main::Int64 = 2
    nlanes::Int64 = 1

    stop_line::Float64 = 10. # in m along the ego car lane
    # cars
    speed_limit::Float64 = 10.0 # in m/s
    car_rate::Float64 = 0.3 # probability of a car appearing every time steps
end

"""
Generate a T intersection given a geometry specify by TInterParams
"""
function gen_T_roadway(params::TInterParams)
    #=
    ________________________________________________________
        <---- seg 2     main street <----- seg 1
        -----> seg 3                 -----> seg 4
    _________________            ___________________________
                    | seg . seg  |
                    |  5  .  6   |
                    |     .      |
                    |     .      |
                    |     .      |


    =#
    roadway = Roadway()
    # aliases
    x_min = params.x_min
    x_max = params.x_max
    y_min = params.y_min
    inter_x = params.inter_x
    inter_y = params.inter_y
    inter_width = params.inter_width
    inter_height = params.inter_height
    lane_width = params.lane_width
    nlanes_main = params.nlanes_main
    nlanes = params.nlanes

    seg1 = VecSE2(x_max, inter_y+nlanes_main*lane_width - lane_width/2, pi)
    length1 = x_max - inter_x - inter_width
    add_line!(seg1, nlanes_main, length1, roadway)

    seg2 = VecSE2(inter_x - inter_width, inter_y+nlanes_main*lane_width - lane_width/2, pi)
    length2 =  inter_x - inter_width - x_min
    add_line!(seg2, nlanes_main, length2, roadway)

    seg3 = VecSE2(x_min, inter_y-nlanes_main*lane_width + lane_width/2, 0.0)
    length3 = inter_x - x_min - inter_width
    add_line!(seg3, nlanes_main, length3, roadway)


    seg4 = VecSE2(inter_x + inter_width, inter_y-nlanes_main*lane_width + lane_width/2, 0.0)
    length4 = x_max - inter_x - inter_width
    add_line!(seg4, nlanes_main, length4, roadway)

    seg5 = VecSE2(inter_x-nlanes*lane_width + lane_width/2, inter_y - nlanes_main*lane_width - inter_height, -pi/2)
    length5 = inter_y - nlanes_main*lane_width - y_min - inter_height
    add_line!(seg5, nlanes, length5, roadway)

    seg6 = VecSE2(inter_x+nlanes*lane_width - lane_width/2, y_min, pi/2)
    length6 = inter_y - nlanes_main*lane_width - y_min - inter_height
    add_line!(seg6, nlanes, length6, roadway)

    connections = [Connection(1,2), Connection(3,4)]
    for i=1:nlanes_main
        push!(connections, Connection(1, 5, 0, [(i, 1)]))
        push!(connections, Connection(3, 5, 0, [(i, 1)]))
        push!(connections, Connection(6, 2, 0, [(1, i)]))
        push!(connections, Connection(6, 4, 0, [(1, i)]))
    end

    add_junction!(Junction(connections), roadway)
    return roadway
end


function build_obstacles(params::TInterParams)
    return
end

"""
T intersection environment
"""
mutable struct IntersectionEnv <: OccludedEnv
    roadway::Roadway
    obstacles::Vector{ConvexPolygon}
    params::TInterParams
    lane_map::Dict{String, Lane}
end


function IntersectionEnv(params::TInterParams = TInterParams())
    roadway = gen_T_roadway(params)
    lane_map = Dict{String, Lane}()
    obstalces = Vector{ConvexPolygon}[]
    return IntersectionEnv(roadway, Vector{ConvexPolygon}[], params, lane_map)
end
