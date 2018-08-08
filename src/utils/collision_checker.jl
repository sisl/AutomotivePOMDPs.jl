
function collision_checker(veh_a::VehicleState, veh_b::VehicleState, veh_a_def::VehicleDef, veh_b_def::VehicleDef)
    center_a = veh_a.posG
    center_b = veh_b.posG
    # first fast check:
    Δ² = (veh_a.posG.x - veh_b.posG.x)^2 + (veh_a.posG.y - veh_b.posG.y)^2
    r_a = sqrt(veh_a_def.length*veh_a_def.length/4 + veh_a_def.width*veh_a_def.width/4)
    r_b = sqrt(veh_b_def.length*veh_b_def.length/4 + veh_b_def.width*veh_b_def.width/4)
    if Δ² ≤ r_a*r_a + 2*r_a*r_b + r_b*r_b
        # fast check is true, run parallel axis theorem
        Pa = polygon(center_a, veh_a_def)
        Pb = polygon(center_b, veh_b_def)
        return overlap(Pa, Pb)
    end
    return false
end

function polygon(pos::VecSE2{Float64}, veh_def::VehicleDef)
    x, y ,θ = pos 
    return polygon(x, y, θ, veh_def)
end

function polygon(x::Float64,y::Float64,theta::Float64,veh_def::VehicleDef)
    """ take the position of the car and the orientation and return the
    verteces of the safety polygon
     car located at x,y with orientation theta
    the safety margin around the car are lsafe,wsafe"""

    # aliases
    lcar  = veh_def.length
    wcar  = veh_def.width

    l_ = lcar/2
    w_ = wcar/2

    # compute each verteces:
    P = zeros(4,2)
    # A, B, C, D = zeros(2), zeros(2), zeros(2), zeros(2)
    # top left
    P[1,1] = x + l_*cos(theta) - w_*sin(theta)
    P[1,2] = y + l_*sin(theta) + w_*cos(theta)

    # top right
    P[2,1] = x + l_*cos(theta) + w_*sin(theta)
    P[2,2] = y + l_*sin(theta) - w_*cos(theta)

    # bottom right
    P[3,1] = x - l_*cos(theta) + w_*sin(theta)
    P[3,2] = y - l_*sin(theta) - w_*cos(theta)

    # bottom left
    P[4,1] = x - l_*cos(theta) - w_*sin(theta)
    P[4,2] = y - l_*sin(theta) + w_*cos(theta)

    return P
end

### POLYGON INTERSECTION USING PARALLEL AXIS THEOREM
# polygon a ,
# polygon described as a nx2 matrix

function overlap(poly_a::Array{Float64,2},
                poly_b::Array{Float64,2})
    """ Check if two convex polygons overlap
    a polygon is a nx2 matrix where n in the number of verteces
        http://gamemath.com/2011/09/detecting-whether-two-convex-polygons-overlap/ """

    if find_separating_axis(poly_a, poly_b)
        return false
    end
    if find_separating_axis(poly_b, poly_a)
        return false
    end

    return true

end

function find_separating_axis(poly_a::Array{Float64,2},
                              poly_b::Array{Float64,2})
    """ build a list of candidate separating axes from the edges of a
        /!\ edges needs to be ordered"""

    n_a = size(poly_a)[1]
    n_b = size(poly_b)[1]
    axis = zeros(2)
    previous_vertex = poly_a[end,:]
    for i=1:n_a # loop through all the edges n edges
        current_vertex = poly_a[i,:]
        # get edge vector
        edge = current_vertex - previous_vertex

        # rotate 90 degrees to get the separating axis
        axis[1] = edge[2]
        axis[2] = -edge[1]

        #  project polygons onto the axis
        a_min,a_max = polygon_projection(poly_a, axis)
        b_min,b_max = polygon_projection(poly_b, axis)

        # check separation
        if a_max < b_min
            return true#,current_vertex,previous_vertex,edge,a_max,b_min

        end
        if b_max < a_min
            return true#,current_vertex,previous_vertex,edge,a_max,b_min

        end

        previous_vertex = poly_a[i,:]
    end

    # no separation was found
    return false
end

function polygon_projection(poly::Array{Float64,2},
                            axis::Vector{Float64})
        """ return the projection interval for the polygon poly over the axis axis """

        n_a = size(poly)[1]
        d1 = dot(poly[1,:],axis)
        d2 = dot(poly[2,:],axis)
        # initialize min and max
        if d1<d2
            out_min = d1
            out_max = d2
        else
            out_min = d2
            out_max = d1
        end

        for i=1:n_a
            d = dot(poly[i,:],axis)

            if d < out_min
                out_min = d
            elseif d > out_max
                out_max = d
            end
        end
        return out_min,out_max
end
