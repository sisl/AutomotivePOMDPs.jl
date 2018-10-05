# additional method to render the crosswalk environment with obstacles
function AutoViz.render!(rendermodel::RenderModel, env::CrosswalkEnv)
    roadway = gen_straight_roadway(2, env.params.roadway_length)
    AutoViz.render!(rendermodel, roadway)

    curve = env.crosswalk.curve
    n = length(curve)
    pts = Array{Float64}(undef, 2, n)
    for (i,pt) in enumerate(curve)
        pts[1,i] = pt.pos.x
        pts[2,i] = pt.pos.y
    end

    add_instruction!(rendermodel, render_dashed_line, (pts, colorant"white", env.crosswalk.width, 1.0, 1.0, 0.0, Cairo.CAIRO_LINE_CAP_BUTT))
    for obs in env.obstacles
        pts = Array{Float64}(undef, 2, obs.npts)
        for (i, pt) in enumerate(obs.pts)
            pts[1,i] = pt.x
            pts[2,i] = pt.y
        end

        add_instruction!(rendermodel, render_fill_region, (pts, colorant"gray"))
    end

    return rendermodel
end


function AutoViz.render!(rendermodel::RenderModel, env::UrbanEnv)
    # regular roadway
    roadway = Roadway(env.roadway.segments[1:end-length(env.crosswalks)])
    AutoViz.render!(rendermodel, roadway)

    for (i, cw) in enumerate(env.crosswalks)
        # crosswalk
        curve = cw.curve
        n = length(curve)
        pts = Array{Float64}(undef, 2, n)
        for (i,pt) in enumerate(curve)
            pts[1,i] = pt.pos.x
            pts[2,i] = pt.pos.y
        end
        add_instruction!(rendermodel, render_dashed_line, (pts, colorant"white", cw.width, 0.5, 0.7, 0.0, Cairo.CAIRO_LINE_CAP_BUTT))
    end
    # obstacles
    for obs in env.obstacles
        pts = Array{Float64}(undef, 2, obs.npts)
        for (i, pt) in enumerate(obs.pts)
            pts[1,i] = pt.x
            pts[2,i] = pt.y
        end

        add_instruction!(rendermodel, render_fill_region, (pts, colorant"gray"))
    end

    # render stop line
    stop_line = get_posG(Frenet(env.roadway[LaneTag(6,1)], env.params.stop_line), env.roadway)
    x_pos, y_pos = stop_line.x, stop_line.y
    stop_pts = zeros(2,2)
    stop_pts[1,:] =  [(x_pos - env.params.lane_width/2) , (x_pos + env.params.lane_width/2)]
    stop_pts[2,:] =  [y_pos, y_pos]
    add_instruction!(rendermodel, render_line, (stop_pts, colorant"white", 1.0, Cairo.CAIRO_LINE_CAP_BUTT))

    return rendermodel
end
