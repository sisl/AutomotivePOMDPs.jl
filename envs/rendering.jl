# additional method to render the crosswalk environment with obstacles
function AutoViz.render!(rendermodel::RenderModel, env::CrosswalkEnv)
    roadway = gen_straight_roadway(2, env.params.roadway_length)
    render!(rendermodel, roadway)

    curve = env.crosswalk.curve
    n = length(curve)
    pts = Array{Float64}(2, n)
    for (i,pt) in enumerate(curve)
        pts[1,i] = pt.pos.x
        pts[2,i] = pt.pos.y
    end

    add_instruction!(rendermodel, render_dashed_line, (pts, colorant"white", env.crosswalk.width, 1.0, 1.0, 0.0, Cairo.CAIRO_LINE_CAP_BUTT))
    obs = env.obstacles[1]
    for obs in env.obstacles
        pts = Array{Float64}(2, obs.npts)
        for (i, pt) in enumerate(obs.pts)
            pts[1,i] = pt.x
            pts[2,i] = pt.y
        end

        add_instruction!(rendermodel, render_fill_region, (pts, colorant"gray"))
    end

    return rendermodel
end
