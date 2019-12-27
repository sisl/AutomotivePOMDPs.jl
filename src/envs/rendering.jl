AutoViz._colortheme["CROSSWALK"] = colorant"white"
  
# additional method to render the crosswalk environment with obstacles
function AutoViz.add_renderable!(rendermodel::RenderModel, env::CrosswalkEnv)
    roadway = gen_straight_roadway(2, env.params.roadway_length)
    AutoViz.add_renderable!(rendermodel, roadway)

    curve = env.crosswalk.curve
    n = length(curve)
    pts = Array{Float64}(undef, 2, n)
    for (i,pt) in enumerate(curve)
        pts[1,i] = pt.pos.x
        pts[2,i] = pt.pos.y
    end

    add_instruction!(rendermodel, render_dashed_line, (pts, AutoViz._colortheme["CROSSWALK"], env.crosswalk.width, 0.7, 0.5, 0.0, Cairo.CAIRO_LINE_CAP_BUTT))
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


function AutoViz.add_renderable!(rendermodel::RenderModel, env::UrbanEnv)
    # regular roadway
    roadway = Roadway(env.roadway.segments[1:end-length(env.crosswalks)])
    AutoViz.add_renderable!(rendermodel, roadway)

    for (i, cw) in enumerate(env.crosswalks)
        # crosswalk
        curve = cw.curve
        n = length(curve)
        pts = Array{Float64}(undef, 2, n)
        for (i,pt) in enumerate(curve)
            pts[1,i] = pt.pos.x
            pts[2,i] = pt.pos.y
        end
        add_instruction!(rendermodel, render_dashed_line, (pts, AutoViz._colortheme["CROSSWALK"], cw.width, 0.7, 0.5, 0.0, Cairo.CAIRO_LINE_CAP_BUTT))
    end
    # obstacles
    for obs in env.obstacles
        pts = Array{Float64}(undef, 2, obs.npts)
        for (i, pt) in enumerate(obs.pts)
            pts[1,i] = pt.x
            pts[2,i] = pt.y
        end
        ox, oy = get_center(obs)
        ow = get_width(obs)
        oh = get_height(obs)
        add_instruction!(rendermodel, render_fancy_obs, (ox, oy, pi, ow, oh))
        # add_instruction!(rendermodel, render_fill_region, (pts, AutoViz._colortheme["OBSTACLES"]))
    end

    # render stop line
    stop_line = posg(Frenet(env.roadway[LaneTag(6,1)], env.params.stop_line), env.roadway)
    x_pos, y_pos = stop_line.x, stop_line.y
    stop_pts = zeros(2,2)
    stop_pts[1,:] =  [(x_pos - env.params.lane_width/2) , (x_pos + env.params.lane_width/2)]
    stop_pts[2,:] =  [y_pos, y_pos]
    add_instruction!(rendermodel, render_line, (stop_pts, AutoViz._colortheme["CROSSWALK"], 1.0, Cairo.CAIRO_LINE_CAP_BUTT))

    return rendermodel
end

const OBSFILE = joinpath(@__DIR__, "..", "..", "icons", "building_topview.svg")
const OBSDATA = parse_file(OBSFILE)

function render_fancy_obs(
    ctx           :: Cairo.CairoContext,
    x             :: Real, # x-pos of the center of the obstacle
    y             :: Real, # y-pos of the center of the obstacle
    yaw           :: Real, # heading angle [rad]
    length        :: Real, #  length
    width         :: Real #  width
    )

    if length < width
        l = width
        w = length
        th = yaw - pi/2
    else
        l = length
        w = width
        th = yaw
    end
    Cairo.save(ctx)
    obsdata = parse_file(OBSFILE)

    r = Rsvg.handle_new_from_data(string(obsdata))

    
    d = Rsvg.handle_get_dimensions(r)
    # scaling factor
    xdir, ydir = length/d.width, width/d.height

    Cairo.translate(ctx, x, y)
    Cairo.scale(ctx, xdir, ydir)
    Cairo.rotate(ctx, th)
    Cairo.translate(ctx, -d.width/2, -d.height/2)

    Rsvg.handle_render_cairo(ctx, r)

    Cairo.restore(ctx)
end
