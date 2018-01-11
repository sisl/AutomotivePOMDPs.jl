function get_true_roadway()
    roadway = Roadway()
    nlanes = 1
    seg1 = VecSE2(-10.0,0.0,0.0)
    length1 = 10.
    add_line!(seg1,nlanes,length1,roadway)

    seg2 = VecSE2(5.0,-5.0,-pi/2)
    length2 = 10.
    add_line!(seg2,nlanes,length2,roadway)

    seg3 = VecSE2(0.0, 3.0, pi)
    length3 = 10.
    add_line!(seg3, nlanes, length3, roadway)

    seg4 = VecSE2(23.0, 3.0, pi)
    length4 = 10.
    add_line!(seg4, nlanes, length4, roadway)

    seg5 = VecSE2(13.0, 0.0, 0.0)
    length5 = 10.
    add_line!(seg5, nlanes, length5, roadway)

    seg6 = VecSE2(8.0, -15, pi/2)
    length6 = 10.
    add_line!(seg6, nlanes, length6, roadway)

    junction = Junction([Connection(1,2), Connection(1,5), Connection(6,3), Connection(4,3), Connection(6,5), Connection(4,2)])  #, Connection(3,4), Connection(1,5), Connection(6,5), Connection(4,2)])

    add_junction!(junction, roadway)

    roadway
end

function true_scene(scene::Scene, roadway::Roadway)
    new_scene = Scene
    for veh in scene
        new_veh_state = VehicleState(veh.state.posG, roadway, veh.state.v)
        new_veh = Vehicle(new_veh_state, veh.def, veh.id)
        push!(new_scene, new_veh)
    end
    return new_scene
end

# doc, r = initialize_XML()
# convert_roadway!(r, roadway)
# write("test_intersection.xodr", doc)
