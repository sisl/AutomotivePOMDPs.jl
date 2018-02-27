@with_kw mutable struct UrbanDriver <: DriverModel{LonAccelDirection}
    a::LonAccelDirection = LonAccelDirection(0., 1)
    navigator::RouteFollowingIDM = RouteFollowingIDM()
    intersection_driver::IntersectionDriver = IntersectionDriver()
    crosswalk_driver::CrosswalkDriver = CrosswalkDriver()


    debug::Bool = false
end

function AutomotiveDrivingModels.reset_hidden_state!(model::UrbanDriver)
    reset_hidden_state!(model.navigator)
    reset_hidden_state!(model.intersection_driver)
    reset_hidden_state!(model.crosswalk_driver)
    model
end

function AutomotiveDrivingModels.observe!(model::UrbanDriver, scene::Scene, roadway::Roadway, egoid::Int)
    AutomotiveDrivingModels.observe!(model.navigator, scene, roadway, egoid)
    AutomotiveDrivingModels.observe!(model.intersection_driver, scene, roadway, egoid)
    AutomotiveDrivingModels.observe!(model.crosswalk_driver, scene, roadway, egoid)
    # println("intersection driver ", model.intersection_driver.a)
    # println("crosswalk driver ", model.crosswalk_driver.a)
    # take the more conservative of the three actions
    a_lon = min(model.intersection_driver.a.a_lon, model.crosswalk_driver.a.a_lon)
    if model.debug
        println("inter ", model.intersection_driver.a.a_lon)
        println("crosswalk ", model.crosswalk_driver.a.a_lon)
    end
    model.a = LonAccelDirection(a_lon, model.navigator.dir)
    return model
end

get_name(::UrbanDriver) = "UrbanDriver"
Base.rand(model::UrbanDriver) = model.a
