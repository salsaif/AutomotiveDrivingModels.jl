mutable struct Tim2DDriver <: DriverModel{LatLonAccel}
    rec::SceneRecord
    mlon::LaneFollowingDriver
    mlat::LateralDriverModel
    mlane::LaneChangeModel

    function Tim2DDriver(
        timestep::Float64;
        mlon::LaneFollowingDriver=IntelligentDriverModel(),
        mlat::LateralDriverModel=ProportionalLaneTracker(),
        mlane::LaneChangeModel=TimLaneChanger(timestep),
        rec::SceneRecord = SceneRecord(1, timestep)
        )

        retval = new()

        retval.rec = rec
        retval.mlon = mlon
        retval.mlat = mlat
        retval.mlane = mlane

        retval
    end
end
get_name(::Tim2DDriver) = "Tim2DDriver"
function set_desired_speed!(model::Tim2DDriver, v_des::Float64)
    set_desired_speed!(model.mlon, v_des)
    set_desired_speed!(model.mlane, v_des)
    model
end
function track_longitudinal!(driver::LaneFollowingDriver, scene::Scene, roadway::Roadway, vehicle_index::Int, fore::NeighborLongitudinalResult)
    v_ego = scene[vehicle_index].state.v
    if fore.ind != 0
        headway, v_oth = fore.Δs, scene[fore.ind].state.v
    else
        headway, v_oth = NaN, NaN
    end
    return track_longitudinal!(driver, v_ego, v_oth, headway)
end
function observe!(driver::Tim2DDriver, scene::Scene, env::CrosswalkEnv, egoid::Int)

    AutomotiveDrivingModels.update!(driver.rec, scene)
    AutomotiveDrivingModels.observe!(driver.mlane, scene, env.roadway, egoid)
    vehicle_index = findfirst(scene, egoid)
    lane_change_action = rand(driver.mlane)
    laneoffset = get_lane_offset(lane_change_action, driver.rec, env.roadway, vehicle_index)
    lateral_speed = convert(Float64, get(VELFT, driver.rec, env.roadway, vehicle_index))

    if lane_change_action.dir == DIR_MIDDLE
        fore = get_neighbor_fore_along_lane(scene, vehicle_index, env.roadway, VehicleTargetPointFront(), VehicleTargetPointRear(), VehicleTargetPointFront())
    elseif lane_change_action.dir == DIR_LEFT
        fore = get_neighbor_fore_along_left_lane(scene, vehicle_index, env.roadway, VehicleTargetPointFront(), VehicleTargetPointRear(), VehicleTargetPointFront())
    else
        @assert(lane_change_action.dir == DIR_RIGHT)
        fore = get_neighbor_fore_along_right_lane(scene, vehicle_index, env.roadway, VehicleTargetPointFront(), VehicleTargetPointRear(), VehicleTargetPointFront())
    end

    AutomotiveDrivingModels.track_lateral!(driver.mlat, laneoffset, lateral_speed)
#     AutomotiveDrivingModels.track_longitudinal!(driver.mlon, scene, env.roadway, vehicle_index, fore)

    if scene[egoid].state.posG.x < 0          ### If the vehicle has not yet crossed the crosswalk
        observed = measure(scene[egoid].state, scene, env.sensormodel, env)
        v_oth = NaN
        for ped in observed
            loc = ped.state.posG
            if loc.x > -0.5*DEFAULT_LANE_WIDTH && loc.x < 1.5*DEFAULT_LANE_WIDTH && abs(loc.y) < 1
                v_oth = 0.0
                break
            end
        end
        v_ego = scene[egoid].state.v

        AutomotiveDrivingModels.track_longitudinal!(driver.mlon, v_ego, v_oth, 0.05)
    end

    driver
end
Base.rand(driver::Tim2DDriver) = LatLonAccel(rand(driver.mlat), rand(driver.mlon).a)
Distributions.pdf(driver::Tim2DDriver, a::LatLonAccel) = pdf(driver.mlat, a.a_lat) * pdf(driver.mlon, a.a_lon)
Distributions.logpdf(driver::Tim2DDriver, a::LatLonAccel) = logpdf(driver.mlat, a.a_lat) * logpdf(driver.mlon, a.a_lon)
