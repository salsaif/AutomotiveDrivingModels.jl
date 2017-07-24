type SimpleSensor
    pos_noise::Float64
    vel_noise::Float64
end
# add observation structure
# type Observation
#
#
# end
type CrosswalkEnv
    roadway::Roadway
    crosswalk::Lane
    sensormodel::SimpleSensor
    Observation::Array{Any,1}
end



function measure(ego::VehicleState, scene::Scene, model::SimpleSensor, env::CrosswalkEnv)
    observed = Vehicle[] # sizehint!(Vector{Vehicle(0)}, 5)
    for i in 2:scene.n
        veh = scene[i]
        car = veh.state
#         if is_observable(car, ego, cars, env) # needed if obstacles exist
        obs_state = VehicleState(VecSE2(car.posG.x + model.pos_noise*randn(),
                                                  car.posG.y + model.pos_noise*randn(),
                                                  car.posG.θ),
                                                  env.roadway,
                                                   car.v + model.vel_noise*randn())
        push!(observed, Vehicle(obs_state, veh.def, veh.id))
#         end
    end
    return observed
end
function observe!(driver::DriverModel{LatLonAccel}, scene::Scene, env::CrosswalkEnv, egoid::Int)

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
        measured = measure(scene[egoid].state, scene, env.sensormodel, env)
        tracker!(env,measured)
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

function propagate{D<:Union{VehicleDef, BicycleModel}}(veh::Entity{VehicleState, D, Int}, action::LatLonAccel, env::CrosswalkEnv, ΔT::Float64)
    probagate(veh,action,env.roadway,ΔT)
end

function get_actions!{S,D,I,A,M<:DriverModel}(
    actions::Vector{A},
    scene::EntityFrame{S,D,I},
    env::CrosswalkEnv,
    models::Dict{I, M}, # id → model
    )

    for (i,veh) in enumerate(scene)
        model = models[veh.id]
        observe!(model, scene, env, veh.id)
        actions[i] = rand(model)
    end

    actions
end

  function tick!{S,D,I,A}(
    scene::EntityFrame{S,D,I},
    env::CrosswalkEnv,
    actions::Vector{A},
    Δt::Float64,
    )
    tick!(scene, env.roadway, actions, ΔT)
  end

  function simulate!{S,D,I,A,M<:DriverModel}(
    ::Type{A},
    rec::EntityQueueRecord{S,D,I},
    scene::EntityFrame{S,D,I},
    env::CrosswalkEnv,
    models::Dict{I,M},
    nticks::Int,
    )

    empty!(rec)
    update!(rec, scene)
    actions = Array{A}(length(scene))

    for tick in 1 : nticks
        get_actions!(actions, scene, env, models)
        tick!(scene, env, actions, rec.timestep)
        update!(rec, scene)
    end

    return rec
end
