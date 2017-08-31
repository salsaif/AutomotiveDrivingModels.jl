using Vec
type SimpleSensor
    pos_noise::Float64
    vel_noise::Float64
    likelihood::Float64
end

type CrosswalkEnv
    roadway::Roadway
    crosswalk::Lane
    sensormodel::SimpleSensor
    observed::Array{Any,1}
end



function measure(ego::VehicleState, scene::Scene, model::SimpleSensor, env::CrosswalkEnv)
    measured = Vehicle[] # sizehint!(Vector{Vehicle(0)}, 5)
    model.likelihood = 0
    for i in 2:scene.n
        veh = scene[i]
        car = veh.state
        rand1 = randn()
        rand2 = randn()
        rand3 = randn()
        rand4 = randn()
        model.likelihood = (rand1^2+rand2^2+rand3^2)/model.pos_noise + rand4^2/model.vel_noise
#         if is_observable(car, ego, cars, env) # needed if obstacles exist
        vx = car.v*cos(car.posG.θ) + model.pos_noise*rand3
        vy = car.v*sin(car.posG.θ) + model.vel_noise*rand4
        obs_state = VehicleState(VecSE2(car.posG.x + model.pos_noise*rand1,
                                                  car.posG.y + model.pos_noise*rand2,
                                                  atan2(vy,vx)),
                                                  env.roadway,
                                                  sqrt(vx^2+vy^2))
        push!(measured, Vehicle(obs_state, veh.def, veh.id))
#         end
    end
    return measured
end
function observe!(driver::DriverModel{LatLonAccel}, scene::Scene, env::CrosswalkEnv, egoid::Int)
    update!(driver.rec, scene)
    observe!(driver.mlane, scene, env.roadway, egoid)
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

    track_lateral!(driver.mlat, laneoffset, lateral_speed)
    # track_longitudinal!(driver.mlon, scene, env.roadway, vehicle_index, fore)

    if scene[egoid].state.posG.x < 0.0          ### If the vehicle has not yet crossed the crosswalk
        measured = measure(scene[egoid].state, scene, env.sensormodel, env)
        tracker!(env,measured)
        v_oth = NaN
        min_dist = Inf
        headway = Inf
        for ped in env.observed
            loc = ped.state.posG
            if loc.y > -0.5*DEFAULT_LANE_WIDTH && loc.y < 1.5*DEFAULT_LANE_WIDTH
                v_oth = ped.state.v*cos(ped.state.posG.θ)
                distance = dist(convert(VecE2, scene[egoid].state.posG), convert(VecE2, ped.state.posG))
                if distance < min_dist
                    min_dist = distance
                    headway = ped.state.posG.x - scene[egoid].state.posG.x
                end
            end
        end
        v_ego = scene[egoid].state.v
        track_longitudinal!(driver.mlon, v_ego, v_oth, headway)
    else
        track_longitudinal!(driver.mlon, scene, env.roadway, vehicle_index, fore)
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
    tick!(scene, env.roadway, actions, Δt)
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
