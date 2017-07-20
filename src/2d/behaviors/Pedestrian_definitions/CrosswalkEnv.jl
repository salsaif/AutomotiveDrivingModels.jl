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
    # add observation holder
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
