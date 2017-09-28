immutable LaneSpecificAccelLatLon
    a_lat::Float64
    a_lon::Float64
end

type CrosswalkDriver <: DriverModel{LaneSpecificAccelLatLon}
    a::LaneSpecificAccelLatLon
    σ::Array{Float64,1}
    function CrosswalkDriver(
        a::LaneSpecificAccelLatLon,
        σ::Array{Float64,1} = [NaN, NaN]
        )
        retval = new()
        retval.a = a
        retval.σ = σ
        retval
    end
end

function propagate(veh::Vehicle, action::LaneSpecificAccelLatLon, roadway::Roadway, Δt::Float64)
    lane_tag_orig = veh.state.posF.roadind.tag
    state = propagate(veh, LatLonAccel(action.a_lat, action.a_lon), roadway, Δt)
    roadproj = proj(state.posG, roadway[lane_tag_orig], roadway, move_along_curves=false)
    retval = VehicleState(Frenet(roadproj, roadway), roadway, state.v)
    return retval
end

# Check which to use, posG.Θ or posF.ϕ
function tracker!(env, measured)
    dt = 0.1
    α = 0.85
    β = 0.005
    cw_roadway = Roadway([RoadSegment(2, [env.crosswalk])])
    for i in 1:length(env.observed)
        x_hat = env.observed[i].state.posG.x + env.observed[i].state.v*cos(env.observed[i].state.posG.θ)*dt
        y_hat = env.observed[i].state.posG.y + env.observed[i].state.v*sin(env.observed[i].state.posG.θ)*dt
        rx = measured[i].state.posG.x - x_hat
        ry = measured[i].state.posG.y - y_hat
        x_hat = x_hat + rx*α
        y_hat = y_hat + ry*α
        vx_hat = env.observed[i].state.v*cos(env.observed[i].state.posG.θ) + rx*(β/dt)
        vy_hat = env.observed[i].state.v*sin(env.observed[i].state.posG.θ) + ry*(β/dt)
        v_hat = sqrt(vx_hat^2+vy_hat^2)
        θ_hat = atan2(vy_hat,vx_hat)
        env.observed[i] = Vehicle(VehicleState(VecSE2(x_hat,y_hat,θ_hat), env.crosswalk, cw_roadway, v_hat), measured[i].def, measured[i].id)
    end
    return env
end

AutomotiveDrivingModels.get_name(model::CrosswalkDriver) = "CrosswalkDriver"
function Base.rand(model::CrosswalkDriver)
    if any(isnan, model.σ) || any(x -> x<0,model.σ)
        model.a
    else
        temp = rand(MvNormal([0.0, 0.0], [model.σ[1] 0.0;0.0 model.σ[2]]))
        LaneSpecificAccelLatLon(temp[1],temp[2])
    end
end
Distributions.pdf(model::CrosswalkDriver, a::LaneSpecificAccelLatLon) = pdf(MvNormal([model.a.a_lat,model.a.a_lon],
                                                                                      [model.σ[1] 0.0;0.0 model.σ[2]]),
                                                                                        [a.a_lat, a.a_lon])
Distributions.sqmahal(model::CrosswalkDriver, a::LaneSpecificAccelLatLon) = sqmahal(MvNormal([model.a.a_lat,model.a.a_lon],
                                                                                      [model.σ[1] 0.0;0.0 model.σ[2]]),
                                                                                        [a.a_lat, a.a_lon])
