"""
BlackBoxOptimization with the AutomotiveSimulator
3 lanes with 4 vehicles.
"""

using BlackBoxOptim
using AutomotiveSimulator
using AutomotiveVisualization
using Logging
using Reel

"""
definition of the param bounds can be found at the beginning of `sim_two_lane` function.
"""
params_bounds = [
    (9.0, 11.0),
]
params_dim = length(params_bounds)

"""
    scenario_simulation(params::Array)

Function to simulate the scenario
"""
function scenario_simulation(params::Array)
    start_position_d2 = params[1]

    @debug "Parameeters used in this run: " params

    global roadway = gen_straight_roadway(3, 10_000.0) # 1_000m
    w = DEFAULT_LANE_WIDTH

    "scene definiton"
    global scene = Scene([
        Entity(VehicleState(VecSE2(100.0, 0.0, 0.0), roadway, 10.0), VehicleDef(), "ego"),
        Entity(VehicleState(VecSE2(start_position_d2, 0.0,0.0), roadway, 10.0), VehicleDef(), "d2"),
        Entity(VehicleState(VecSE2(10.0, 1w,0.0), roadway, 10.0), VehicleDef(), "d3"),
        Entity(VehicleState(VecSE2(40.0, 0.0,0.0), roadway, 10.0), VehicleDef(), "d4"),
    ])

    global models = Dict{String, DriverModel}(
        "ego" => Tim2DDriver(
                            mlon=IntelligentDriverModel(v_des=15.0),
                            mlat=ProportionalLaneTracker(),
                            ),
        "d2" => Tim2DDriver(mlon=IntelligentDriverModel(v_des=15.0)),
        "d3" => Tim2DDriver(mlon=IntelligentDriverModel(v_des=15.0)),
        "d4" => Tim2DDriver(mlon=IntelligentDriverModel(v_des=15.0)),
    )

    global nticks = 200
    global timestep = 0.1
    global scenes = simulate(scene, roadway, models, nticks, timestep)# , callbacks = MyCollisionCallback)

    distance_to("d2")
    distance_to("d3")
    distance_to("d4")

    global features = extract_features(
        (
            posgx, 
            posgy, 
            velgx, 
            velgy, 
            iscolliding, 
            distance_to_d2,
            distance_to_d3,
            distance_to_d4,
        ),
        roadway, 
        scenes, 
        ["ego"]
    )

    @debug "safety: $(round(safety, digits=2))"

    safety = 0.0

end

"""
TODO bboptimize needs to be started twice in order to find `distance_to_d2` as a function.
"""

@debug "Optimization started"
"""optimization"""
res = bboptimize(
    scenario_simulation,
    SearchRange = params_bounds,
    NumDimensions = params_dim,
    MaxTime = 10, # time in seconds
)

scenario_simulation(best_candidate(res))

AutomotiveVisualization.colortheme["background"] = colorant"white"
camera = SceneFollowCamera(padding=30.)
idoverlay = IDOverlay(scene=scene, color=colorant"black", font_size=10, y_off=2.)

animation = roll(fps=1.0/timestep, duration=nticks*timestep) do t, dt
    i = Int(floor(t/dt)) + 1
    idoverlay.scene = scenes[i]
    update_camera!(camera, scenes[i])
    text = TextOverlay(
                text=[
                    ""
                    ],
                font_size=16, pos=VecE2(50.0, 100.0), color=colorant"black",
                coordinate_system=:camera_pixels
                )
    renderables = [roadway, scenes[i], idoverlay, text]
    render(renderables, camera=camera)
end

write("BBOAutoSim.gif", animation)