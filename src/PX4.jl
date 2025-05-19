module PX4
import IDLParser
import IDLParser.Parse: open_idl
import IDLParser.ConstResolution: resolve_constants
import IDLParser.Generation: generate_code
using Zenoh, CDRSerialization
using CDRSerialization: CDRReader
using Rotations, LinearAlgebra
using WGLMakie, StaticArrays, Makie


code = generate_code([
    resolve_constants(convert(Vector{IDLParser.Parse.Decl}, open_idl(joinpath(@__DIR__, "..", "idl/VehicleLocalPosition.idl"))))[1],
    resolve_constants(convert(Vector{IDLParser.Parse.Decl}, open_idl(joinpath(@__DIR__, "..", "idl/VehicleOdometry.idl"))))[1],
    resolve_constants(convert(Vector{IDLParser.Parse.Decl}, open_idl(joinpath(@__DIR__, "..", "idl/FailsafeFlags.idl"))))[1],
    resolve_constants(convert(Vector{IDLParser.Parse.Decl}, open_idl(joinpath(@__DIR__, "..", "idl/VehicleCommand.idl"))))[1],
    resolve_constants(convert(Vector{IDLParser.Parse.Decl}, open_idl(joinpath(@__DIR__, "..", "idl/OffboardControlMode.idl"))))[1],
    resolve_constants(convert(Vector{IDLParser.Parse.Decl}, open_idl(joinpath(@__DIR__, "..", "idl/TrajectorySetpoint.idl"))))[1]])
eval(code[1])

pub_setpoint = Zenoh.Keyexpr("setpoint")
pub_mode = Zenoh.Keyexpr("offboard_mode")
pub_command = Zenoh.Keyexpr("vehicle_command")


# Drone properties
Base.@kwdef struct QuadrotorModel
    mass::Float64  # [kg] = 1.5 + 0.015 + 0.005*4 for iris
    g::Float64 # [m/s^2] = 9.82
    e3::Vector{Int64} = [0; 0; 1]
end

# Geometric control gains
Base.@kwdef struct GeometricControlParams
    kx::Float64
    kv::Float64
end

function circle()
    sub_odo = nothing
    sub_failsafes = nothing
    s = nothing
    try

        c = Zenoh.Config(; str = """{connect: { endpoints: ["tcp/localhost:7448"]}}""")
        s = open(c)

ready_for_arm = Threads.Atomic{Bool}(false)
armed = false
start_time = UInt64(0)
ref_pos = [0.0, 0.0, 0.0]
target_pos = [0.0, 0.0, 0.0]
ref_vel = [NaN, NaN, NaN]
ref_accel = [NaN, NaN, NaN]
data = IOBuffer()

pts = Observable([Point2f(0.0, 0.0), Point2f(0.0, 0.0)])
f=Figure()
mp=Axis(f[1,1:2])
scatter!(mp, pts, color=[:red, :blue])
sl_kx = Slider(f[2,1], range=0:0.01:10.0, startvalue=1.0, update_while_dragging=true)
sl_kv = Slider(f[2,2], range=0:0.01:10.0, startvalue=1.0, update_while_dragging=true)
display(f)

iris_model = QuadrotorModel(mass = 1.5 + 0.015 + 0.005*4, g = 0.0) # g is already compensated by px4
ctrl_pars = GeometricControlParams(kx = 1.0, kv=1.0)
function geometric_force_controller(x_des, xd_des, xdd_des, x, v, R, QuadModel::QuadrotorModel, ControlGains::GeometricControlParams)
    kx = ControlGains.kx
    kv = ControlGains.kv
    g = QuadModel.g
    e3 = QuadModel.e3
    mass = QuadModel.mass

    # Errors in state
    ex = x - x_des
    ev = v - xd_des

    # Force Control

    return (-kx * ex - kv * ev - mass * g * e3 + mass * xdd_des)/mass
end



sub_odo = open((sample) -> begin 
    p = Zenoh.payload(sample)
    open(p, Val(:read)) do str
        r = CDRReader(str)
        odometry = read(r, px4.msg.VehicleOdometry)
        if !armed 
            ref_pos .= odometry.position
            target_pos .= ref_pos
        else
            elapsed = (odometry.timestamp - start_time)/1e6
            speed = 1.25
            size = 5.0
            travel_dist = elapsed * speed
            target_pos .= SVector(size*sin(travel_dist+3π/2)+1 + ref_pos[1], size*cos(travel_dist+3π/2) + ref_pos[2], ref_pos[3])
            ref_vel .= SVector(size*speed*cos(travel_dist+3π/2), -size*speed*sin(travel_dist+3π/2), 0.0)
            target_accel = SVector(-size*speed*speed*sin(travel_dist+3π/2), -size*speed*speed*cos(travel_dist+3π/2), 0.0)
            ref_accel .= geometric_force_controller(target_pos, ref_vel, target_accel, odometry.position, odometry.velocity, QuatRotation(1.0, 0.0, 0.0, 0.0), iris_model, ctrl_pars)
            pts[] = [Point2f(odometry.position[1], odometry.position[2]), Point2f(target_pos[1], target_pos[2])]
            ctrl_pars = GeometricControlParams(kx = sl_kx.value[], kv = sl_kv.value[])
            #ref_accel .= SVector(-size*speed*speed*sin(travel_dist+3π/2), -size*speed*speed*cos(travel_dist+3π/2), 0.0)
        end
        cmd_mode=PX4.px4.msg.OffboardControlMode(0, false, false, true, false, false, false, false)
        w = CDRSerialization.CDRWriter(data, CDRSerialization.CDR_LE)
        write(w, cmd_mode)
        Zenoh.put(s, pub_mode, take!(data))
        target = PX4.px4.msg.TrajectorySetpoint(;
            timestamp = 0, 
            position = target_pos, velocity = ref_vel, 
            acceleration = ref_accel, jerk = [NaN, NaN, NaN], 
            yaw = NaN, yawspeed = NaN)
        w = CDRSerialization.CDRWriter(data, CDRSerialization.CDR_LE)
        write(w, target)
        Zenoh.put(s, pub_setpoint, take!(data))
        if ready_for_arm[] && !armed
            armed = true
            start_time = odometry.timestamp
            w = CDRSerialization.CDRWriter(data, CDRSerialization.CDR_LE)
            msg = PX4.px4.msg.VehicleCommand(;
                timestamp=0,
                command=PX4.px4.msg.VehicleCommand_Constants.VEHICLE_CMD_DO_SET_MODE,
                param1=1, param2=6, param3=0, param4=0,param5=0, param6=0, param7=0,
                target_system=1, target_component=1, source_system=1, source_component=1, confirmation=0, from_external=true)
            write(w, msg)
            Zenoh.put(s, pub_command, take!(data))
        end
        println("position: $(odometry.position) target:$(target)")
    end
end, s, Zenoh.Keyexpr("odo"))
sub_failsafes = open((sample) -> begin 
    p = Zenoh.payload(sample)
    open(p, Val(:read)) do str
        r = CDRReader(str)
        failsafes = read(r, px4.msg.FailsafeFlags)
        Threads.atomic_xchg!(ready_for_arm, !failsafes.offboard_control_signal_lost)
        @show !failsafes.offboard_control_signal_lost
    end
end, s, Zenoh.Keyexpr("failsafes"))
readline()
finally
    if !isnothing(sub_odo) close(sub_odo) end
    if !isnothing(sub_failsafes) close(sub_failsafes) end
    if !isnothing(s) close(s) end
end
end

end
