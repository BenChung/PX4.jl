module px4
#= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:115 =#
#= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:115 =#
import StaticArrays, CDRSerialization
module msg
#= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:115 =#
#= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:115 =#
import StaticArrays, CDRSerialization
const float__12 = StaticArrays.SArray{Tuple{12}, Float32, 1, 12}
module ActuatorMotors_Constants
#= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:115 =#
#= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:115 =#
import StaticArrays, CDRSerialization
const MESSAGE_VERSION = 0
const ACTUATOR_FUNCTION_MOTOR1 = 101
const NUM_CONTROLS = 12
end
begin
    #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:75 =#
    #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:75 =# @kwdef struct ActuatorMotors
            #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:76 =#
            timestamp::UInt64
            timestamp_sample::UInt64
            reversible_flags::UInt16
            control::float__12
        end
    #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:78 =#
    Base.read(rdr::CDRSerialization.CDRReader, ::Type{ActuatorMotors}) = begin
            #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:78 =#
            ActuatorMotors(; timestamp = read(rdr, UInt64), timestamp_sample = read(rdr, UInt64), reversible_flags = read(rdr, UInt16), control = read(rdr, float__12))
        end
    #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:79 =#
    Base.write(dst::CDRSerialization.CDRWriter, o::ActuatorMotors) = begin
            #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:79 =#
            #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:79 =#
            write(dst, o.timestamp)
            write(dst, o.timestamp_sample)
            write(dst, o.reversible_flags)
            write(dst, o.control)
        end
end
begin
    #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:75 =#
    #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:75 =# @kwdef struct FailsafeFlags
            #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:76 =#
            timestamp::UInt64
            mode_req_angular_velocity::UInt32
            mode_req_attitude::UInt32
            mode_req_local_alt::UInt32
            mode_req_local_position::UInt32
            mode_req_local_position_relaxed::UInt32
            mode_req_global_position::UInt32
            mode_req_mission::UInt32
            mode_req_offboard_signal::UInt32
            mode_req_home_position::UInt32
            mode_req_wind_and_flight_time_compliance::UInt32
            mode_req_prevent_arming::UInt32
            mode_req_manual_control::UInt32
            mode_req_other::UInt32
            angular_velocity_invalid::Bool
            attitude_invalid::Bool
            local_altitude_invalid::Bool
            local_position_invalid::Bool
            local_position_invalid_relaxed::Bool
            local_velocity_invalid::Bool
            global_position_invalid::Bool
            auto_mission_missing::Bool
            offboard_control_signal_lost::Bool
            home_position_invalid::Bool
            manual_control_signal_lost::Bool
            gcs_connection_lost::Bool
            battery_warning::UInt8
            battery_low_remaining_time::Bool
            battery_unhealthy::Bool
            geofence_breached::Bool
            mission_failure::Bool
            vtol_fixed_wing_system_failure::Bool
            wind_limit_exceeded::Bool
            flight_time_limit_exceeded::Bool
            local_position_accuracy_low::Bool
            navigator_failure::Bool
            fd_critical_failure::Bool
            fd_esc_arming_failure::Bool
            fd_imbalanced_prop::Bool
            fd_motor_failure::Bool
        end
    #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:78 =#
    Base.read(rdr::CDRSerialization.CDRReader, ::Type{FailsafeFlags}) = begin
            #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:78 =#
            FailsafeFlags(; timestamp = read(rdr, UInt64), mode_req_angular_velocity = read(rdr, UInt32), mode_req_attitude = read(rdr, UInt32), mode_req_local_alt = read(rdr, UInt32), mode_req_local_position = read(rdr, UInt32), mode_req_local_position_relaxed = read(rdr, UInt32), mode_req_global_position = read(rdr, UInt32), mode_req_mission = read(rdr, UInt32), mode_req_offboard_signal = read(rdr, UInt32), mode_req_home_position = read(rdr, UInt32), mode_req_wind_and_flight_time_compliance = read(rdr, UInt32), mode_req_prevent_arming = read(rdr, UInt32), mode_req_manual_control = read(rdr, UInt32), mode_req_other = read(rdr, UInt32), angular_velocity_invalid = read(rdr, Bool), attitude_invalid = read(rdr, Bool), local_altitude_invalid = read(rdr, Bool), local_position_invalid = read(rdr, Bool), local_position_invalid_relaxed = read(rdr, Bool), local_velocity_invalid = read(rdr, Bool), global_position_invalid = read(rdr, Bool), auto_mission_missing = read(rdr, Bool), offboard_control_signal_lost = read(rdr, Bool), home_position_invalid = read(rdr, Bool), manual_control_signal_lost = read(rdr, Bool), gcs_connection_lost = read(rdr, Bool), battery_warning = read(rdr, UInt8), battery_low_remaining_time = read(rdr, Bool), battery_unhealthy = read(rdr, Bool), geofence_breached = read(rdr, Bool), mission_failure = read(rdr, Bool), vtol_fixed_wing_system_failure = read(rdr, Bool), wind_limit_exceeded = read(rdr, Bool), flight_time_limit_exceeded = read(rdr, Bool), local_position_accuracy_low = read(rdr, Bool), navigator_failure = read(rdr, Bool), fd_critical_failure = read(rdr, Bool), fd_esc_arming_failure = read(rdr, Bool), fd_imbalanced_prop = read(rdr, Bool), fd_motor_failure = read(rdr, Bool))
        end
    #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:79 =#
    Base.write(dst::CDRSerialization.CDRWriter, o::FailsafeFlags) = begin
            #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:79 =#
            #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:79 =#
            write(dst, o.timestamp)
            write(dst, o.mode_req_angular_velocity)
            write(dst, o.mode_req_attitude)
            write(dst, o.mode_req_local_alt)
            write(dst, o.mode_req_local_position)
            write(dst, o.mode_req_local_position_relaxed)
            write(dst, o.mode_req_global_position)
            write(dst, o.mode_req_mission)
            write(dst, o.mode_req_offboard_signal)
            write(dst, o.mode_req_home_position)
            write(dst, o.mode_req_wind_and_flight_time_compliance)
            write(dst, o.mode_req_prevent_arming)
            write(dst, o.mode_req_manual_control)
            write(dst, o.mode_req_other)
            write(dst, o.angular_velocity_invalid)
            write(dst, o.attitude_invalid)
            write(dst, o.local_altitude_invalid)
            write(dst, o.local_position_invalid)
            write(dst, o.local_position_invalid_relaxed)
            write(dst, o.local_velocity_invalid)
            write(dst, o.global_position_invalid)
            write(dst, o.auto_mission_missing)
            write(dst, o.offboard_control_signal_lost)
            write(dst, o.home_position_invalid)
            write(dst, o.manual_control_signal_lost)
            write(dst, o.gcs_connection_lost)
            write(dst, o.battery_warning)
            write(dst, o.battery_low_remaining_time)
            write(dst, o.battery_unhealthy)
            write(dst, o.geofence_breached)
            write(dst, o.mission_failure)
            write(dst, o.vtol_fixed_wing_system_failure)
            write(dst, o.wind_limit_exceeded)
            write(dst, o.flight_time_limit_exceeded)
            write(dst, o.local_position_accuracy_low)
            write(dst, o.navigator_failure)
            write(dst, o.fd_critical_failure)
            write(dst, o.fd_esc_arming_failure)
            write(dst, o.fd_imbalanced_prop)
            write(dst, o.fd_motor_failure)
        end
end
begin
    #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:75 =#
    #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:75 =# @kwdef struct OffboardControlMode
            #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:76 =#
            timestamp::UInt64
            position::Bool
            velocity::Bool
            acceleration::Bool
            attitude::Bool
            body_rate::Bool
            thrust_and_torque::Bool
            direct_actuator::Bool
        end
    #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:78 =#
    Base.read(rdr::CDRSerialization.CDRReader, ::Type{OffboardControlMode}) = begin
            #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:78 =#
            OffboardControlMode(; timestamp = read(rdr, UInt64), position = read(rdr, Bool), velocity = read(rdr, Bool), acceleration = read(rdr, Bool), attitude = read(rdr, Bool), body_rate = read(rdr, Bool), thrust_and_torque = read(rdr, Bool), direct_actuator = read(rdr, Bool))
        end
    #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:79 =#
    Base.write(dst::CDRSerialization.CDRWriter, o::OffboardControlMode) = begin
            #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:79 =#
            #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:79 =#
            write(dst, o.timestamp)
            write(dst, o.position)
            write(dst, o.velocity)
            write(dst, o.acceleration)
            write(dst, o.attitude)
            write(dst, o.body_rate)
            write(dst, o.thrust_and_torque)
            write(dst, o.direct_actuator)
        end
end
module SensorGps_Constants
#= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:115 =#
#= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:115 =#
import StaticArrays, CDRSerialization
const FIX_TYPE_NONE = 1
const FIX_TYPE_2D = 2
const FIX_TYPE_3D = 3
const FIX_TYPE_RTCM_CODE_DIFFERENTIAL = 4
const FIX_TYPE_RTK_FLOAT = 5
const FIX_TYPE_RTK_FIXED = 6
const FIX_TYPE_EXTRAPOLATED = 8
const JAMMING_STATE_UNKNOWN = 0
const JAMMING_STATE_OK = 1
const JAMMING_STATE_WARNING = 2
const JAMMING_STATE_CRITICAL = 3
const SPOOFING_STATE_UNKNOWN = 0
const SPOOFING_STATE_NONE = 1
const SPOOFING_STATE_INDICATED = 2
const SPOOFING_STATE_MULTIPLE = 3
const RTCM_MSG_USED_UNKNOWN = 0
const RTCM_MSG_USED_NOT_USED = 1
const RTCM_MSG_USED_USED = 2
end
begin
    #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:75 =#
    #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:75 =# @kwdef struct SensorGps
            #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:76 =#
            timestamp::UInt64
            timestamp_sample::UInt64
            device_id::UInt32
            latitude_deg::Float64
            longitude_deg::Float64
            altitude_msl_m::Float64
            altitude_ellipsoid_m::Float64
            s_variance_m_s::Float32
            c_variance_rad::Float32
            fix_type::UInt8
            eph::Float32
            epv::Float32
            hdop::Float32
            vdop::Float32
            noise_per_ms::Int32
            automatic_gain_control::UInt16
            jamming_state::UInt8
            jamming_indicator::Int32
            spoofing_state::UInt8
            vel_m_s::Float32
            vel_n_m_s::Float32
            vel_e_m_s::Float32
            vel_d_m_s::Float32
            cog_rad::Float32
            vel_ned_valid::Bool
            timestamp_time_relative::Int32
            time_utc_usec::UInt64
            satellites_used::UInt8
            heading::Float32
            heading_offset::Float32
            heading_accuracy::Float32
            rtcm_injection_rate::Float32
            selected_rtcm_instance::UInt8
            rtcm_crc_failed::Bool
            rtcm_msg_used::UInt8
        end
    #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:78 =#
    Base.read(rdr::CDRSerialization.CDRReader, ::Type{SensorGps}) = begin
            #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:78 =#
            SensorGps(; timestamp = read(rdr, UInt64), timestamp_sample = read(rdr, UInt64), device_id = read(rdr, UInt32), latitude_deg = read(rdr, Float64), longitude_deg = read(rdr, Float64), altitude_msl_m = read(rdr, Float64), altitude_ellipsoid_m = read(rdr, Float64), s_variance_m_s = read(rdr, Float32), c_variance_rad = read(rdr, Float32), fix_type = read(rdr, UInt8), eph = read(rdr, Float32), epv = read(rdr, Float32), hdop = read(rdr, Float32), vdop = read(rdr, Float32), noise_per_ms = read(rdr, Int32), automatic_gain_control = read(rdr, UInt16), jamming_state = read(rdr, UInt8), jamming_indicator = read(rdr, Int32), spoofing_state = read(rdr, UInt8), vel_m_s = read(rdr, Float32), vel_n_m_s = read(rdr, Float32), vel_e_m_s = read(rdr, Float32), vel_d_m_s = read(rdr, Float32), cog_rad = read(rdr, Float32), vel_ned_valid = read(rdr, Bool), timestamp_time_relative = read(rdr, Int32), time_utc_usec = read(rdr, UInt64), satellites_used = read(rdr, UInt8), heading = read(rdr, Float32), heading_offset = read(rdr, Float32), heading_accuracy = read(rdr, Float32), rtcm_injection_rate = read(rdr, Float32), selected_rtcm_instance = read(rdr, UInt8), rtcm_crc_failed = read(rdr, Bool), rtcm_msg_used = read(rdr, UInt8))
        end
    #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:79 =#
    Base.write(dst::CDRSerialization.CDRWriter, o::SensorGps) = begin
            #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:79 =#
            #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:79 =#
            write(dst, o.timestamp)
            write(dst, o.timestamp_sample)
            write(dst, o.device_id)
            write(dst, o.latitude_deg)
            write(dst, o.longitude_deg)
            write(dst, o.altitude_msl_m)
            write(dst, o.altitude_ellipsoid_m)
            write(dst, o.s_variance_m_s)
            write(dst, o.c_variance_rad)
            write(dst, o.fix_type)
            write(dst, o.eph)
            write(dst, o.epv)
            write(dst, o.hdop)
            write(dst, o.vdop)
            write(dst, o.noise_per_ms)
            write(dst, o.automatic_gain_control)
            write(dst, o.jamming_state)
            write(dst, o.jamming_indicator)
            write(dst, o.spoofing_state)
            write(dst, o.vel_m_s)
            write(dst, o.vel_n_m_s)
            write(dst, o.vel_e_m_s)
            write(dst, o.vel_d_m_s)
            write(dst, o.cog_rad)
            write(dst, o.vel_ned_valid)
            write(dst, o.timestamp_time_relative)
            write(dst, o.time_utc_usec)
            write(dst, o.satellites_used)
            write(dst, o.heading)
            write(dst, o.heading_offset)
            write(dst, o.heading_accuracy)
            write(dst, o.rtcm_injection_rate)
            write(dst, o.selected_rtcm_instance)
            write(dst, o.rtcm_crc_failed)
            write(dst, o.rtcm_msg_used)
        end
end
const float__3 = StaticArrays.SArray{Tuple{3}, Float32, 1, 3}
module TrajectorySetpoint_Constants
#= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:115 =#
#= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:115 =#
import StaticArrays, CDRSerialization
const MESSAGE_VERSION = 0
end
begin
    #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:75 =#
    #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:75 =# @kwdef struct TrajectorySetpoint
            #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:76 =#
            timestamp::UInt64
            position::float__3
            velocity::float__3
            acceleration::float__3
            jerk::float__3
            yaw::Float32
            yawspeed::Float32
        end
    #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:78 =#
    Base.read(rdr::CDRSerialization.CDRReader, ::Type{TrajectorySetpoint}) = begin
            #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:78 =#
            TrajectorySetpoint(; timestamp = read(rdr, UInt64), position = read(rdr, float__3), velocity = read(rdr, float__3), acceleration = read(rdr, float__3), jerk = read(rdr, float__3), yaw = read(rdr, Float32), yawspeed = read(rdr, Float32))
        end
    #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:79 =#
    Base.write(dst::CDRSerialization.CDRWriter, o::TrajectorySetpoint) = begin
            #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:79 =#
            #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:79 =#
            write(dst, o.timestamp)
            write(dst, o.position)
            write(dst, o.velocity)
            write(dst, o.acceleration)
            write(dst, o.jerk)
            write(dst, o.yaw)
            write(dst, o.yawspeed)
        end
end
module VehicleCommand_Constants
#= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:115 =#
#= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:115 =#
import StaticArrays, CDRSerialization
const MESSAGE_VERSION = 0
const VEHICLE_CMD_CUSTOM_0 = 0
const VEHICLE_CMD_CUSTOM_1 = 1
const VEHICLE_CMD_CUSTOM_2 = 2
const VEHICLE_CMD_NAV_WAYPOINT = 16
const VEHICLE_CMD_NAV_LOITER_UNLIM = 17
const VEHICLE_CMD_NAV_LOITER_TURNS = 18
const VEHICLE_CMD_NAV_LOITER_TIME = 19
const VEHICLE_CMD_NAV_RETURN_TO_LAUNCH = 20
const VEHICLE_CMD_NAV_LAND = 21
const VEHICLE_CMD_NAV_TAKEOFF = 22
const VEHICLE_CMD_NAV_PRECLAND = 23
const VEHICLE_CMD_DO_ORBIT = 34
const VEHICLE_CMD_DO_FIGUREEIGHT = 35
const VEHICLE_CMD_NAV_ROI = 80
const VEHICLE_CMD_NAV_PATHPLANNING = 81
const VEHICLE_CMD_NAV_VTOL_TAKEOFF = 84
const VEHICLE_CMD_NAV_VTOL_LAND = 85
const VEHICLE_CMD_NAV_GUIDED_LIMITS = 90
const VEHICLE_CMD_NAV_GUIDED_MASTER = 91
const VEHICLE_CMD_NAV_DELAY = 93
const VEHICLE_CMD_NAV_LAST = 95
const VEHICLE_CMD_CONDITION_DELAY = 112
const VEHICLE_CMD_CONDITION_CHANGE_ALT = 113
const VEHICLE_CMD_CONDITION_DISTANCE = 114
const VEHICLE_CMD_CONDITION_YAW = 115
const VEHICLE_CMD_CONDITION_LAST = 159
const VEHICLE_CMD_CONDITION_GATE = 4501
const VEHICLE_CMD_DO_SET_MODE = 176
const VEHICLE_CMD_DO_JUMP = 177
const VEHICLE_CMD_DO_CHANGE_SPEED = 178
const VEHICLE_CMD_DO_SET_HOME = 179
const VEHICLE_CMD_DO_SET_PARAMETER = 180
const VEHICLE_CMD_DO_SET_RELAY = 181
const VEHICLE_CMD_DO_REPEAT_RELAY = 182
const VEHICLE_CMD_DO_REPEAT_SERVO = 184
const VEHICLE_CMD_DO_FLIGHTTERMINATION = 185
const VEHICLE_CMD_DO_CHANGE_ALTITUDE = 186
const VEHICLE_CMD_DO_SET_ACTUATOR = 187
const VEHICLE_CMD_DO_LAND_START = 189
const VEHICLE_CMD_DO_GO_AROUND = 191
const VEHICLE_CMD_DO_REPOSITION = 192
const VEHICLE_CMD_DO_PAUSE_CONTINUE = 193
const VEHICLE_CMD_DO_SET_ROI_LOCATION = 195
const VEHICLE_CMD_DO_SET_ROI_WPNEXT_OFFSET = 196
const VEHICLE_CMD_DO_SET_ROI_NONE = 197
const VEHICLE_CMD_DO_CONTROL_VIDEO = 200
const VEHICLE_CMD_DO_SET_ROI = 201
const VEHICLE_CMD_DO_DIGICAM_CONTROL = 203
const VEHICLE_CMD_DO_MOUNT_CONFIGURE = 204
const VEHICLE_CMD_DO_MOUNT_CONTROL = 205
const VEHICLE_CMD_DO_SET_CAM_TRIGG_DIST = 206
const VEHICLE_CMD_DO_FENCE_ENABLE = 207
const VEHICLE_CMD_DO_PARACHUTE = 208
const VEHICLE_CMD_DO_MOTOR_TEST = 209
const VEHICLE_CMD_DO_INVERTED_FLIGHT = 210
const VEHICLE_CMD_DO_GRIPPER = 211
const VEHICLE_CMD_DO_SET_CAM_TRIGG_INTERVAL = 214
const VEHICLE_CMD_DO_MOUNT_CONTROL_QUAT = 220
const VEHICLE_CMD_DO_GUIDED_MASTER = 221
const VEHICLE_CMD_DO_GUIDED_LIMITS = 222
const VEHICLE_CMD_DO_LAST = 240
const VEHICLE_CMD_PREFLIGHT_CALIBRATION = 241
const PREFLIGHT_CALIBRATION_TEMPERATURE_CALIBRATION = 3
const VEHICLE_CMD_PREFLIGHT_SET_SENSOR_OFFSETS = 242
const VEHICLE_CMD_PREFLIGHT_UAVCAN = 243
const VEHICLE_CMD_PREFLIGHT_STORAGE = 245
const VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246
const VEHICLE_CMD_OBLIQUE_SURVEY = 260
const VEHICLE_CMD_DO_SET_STANDARD_MODE = 262
const VEHICLE_CMD_GIMBAL_DEVICE_INFORMATION = 283
const VEHICLE_CMD_MISSION_START = 300
const VEHICLE_CMD_ACTUATOR_TEST = 310
const VEHICLE_CMD_CONFIGURE_ACTUATOR = 311
const VEHICLE_CMD_COMPONENT_ARM_DISARM = 400
const VEHICLE_CMD_RUN_PREARM_CHECKS = 401
const VEHICLE_CMD_INJECT_FAILURE = 420
const VEHICLE_CMD_START_RX_PAIR = 500
const VEHICLE_CMD_REQUEST_MESSAGE = 512
const VEHICLE_CMD_REQUEST_CAMERA_INFORMATION = 521
const VEHICLE_CMD_SET_CAMERA_MODE = 530
const VEHICLE_CMD_SET_CAMERA_ZOOM = 531
const VEHICLE_CMD_SET_CAMERA_FOCUS = 532
const VEHICLE_CMD_DO_GIMBAL_MANAGER_PITCHYAW = 1000
const VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE = 1001
const VEHICLE_CMD_IMAGE_START_CAPTURE = 2000
const VEHICLE_CMD_DO_TRIGGER_CONTROL = 2003
const VEHICLE_CMD_VIDEO_START_CAPTURE = 2500
const VEHICLE_CMD_VIDEO_STOP_CAPTURE = 2501
const VEHICLE_CMD_LOGGING_START = 2510
const VEHICLE_CMD_LOGGING_STOP = 2511
const VEHICLE_CMD_CONTROL_HIGH_LATENCY = 2600
const VEHICLE_CMD_DO_VTOL_TRANSITION = 3000
const VEHICLE_CMD_ARM_AUTHORIZATION_REQUEST = 3001
const VEHICLE_CMD_PAYLOAD_PREPARE_DEPLOY = 30001
const VEHICLE_CMD_PAYLOAD_CONTROL_DEPLOY = 30002
const VEHICLE_CMD_FIXED_MAG_CAL_YAW = 42006
const VEHICLE_CMD_DO_WINCH = 42600
const VEHICLE_CMD_EXTERNAL_POSITION_ESTIMATE = 43003
const VEHICLE_CMD_EXTERNAL_WIND_ESTIMATE = 43004
const VEHICLE_CMD_PX4_INTERNAL_START = 65537
const VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN = 100000
const VEHICLE_CMD_SET_NAV_STATE = 100001
const VEHICLE_MOUNT_MODE_RETRACT = 0
const VEHICLE_MOUNT_MODE_NEUTRAL = 1
const VEHICLE_MOUNT_MODE_MAVLINK_TARGETING = 2
const VEHICLE_MOUNT_MODE_RC_TARGETING = 3
const VEHICLE_MOUNT_MODE_GPS_POINT = 4
const VEHICLE_MOUNT_MODE_ENUM_END = 5
const VEHICLE_ROI_NONE = 0
const VEHICLE_ROI_WPNEXT = 1
const VEHICLE_ROI_WPINDEX = 2
const VEHICLE_ROI_LOCATION = 3
const VEHICLE_ROI_TARGET = 4
const VEHICLE_ROI_ENUM_END = 5
const PARACHUTE_ACTION_DISABLE = 0
const PARACHUTE_ACTION_ENABLE = 1
const PARACHUTE_ACTION_RELEASE = 2
const FAILURE_UNIT_SENSOR_GYRO = 0
const FAILURE_UNIT_SENSOR_ACCEL = 1
const FAILURE_UNIT_SENSOR_MAG = 2
const FAILURE_UNIT_SENSOR_BARO = 3
const FAILURE_UNIT_SENSOR_GPS = 4
const FAILURE_UNIT_SENSOR_OPTICAL_FLOW = 5
const FAILURE_UNIT_SENSOR_VIO = 6
const FAILURE_UNIT_SENSOR_DISTANCE_SENSOR = 7
const FAILURE_UNIT_SENSOR_AIRSPEED = 8
const FAILURE_UNIT_SYSTEM_BATTERY = 100
const FAILURE_UNIT_SYSTEM_MOTOR = 101
const FAILURE_UNIT_SYSTEM_SERVO = 102
const FAILURE_UNIT_SYSTEM_AVOIDANCE = 103
const FAILURE_UNIT_SYSTEM_RC_SIGNAL = 104
const FAILURE_UNIT_SYSTEM_MAVLINK_SIGNAL = 105
const FAILURE_TYPE_OK = 0
const FAILURE_TYPE_OFF = 1
const FAILURE_TYPE_STUCK = 2
const FAILURE_TYPE_GARBAGE = 3
const FAILURE_TYPE_WRONG = 4
const FAILURE_TYPE_SLOW = 5
const FAILURE_TYPE_DELAYED = 6
const FAILURE_TYPE_INTERMITTENT = 7
const SPEED_TYPE_AIRSPEED = 0
const SPEED_TYPE_GROUNDSPEED = 1
const SPEED_TYPE_CLIMB_SPEED = 2
const SPEED_TYPE_DESCEND_SPEED = 3
const ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TO_CIRCLE_CENTER = 0
const ORBIT_YAW_BEHAVIOUR_HOLD_INITIAL_HEADING = 1
const ORBIT_YAW_BEHAVIOUR_UNCONTROLLED = 2
const ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TANGENT_TO_CIRCLE = 3
const ORBIT_YAW_BEHAVIOUR_RC_CONTROLLED = 4
const ORBIT_YAW_BEHAVIOUR_UNCHANGED = 5
const ARMING_ACTION_DISARM = 0
const ARMING_ACTION_ARM = 1
const GRIPPER_ACTION_RELEASE = 0
const GRIPPER_ACTION_GRAB = 1
const ORB_QUEUE_LENGTH = 8
const COMPONENT_MODE_EXECUTOR_START = 1000
end
begin
    #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:75 =#
    #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:75 =# @kwdef struct VehicleCommand
            #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:76 =#
            timestamp::UInt64
            param1::Float32
            param2::Float32
            param3::Float32
            param4::Float32
            param5::Float64
            param6::Float64
            param7::Float32
            command::UInt32
            target_system::UInt8
            target_component::UInt8
            source_system::UInt8
            source_component::UInt16
            confirmation::UInt8
            from_external::Bool
        end
    #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:78 =#
    Base.read(rdr::CDRSerialization.CDRReader, ::Type{VehicleCommand}) = begin
            #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:78 =#
            VehicleCommand(; timestamp = read(rdr, UInt64), param1 = read(rdr, Float32), param2 = read(rdr, Float32), param3 = read(rdr, Float32), param4 = read(rdr, Float32), param5 = read(rdr, Float64), param6 = read(rdr, Float64), param7 = read(rdr, Float32), command = read(rdr, UInt32), target_system = read(rdr, UInt8), target_component = read(rdr, UInt8), source_system = read(rdr, UInt8), source_component = read(rdr, UInt16), confirmation = read(rdr, UInt8), from_external = read(rdr, Bool))
        end
    #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:79 =#
    Base.write(dst::CDRSerialization.CDRWriter, o::VehicleCommand) = begin
            #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:79 =#
            #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:79 =#
            write(dst, o.timestamp)
            write(dst, o.param1)
            write(dst, o.param2)
            write(dst, o.param3)
            write(dst, o.param4)
            write(dst, o.param5)
            write(dst, o.param6)
            write(dst, o.param7)
            write(dst, o.command)
            write(dst, o.target_system)
            write(dst, o.target_component)
            write(dst, o.source_system)
            write(dst, o.source_component)
            write(dst, o.confirmation)
            write(dst, o.from_external)
        end
end
module VehicleCommandAck_Constants
#= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:115 =#
#= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:115 =#
import StaticArrays, CDRSerialization
const MESSAGE_VERSION = 0
const VEHICLE_CMD_RESULT_ACCEPTED = 0
const VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED = 1
const VEHICLE_CMD_RESULT_DENIED = 2
const VEHICLE_CMD_RESULT_UNSUPPORTED = 3
const VEHICLE_CMD_RESULT_FAILED = 4
const VEHICLE_CMD_RESULT_IN_PROGRESS = 5
const VEHICLE_CMD_RESULT_CANCELLED = 6
const ARM_AUTH_DENIED_REASON_GENERIC = 0
const ARM_AUTH_DENIED_REASON_NONE = 1
const ARM_AUTH_DENIED_REASON_INVALID_WAYPOINT = 2
const ARM_AUTH_DENIED_REASON_TIMEOUT = 3
const ARM_AUTH_DENIED_REASON_AIRSPACE_IN_USE = 4
const ARM_AUTH_DENIED_REASON_BAD_WEATHER = 5
const ORB_QUEUE_LENGTH = 4
end
begin
    #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:75 =#
    #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:75 =# @kwdef struct VehicleCommandAck
            #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:76 =#
            timestamp::UInt64
            command::UInt32
            result::UInt8
            result_param1::UInt8
            result_param2::Int32
            target_system::UInt8
            target_component::UInt16
            from_external::Bool
        end
    #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:78 =#
    Base.read(rdr::CDRSerialization.CDRReader, ::Type{VehicleCommandAck}) = begin
            #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:78 =#
            VehicleCommandAck(; timestamp = read(rdr, UInt64), command = read(rdr, UInt32), result = read(rdr, UInt8), result_param1 = read(rdr, UInt8), result_param2 = read(rdr, Int32), target_system = read(rdr, UInt8), target_component = read(rdr, UInt16), from_external = read(rdr, Bool))
        end
    #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:79 =#
    Base.write(dst::CDRSerialization.CDRWriter, o::VehicleCommandAck) = begin
            #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:79 =#
            #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:79 =#
            write(dst, o.timestamp)
            write(dst, o.command)
            write(dst, o.result)
            write(dst, o.result_param1)
            write(dst, o.result_param2)
            write(dst, o.target_system)
            write(dst, o.target_component)
            write(dst, o.from_external)
        end
end
const float__2 = StaticArrays.SArray{Tuple{2}, Float32, 1, 2}
module VehicleLocalPosition_Constants
#= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:115 =#
#= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:115 =#
import StaticArrays, CDRSerialization
const MESSAGE_VERSION = 0
const DIST_BOTTOM_SENSOR_NONE = 0
const DIST_BOTTOM_SENSOR_RANGE = 1
const DIST_BOTTOM_SENSOR_FLOW = 2
end
begin
    #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:75 =#
    #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:75 =# @kwdef struct VehicleLocalPosition
            #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:76 =#
            timestamp::UInt64
            timestamp_sample::UInt64
            xy_valid::Bool
            z_valid::Bool
            v_xy_valid::Bool
            v_z_valid::Bool
            x::Float32
            y::Float32
            z::Float32
            delta_xy::float__2
            xy_reset_counter::UInt8
            delta_z::Float32
            z_reset_counter::UInt8
            vx::Float32
            vy::Float32
            vz::Float32
            z_deriv::Float32
            delta_vxy::float__2
            vxy_reset_counter::UInt8
            delta_vz::Float32
            vz_reset_counter::UInt8
            ax::Float32
            ay::Float32
            az::Float32
            heading::Float32
            heading_var::Float32
            unaided_heading::Float32
            delta_heading::Float32
            heading_reset_counter::UInt8
            heading_good_for_control::Bool
            tilt_var::Float32
            xy_global::Bool
            z_global::Bool
            ref_timestamp::UInt64
            ref_lat::Float64
            ref_lon::Float64
            ref_alt::Float32
            dist_bottom_valid::Bool
            dist_bottom::Float32
            dist_bottom_var::Float32
            delta_dist_bottom::Float32
            dist_bottom_reset_counter::UInt8
            dist_bottom_sensor_bitfield::UInt8
            eph::Float32
            epv::Float32
            evh::Float32
            evv::Float32
            dead_reckoning::Bool
            vxy_max::Float32
            vz_max::Float32
            hagl_min::Float32
            hagl_max_z::Float32
            hagl_max_xy::Float32
        end
    #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:78 =#
    Base.read(rdr::CDRSerialization.CDRReader, ::Type{VehicleLocalPosition}) = begin
            #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:78 =#
            VehicleLocalPosition(; timestamp = read(rdr, UInt64), timestamp_sample = read(rdr, UInt64), xy_valid = read(rdr, Bool), z_valid = read(rdr, Bool), v_xy_valid = read(rdr, Bool), v_z_valid = read(rdr, Bool), x = read(rdr, Float32), y = read(rdr, Float32), z = read(rdr, Float32), delta_xy = read(rdr, float__2), xy_reset_counter = read(rdr, UInt8), delta_z = read(rdr, Float32), z_reset_counter = read(rdr, UInt8), vx = read(rdr, Float32), vy = read(rdr, Float32), vz = read(rdr, Float32), z_deriv = read(rdr, Float32), delta_vxy = read(rdr, float__2), vxy_reset_counter = read(rdr, UInt8), delta_vz = read(rdr, Float32), vz_reset_counter = read(rdr, UInt8), ax = read(rdr, Float32), ay = read(rdr, Float32), az = read(rdr, Float32), heading = read(rdr, Float32), heading_var = read(rdr, Float32), unaided_heading = read(rdr, Float32), delta_heading = read(rdr, Float32), heading_reset_counter = read(rdr, UInt8), heading_good_for_control = read(rdr, Bool), tilt_var = read(rdr, Float32), xy_global = read(rdr, Bool), z_global = read(rdr, Bool), ref_timestamp = read(rdr, UInt64), ref_lat = read(rdr, Float64), ref_lon = read(rdr, Float64), ref_alt = read(rdr, Float32), dist_bottom_valid = read(rdr, Bool), dist_bottom = read(rdr, Float32), dist_bottom_var = read(rdr, Float32), delta_dist_bottom = read(rdr, Float32), dist_bottom_reset_counter = read(rdr, UInt8), dist_bottom_sensor_bitfield = read(rdr, UInt8), eph = read(rdr, Float32), epv = read(rdr, Float32), evh = read(rdr, Float32), evv = read(rdr, Float32), dead_reckoning = read(rdr, Bool), vxy_max = read(rdr, Float32), vz_max = read(rdr, Float32), hagl_min = read(rdr, Float32), hagl_max_z = read(rdr, Float32), hagl_max_xy = read(rdr, Float32))
        end
    #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:79 =#
    Base.write(dst::CDRSerialization.CDRWriter, o::VehicleLocalPosition) = begin
            #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:79 =#
            #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:79 =#
            write(dst, o.timestamp)
            write(dst, o.timestamp_sample)
            write(dst, o.xy_valid)
            write(dst, o.z_valid)
            write(dst, o.v_xy_valid)
            write(dst, o.v_z_valid)
            write(dst, o.x)
            write(dst, o.y)
            write(dst, o.z)
            write(dst, o.delta_xy)
            write(dst, o.xy_reset_counter)
            write(dst, o.delta_z)
            write(dst, o.z_reset_counter)
            write(dst, o.vx)
            write(dst, o.vy)
            write(dst, o.vz)
            write(dst, o.z_deriv)
            write(dst, o.delta_vxy)
            write(dst, o.vxy_reset_counter)
            write(dst, o.delta_vz)
            write(dst, o.vz_reset_counter)
            write(dst, o.ax)
            write(dst, o.ay)
            write(dst, o.az)
            write(dst, o.heading)
            write(dst, o.heading_var)
            write(dst, o.unaided_heading)
            write(dst, o.delta_heading)
            write(dst, o.heading_reset_counter)
            write(dst, o.heading_good_for_control)
            write(dst, o.tilt_var)
            write(dst, o.xy_global)
            write(dst, o.z_global)
            write(dst, o.ref_timestamp)
            write(dst, o.ref_lat)
            write(dst, o.ref_lon)
            write(dst, o.ref_alt)
            write(dst, o.dist_bottom_valid)
            write(dst, o.dist_bottom)
            write(dst, o.dist_bottom_var)
            write(dst, o.delta_dist_bottom)
            write(dst, o.dist_bottom_reset_counter)
            write(dst, o.dist_bottom_sensor_bitfield)
            write(dst, o.eph)
            write(dst, o.epv)
            write(dst, o.evh)
            write(dst, o.evv)
            write(dst, o.dead_reckoning)
            write(dst, o.vxy_max)
            write(dst, o.vz_max)
            write(dst, o.hagl_min)
            write(dst, o.hagl_max_z)
            write(dst, o.hagl_max_xy)
        end
end
const float__4 = StaticArrays.SArray{Tuple{4}, Float32, 1, 4}
module VehicleOdometry_Constants
#= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:115 =#
#= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:115 =#
import StaticArrays, CDRSerialization
const MESSAGE_VERSION = 0
const POSE_FRAME_UNKNOWN = 0
const POSE_FRAME_NED = 1
const POSE_FRAME_FRD = 2
const VELOCITY_FRAME_UNKNOWN = 0
const VELOCITY_FRAME_NED = 1
const VELOCITY_FRAME_FRD = 2
const VELOCITY_FRAME_BODY_FRD = 3
end
begin
    #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:75 =#
    #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:75 =# @kwdef struct VehicleOdometry
            #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:76 =#
            timestamp::UInt64
            timestamp_sample::UInt64
            pose_frame::UInt8
            position::float__3
            q::float__4
            velocity_frame::UInt8
            velocity::float__3
            angular_velocity::float__3
            position_variance::float__3
            orientation_variance::float__3
            velocity_variance::float__3
            reset_counter::UInt8
            quality::Int8
        end
    #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:78 =#
    Base.read(rdr::CDRSerialization.CDRReader, ::Type{VehicleOdometry}) = begin
            #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:78 =#
            VehicleOdometry(; timestamp = read(rdr, UInt64), timestamp_sample = read(rdr, UInt64), pose_frame = read(rdr, UInt8), position = read(rdr, float__3), q = read(rdr, float__4), velocity_frame = read(rdr, UInt8), velocity = read(rdr, float__3), angular_velocity = read(rdr, float__3), position_variance = read(rdr, float__3), orientation_variance = read(rdr, float__3), velocity_variance = read(rdr, float__3), reset_counter = read(rdr, UInt8), quality = read(rdr, Int8))
        end
    #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:79 =#
    Base.write(dst::CDRSerialization.CDRWriter, o::VehicleOdometry) = begin
            #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:79 =#
            #= /mnt/e/work/juliahub/drone/IDL/src/generation/gen.jl:79 =#
            write(dst, o.timestamp)
            write(dst, o.timestamp_sample)
            write(dst, o.pose_frame)
            write(dst, o.position)
            write(dst, o.q)
            write(dst, o.velocity_frame)
            write(dst, o.velocity)
            write(dst, o.angular_velocity)
            write(dst, o.position_variance)
            write(dst, o.orientation_variance)
            write(dst, o.velocity_variance)
            write(dst, o.reset_counter)
            write(dst, o.quality)
        end
end
end
end
