#include <unordered_map>

#include <commands/command.hpp>
#include <commands/set_flight_task.hpp>
#include <commands/set_rtk_enable.hpp>
#include <commands/set_simulation_on.hpp>
#include <commands/obtain_release_control_authority.hpp>
#include <commands/gimbal_task_control.hpp>
#include <commands/set_camera_optical_zoom_factor.hpp>
#include <commands/camera_start_shoot_single_photo.hpp>
#include <commands/camera_record_video_action.hpp>
#include <commands/camera_task_set_ev.hpp>
#include <commands/camera_task_set_iso.hpp>
#include <commands/camera_task_set_aperture.hpp>
#include <commands/camera_task_set_focus_point.hpp>
#include <commands/camera_task_tap_zoom_point.hpp>
#include <commands/camera_task_zoom_ctrl.hpp>
#include <commands/change_camera_h264_source.hpp>
#include <commands/waypointv2_pausemission.hpp>
#include <commands/waypointv2_resumemission.hpp>
#include <commands/waypointv2_stopmission.hpp>
#include <commands/waypointv2_startmission.hpp>
#include <commands/flight_control_setpoint_generic.hpp>

#include <application/logging.hpp>
#include <common/make_string.hpp>
#include <nlohmann/json.hpp>

// Command factory
// clang-format off
const std::unordered_map<std::string, std::function<std::shared_ptr<Command>(const CommandId& id, const CommandPayload& payload)>> COMMAND_LIST {
    {"set_flight_task",                     [](const auto& id, const auto& payload)
                                                { return std::make_shared<SetFlightTask>(id, payload); }},
    {"set_rtk_enable",                      [](const auto& id, const auto& payload)
                                                { return std::make_shared<SetRtkEnable>(id, payload); }},
    {"set_simulation_on",                   [](const auto& id, const auto& payload)
                                                { return std::make_shared<SetSimulationOn>(id, payload); }},
    {"obtain_release_control_authority",    [](const auto& id, const auto& payload)
                                                { return std::make_shared<ObtainReleaseControlAuthority>(id, payload); }},
    {"gimbal_task_control",                 [](const auto& id, const auto& payload)
                                                { return std::make_shared<GimbalTaskControl>(id, payload); }},
    {"set_camera_optical_zoom_factor",      [](const auto& id, const auto& payload)
                                                { return std::make_shared<SetCameraOpticalZoomFactor>(id, payload); }},
    {"camera_start_shoot_single_photo",     [](const auto& id, const auto& payload)
                                                { return std::make_shared<CameraStartShootSinglePhoto>(id, payload); }},
    {"camera_record_video_action",          [](const auto& id, const auto& payload)
                                                { return std::make_shared<CameraRecordVideoAction>(id, payload); }},
    {"camera_task_set_ev",                  [](const auto& id, const auto& payload)
                                                { return std::make_shared<CameraTaskSetEV>(id, payload); }},
    {"camera_task_set_iso",                 [](const auto& id, const auto& payload)
                                                { return std::make_shared<CameraTaskSetISO>(id, payload); }},
    {"camera_task_set_aperture",            [](const auto& id, const auto& payload)
                                                { return std::make_shared<CameraTaskSetAperture>(id, payload); }},
    {"camera_task_set_focus_point",         [](const auto& id, const auto& payload)
                                                { return std::make_shared<CameraFocusPoint>(id, payload); }},
    {"camera_task_tap_zoom_point",          [](const auto& id, const auto& payload)
                                                { return std::make_shared<CameraTapZoomPoint>(id, payload); }},
    {"camera_task_zoom_ctrl",               [](const auto& id, const auto& payload)
                                                { return std::make_shared<CameraZoomCtrl>(id, payload); }},
    {"change_camera_h264_source",           [](const auto& id, const auto& payload)
                                                { return std::make_shared<ChangeCameraH264Source>(id, payload); }},
    {"waypointv2_pausemission",             [](const auto& id, const auto& payload)
                                                { return std::make_shared<PauseWaypointV2Mission>(id, payload); }},
    {"waypointv2_resumemission",            [](const auto& id, const auto& payload)
                                                { return std::make_shared<ResumeWaypointV2Mission>(id, payload); }},
    {"waypointv2_stopmission",              [](const auto& id, const auto& payload)
                                                { return std::make_shared<StopWaypointV2Mission>(id, payload); }},
    {"waypointv2_startmission",             [](const auto& id, const auto& payload)
                                                { return std::make_shared<StartWaypointV2Mission>(id, payload); }},
    {"flight_control_setpoint_generic",     [](const auto& id, const auto& payload)
                                                { return std::make_shared<FlightControlSetpointGeneric>(id, payload); }},
};
// clang-format on

// Return nullptr if the command is unknown. The Command will not be created.
std::shared_ptr<Command> Command::create(const CommandCode& code, const CommandId& id, const CommandPayload& payload) {
    const std::string command = code.data();
    const auto it = COMMAND_LIST.find(command);
    if (it != COMMAND_LIST.end()) {
        return (it->second)(id, payload);
    }

    Logging::errorMessage(drone::utils::make_string() << "Unknown command: " << command, __FUNCTION__);
    return nullptr;
}
