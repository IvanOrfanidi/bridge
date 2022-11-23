#pragma once

#include <services/serial_number.hpp>
#include <services/get_camera_optical_zoom_factor.hpp>
#include <services/camera_task_get_ev.hpp>
#include <services/camera_task_get_iso.hpp>
#include <services/camera_task_get_aperture.hpp>
#include <services/get_whole_battery_info.hpp>
#include <services/get_single_battery_dynamic_info.hpp>
#include <services/logging_info.hpp>

struct ServicePool {
    static constexpr unsigned H20_CAMMERA = 0;
    static constexpr unsigned FIRST_BATTERY = 1;
    static constexpr unsigned SECOND_BATTERY = 2;

    const std::vector<std::shared_ptr<Service>> services{
      {std::make_shared<SerialNumber>()},
      {std::make_shared<GetCameraOpticalZoomFactor>(H20_CAMMERA)},
      {std::make_shared<CameraTaskGetEV>(H20_CAMMERA)},
      {std::make_shared<CameraTaskGetISO>(H20_CAMMERA)},
      {std::make_shared<CameraTaskGetAperture>(H20_CAMMERA)},
      {std::make_shared<GetWholeBatteryInfo>()},
      {std::make_shared<GetSingleBatteryDynamicInfo>(FIRST_BATTERY)},
      {std::make_shared<GetSingleBatteryDynamicInfo>(SECOND_BATTERY)},
      {std::make_shared<LoggingInfo>()},
    };
};