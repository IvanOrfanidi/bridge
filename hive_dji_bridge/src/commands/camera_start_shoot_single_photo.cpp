#include <commands/camera_start_shoot_single_photo.hpp>
#include <commands/payload.hpp>

void CameraStartShootSinglePhoto::execute() {
    Logging::informationMessage("Command: 'camera start shoot single photo'", __FUNCTION__);

    dji_osdk_ros::CameraStartShootSinglePhoto cameraStartShootSinglePhoto;
    // payload_index - usually this is the camera index (for H20 = 0).
    const auto valid = parsePayload(_payload, cameraStartShootSinglePhoto.request.payload_index);
    if (valid) {
        Logging::informationMessage(drone::utils::make_string()
                                      << "Camera Shoot Photo settings: "
                                      << static_cast<unsigned>(cameraStartShootSinglePhoto.request.payload_index),
                                    __FUNCTION__);

        _client.call(cameraStartShootSinglePhoto);

        _data.result = cameraStartShootSinglePhoto.response.result;
    } else {
        errorIncorrectPayload();
    }

    Logging::informationMessage(drone::utils::make_string() << "Result: " << std::boolalpha << _data.result, __FUNCTION__);
}