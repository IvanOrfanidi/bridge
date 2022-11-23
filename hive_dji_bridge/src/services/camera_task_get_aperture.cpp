#include <services/camera_task_get_aperture.hpp>
#include <commands/payload.hpp>
#include <common/math.hpp>

bool CameraTaskGetAperture::update() {
    dji_osdk_ros::CameraAperture cameraAperture;
    cameraAperture.request.payload_index = _data.payloadIndex;

    _client.call(cameraAperture);

    if (cameraAperture.response.aperture == _data.aperture) {
        return false;
    }

    _data.aperture = cameraAperture.response.aperture;
    return true;
}