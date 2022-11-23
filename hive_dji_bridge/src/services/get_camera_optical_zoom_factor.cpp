#include <services/get_camera_optical_zoom_factor.hpp>
#include <commands/payload.hpp>
#include <common/math.hpp>

bool GetCameraOpticalZoomFactor::update() {
    dji_osdk_ros::GetCameraOpticalZoomFactor cameraOpticalZoomFactor;
    cameraOpticalZoomFactor.request.payloadIndex = _data.payloadIndex;

    _client.call(cameraOpticalZoomFactor);

    const bool isZero = cameraOpticalZoomFactor.response.factor < 1.0; // Bug from the drone, zoom debounce to zero.
    if (isZero || drone::math::isEqual(cameraOpticalZoomFactor.response.factor, _data.factor)) {
        return false;
    }

    _data.factor = cameraOpticalZoomFactor.response.factor;
    return true;
}