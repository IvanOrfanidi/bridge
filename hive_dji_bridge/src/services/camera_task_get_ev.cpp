#include <services/camera_task_get_ev.hpp>
#include <commands/payload.hpp>
#include <common/math.hpp>

bool CameraTaskGetEV::update() {
    dji_osdk_ros::CameraEV cameraEv;
    cameraEv.request.payload_index = _data.payloadIndex;

    _client.call(cameraEv);

    if (cameraEv.response.exposure_compensation == _data.exposureCompensation) {
        return false;
    }

    _data.exposureCompensation = cameraEv.response.exposure_compensation;
    return true;
}