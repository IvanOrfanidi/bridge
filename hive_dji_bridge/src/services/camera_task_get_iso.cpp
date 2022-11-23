#include <services/camera_task_get_iso.hpp>
#include <commands/payload.hpp>
#include <common/math.hpp>

bool CameraTaskGetISO::update() {
    dji_osdk_ros::CameraISO cameraIso;
    cameraIso.request.payload_index = _data.payloadIndex;

    _client.call(cameraIso);

    if (cameraIso.response.iso_data == _data.isoData) {
        return false;
    }

    _data.isoData = cameraIso.response.iso_data;
    return true;
}