#include <commands/obtain_release_control_authority.hpp>
#include <commands/payload.hpp>

void ObtainReleaseControlAuthority::execute() {
    Logging::informationMessage("Command: 'obtain release control authority'", __FUNCTION__);

    dji_osdk_ros::ObtainControlAuthority obtainCtrlAuthority;
    const auto valid = parsePayload(_payload, obtainCtrlAuthority.request.enable_obtain);
    if (valid) {
        Logging::informationMessage(drone::utils::make_string()
                                      << "Obtain Control settings: "
                                      << static_cast<unsigned>(obtainCtrlAuthority.request.enable_obtain),
                                    __FUNCTION__);

        _client.call(obtainCtrlAuthority);

        _data.result = obtainCtrlAuthority.response.result;
    } else {
        errorIncorrectPayload();
    }

    Logging::informationMessage(drone::utils::make_string() << "Result: " << std::boolalpha << _data.result, __FUNCTION__);
}