#pragma once

#include <array>
#include <common/operators.hpp>
#include <data/subscriber.hpp>
#include <data/header.hpp>

class GpsPosition : public Subscriber {
public:
    enum class Status : int8_t {
        NO_FIX = -1,  // unable to fix position
        FIX = 0,      // unaugmented fix
        SBAS_FIX = 1, // with satellite-based augmentation
        GBAS_FIX = 2, // with ground-based augmentation
    };

    enum Service : uint16_t {
        GPS = (1 << 0),
        GLONASS = (1 << 1),
        COMPASS = (1 << 2), // includes BeiDou.
        GALILEO = (1 << 3),
    };

    struct NavigationSatellite {
        Status status = Status::NO_FIX;
        uint16_t service = 0;

        static std::string statusToString(Status satelliteStatus) {
            static const std::map<Status, std::string> mapOfMsgStatus = {
              {Status::NO_FIX, "no fix"},
              {Status::FIX, "fix"},
              {Status::SBAS_FIX, "sbas fix"},
              {Status::GBAS_FIX, "gbas fix"},
            };
            const auto it = mapOfMsgStatus.find(satelliteStatus);
            return (it != mapOfMsgStatus.end()) ? it->second : "UNKNOWN";
        }

        static std::string serviceToString(uint16_t satelliteService) {
            static const std::map<Service, std::string> mapOfMsgService = {
              {GPS, "gps"},
              {GLONASS, "glonass"},
              {COMPASS, "compass"},
              {GALILEO, "galileo"},
            };
            std::string out;
            for (const auto& [key, service] : mapOfMsgService) {
                if (satelliteService & key) {
                    out += service + " & ";
                }
            }
            if (out.empty()) {
                return "UNKNOWN";
            }
            out.erase(out.size() - 3);
            return out;
        }
    };

    enum class CovarianceType : uint8_t {
        UNKNOWN = 0,
        APPROXIMATED = 1,
        DIAGONAL_KNOWN = 2,
        KNOWN = 3,
    };

    struct Data {
        Header header{};
        NavigationSatellite navigationSatellite{};
        double latitude = 0.0;
        double longitude = 0.0;
        double altitude = 0.0;
        std::array<double, 9> positionCovariance{};
        CovarianceType positionCovarianceType = CovarianceType::UNKNOWN;
    };

    std::type_index getType() override {
        return this->getMyselfType();
    }

    static std::type_index getMyselfType() {
        return typeid(GpsPosition);
    }

    Data getData() const {
        return std::any_cast<Data>(getRawData());
    }

    static std::string covarianceTypeInString(CovarianceType covarianceType) {
        static const std::map<CovarianceType, std::string> mapOfMsgCovarianceType = {
          {CovarianceType::UNKNOWN, "unknown"},
          {CovarianceType::APPROXIMATED, "approximated"},
          {CovarianceType::DIAGONAL_KNOWN, "diagonal known"},
          {CovarianceType::KNOWN, "known"},
        };
        const auto it = mapOfMsgCovarianceType.find(covarianceType);
        return (it != mapOfMsgCovarianceType.end()) ? it->second : "UNKNOWN";
    }
};

template<class StreamType>
StreamType& operator<<(StreamType& stream, const GpsPosition::NavigationSatellite& navigationSatellite) {
    stream << std::make_tuple(static_cast<int>(navigationSatellite.status), navigationSatellite.service);
    return stream;
}

template<class StreamType>
StreamType& operator<<(StreamType& stream, const GpsPosition& gpsPosition) {
    const auto data = gpsPosition.getData();
    const auto coordinates = std::make_tuple(data.latitude, data.longitude, data.altitude);
    const auto covarianceType = static_cast<unsigned>(data.positionCovarianceType);
    stream << "Header: " << data.header << ", Satellite: " << data.navigationSatellite << ", Coordinates: " << coordinates
           << ", Covariance: " << data.positionCovariance << ", Covariance Type: " << std::make_tuple(covarianceType);
    return stream;
}