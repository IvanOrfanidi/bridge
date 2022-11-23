## Data

see link ***http://wiki.ros.org/dji_sdk***

| Data                             |   Path                    | RAW Type                  | Format                                                                  | Limits      |
|:---------------------------------|:--------------------------|:--------------------------|:------------------------------------------------------------------------|:------------|
| GpsHealth                        | gps-health                | uint8_t                   | GPS signal health                                                       | 0 ... 5     |
|                                  |                           |                           | 0 - *VERY_BAD*                                                          |             |
|                                  |                           |                           | 1 - *VERY_WEAK*                                                         |             |
|                                  |                           |                           | 2 - *WEAK*                                                              |             |
|                                  |                           |                           | 3 - *GOOD*                                                              |             |
|                                  |                           |                           | 4 - *VERY_GOOD*                                                         |             |
|                                  |                           |                           | 5 - *VERY_STRONG*                                                       |             |
| ---                              | ---                       | ---                       | ---                                                                     | ---         |
| RemoteController                 | remote-controller         | uint8_t                   | RC connection status                                                    | 0 ... 1     |
|                                  |                           |                           | 0 - *DISCONNECTED*                                                      |             |
|                                  |                           |                           | 1 - *CONNECTED*                                                         |             |
| ---                              | ---                       | ---                       | ---                                                                     | ---         |
| BatteryState                     | battery-state             | struct                    | **[BatteryState::Data](#batterystatedata)**                             |             |
| ---                              | ---                       | ---                       | ---                                                                     | ---         |
| FlightStatus                     | flight-status             | uint8_t                   | Flight status                                                           | 0 ... 3     |
|                                  |                           |                           | 0 - *STOPPED*                                                           |             |
|                                  |                           |                           | 1 - *ON_GROUND*                                                         |             |
|                                  |                           |                           | 2 - *IN_AIR*                                                            |             |
| ---                              | ---                       | ---                       | ---                                                                     | ---         |
| HeightAboveTakeoff               | height-above-takeoff      | float                     | Height above takeoff location                                           | 0 ... max   |
| ---                              | ---                       | ---                       | ---                                                                     | ---         |
| RealTimeKinematic                | real-time-kinematic       | uint8_t                   | RTK connection status                                                   | 0 ... 1     |
|                                  |                           |                           | 0 - *DISCONNECTED*                                                      |             |
|                                  |                           |                           | 1 - *CONNECTED*                                                         |             |
| ---                              | ---                       | ---                       | ---                                                                     | ---         |
| DisplayMode                      | display-mode              | uint8_t                   | Display mode is detailed status of drone                                | 0 ... max   |
|                                  |                           |                           | 0 - *MODE_MANUAL*                                                       |             |
|                                  |                           |                           | 1 - *ATTITUDE*                                                          |             |
|                                  |                           |                           | ...                                                                     |             |
|                                  |                           |                           | 6 - *P_GPS*                                                             |             |
|                                  |                           |                           | ...                                                                     |             |
|                                  |                           |                           | 9 - *HOTPOINT*                                                          |             |
|                                  |                           |                           | 10 - *ASSISTED_TAKEOFF*                                                 |             |
|                                  |                           |                           | 11 - *AUTO_TAKEOFF*                                                     |             |
|                                  |                           |                           | 12 - *AUTO_LANDING*                                                     |             |
|                                  |                           |                           | ...                                                                     |             |
|                                  |                           |                           | 14 - *ON_MISSION*                                                       |             |
|                                  |                           |                           | 15 - *GO_HOME*                                                          |             |
|                                  |                           |                           | ...                                                                     |             |
|                                  |                           |                           | 17 - *SDK_CTRL*                                                         |             |
|                                  |                           |                           | 21 - *SPORT*                                                            |             |
|                                  |                           |                           | ...                                                                     |             |
|                                  |                           |                           | 31 - *SPORT_2*                                                          |             |
|                                  |                           |                           | ...                                                                     |             |
|                                  |                           |                           | 33 - *FORCE_AUTO_LANDING*                                               |             |
|                                  |                           |                           | ...                                                                     |             |
|                                  |                           |                           | 38 - *TRIPOD*                                                           |             |
|                                  |                           |                           | ...                                                                     |             |
|                                  |                           |                           | 40 - *SEARCH*                                                           |             |
|                                  |                           |                           | 41 - *ENGINE_START*                                                     |             |
|                                  |                           |                           | ...                                                                     |             |
|                                  |                           |                           | 51 - *SMART_TRACK*                                                      |             |
|                                  |                           |                           | 52 - *THREE_PROPELLER_EMERGENCY_LANDING*                                |             |
| ---                              | ---                       | ---                       | ---                                                                     | ---         |
| GpsPosition                      | gps-position              | struct                    | **[GpsPosition::Data](#gpspositiondata)**                               |             |
| ---                              | ---                       | ---                       | ---                                                                     | ---         |
| Attitude                         | attitude                  | struct                    | **[Attitude::Data](#attitudedata)**                                     |             |
| ---                              | ---                       | ---                       | ---                                                                     | ---         |
| FlightAnomaly                    | flight-anomaly            | uint32_t                  | Flight anomaly                                                          | 0 ... 8191  |
|                                  |                           |                           | 1 - *IMPACT_IN_AIR*                (0 bit)                              |             |
|                                  |                           |                           | 2 - *RANDOM_FLY*                   (1 bit)                              |             |
|                                  |                           |                           | 4 - *VERTICAL_CONTROL_FAIL*        (2 bit)                              |             |
|                                  |                           |                           | 8 - *HORIZONTAL_CONTROL_FAIL*      (3 bit)                              |             |
|                                  |                           |                           | 16 - *YAW_CONTROL_FAIL*            (4 bit)                              |             |
|                                  |                           |                           | 32 - *AIRCRAFT_IS_FALLING*         (5 bit)                              |             |
|                                  |                           |                           | 64 - *STRONG_WIND_LEVEL1*          (6 bit)                              |             |
|                                  |                           |                           | 128 - *STRONG_WIND_LEVEL2*         (7 bit)                              |             |
|                                  |                           |                           | 256 - *COMPASS_INSTALLATION_ERROR* (8 bit)                              |             |
|                                  |                           |                           | 512 - *IMU_INSTALLATION_ERROR*     (9 bit)                              |             |
|                                  |                           |                           | 1024 - *ESC_TEMPERATURE_HIGH*      (10 bit)                             |             |
|                                  |                           |                           | 2048 - *ESC_DISCONNECTED*          (11 bit)                             |             |
|                                  |                           |                           | 4096 - *GPS_YAW_ERROR*             (12 bit)                             |             |
| ---                              | ---                       | ---                       | ---                                                                     | ---         |
| gimbalangle                      | gimbal-angle              | struct                    | **[GimbalAngle::Data](#gimbalangledata)**                               |             |
| ---                              | ---                       | ---                       | ---                                                                     | ---         |
| WaypointV2MissionState           | waypoint-v2-mission-state | struct                    | **[WaypointV2MissionState::Data](#waypointv2missionstatedata)**         |             |
| ---                              | ---                       | ---                       | ---                                                                     | ---         |

### BatteryState::Data
| Data                             | RAW Type                  | Format                                                                  | Limits      |
|:---------------------------------|:--------------------------|:------------------------------------------------------------------------|:------------|
| Header                           | struct                    | **[Header](#header)**                                                   |             |
| voltage                          | float                     | Voltage in V                                                            | 0 ... max   |
| current                          | float                     | Negative when discharging in A                                          | 0 ... max   |
| charge                           | float                     | Current charge in Ah                                                    | 0 ... max   |
| capacity                         | float                     | Capacity in Ah (last full capacity)                                     | 0 ... max   |
| designCapacity                   | float                     | Capacity in Ah (design capacity)                                        | 0 ... max   |
| percentage                       | float                     | Charge percentage                                                       | 0 ... 1     |
| ---                              | ---                       | ---                                                                     | ---         |
| powerSupplyStatus                | uint8_t                   | The charging status as reported                                         | 0 ... 4     |
|                                  |                           | 1 - *UNKNOWN*                                                           |             |
|                                  |                           | 1 - *CHARGING*                                                          |             |
|                                  |                           | 2 - *DISCHARGING*                                                       |             |
|                                  |                           | 3 - *NOT_CHARGING*                                                      |             |
|                                  |                           | 4 - *FULL*                                                              |             |
| ---                              | ---                       | ---                                                                     | ---         |
| powerSupplyHealth                | uint8_t                   | The battery health metric                                               | 0 ... 8     |
|                                  |                           | 0 - *UNKNOWN*                                                           |             |
|                                  |                           | 1 - *GOOD*                                                              |             |
|                                  |                           | 2 - *OVERHEAT*                                                          |             |
|                                  |                           | 3 - *DEAD*                                                              |             |
|                                  |                           | 4 - *OVERVOLTAGE*                                                       |             |
|                                  |                           | 5 - *UNSPEC_FAILURE*                                                    |             |
|                                  |                           | 6 - *COLD*                                                              |             |
|                                  |                           | 7 - *WATCHDOG_TIMER_EXPIRE*                                             |             |
|                                  |                           | 8 - *SAFETY_TIMER_EXPIRE*                                               |             |
| ---                              | ---                       | ---                                                                     | ---         |
| powerSupplyTechnology            | uint8_t                   | The battery chemistry                                                   | 0 ... 6     |
|                                  |                           | 1 - *UNKNOWN*                                                           |             |
|                                  |                           | 1 - *NIMH*                                                              |             |
|                                  |                           | 2 - *LION*                                                              |             |
|                                  |                           | 3 - *LIPO*                                                              |             |
|                                  |                           | 4 - *LIFE*                                                              |             |
|                                  |                           | 5 - *NICD*                                                              |             |
|                                  |                           | 6 - *LIMN*                                                              |             |
| ---                              | ---                       | ---                                                                     | ---         |
| percentage                       | uint8_t                   | True if the battery is present                                          | 0 ... 1     |
| ---                              | ---                       | ---                                                                     | ---         |
| location                         | string                    | Location the battery                                                    |             |
| ---                              | ---                       | ---                                                                     | ---         |
| serialNumber                     | string                    | Serial number the battery                                               |             |
| ---                              | ---                       | ---                                                                     | ---         |


### GpsPosition::Data
| Data                             | RAW Type                  | Format                                                                  | Limits      |
|:---------------------------------|:--------------------------|:------------------------------------------------------------------------|:------------|
| Header                           | struct                    | **[Header](#header)**                                                   |             |
| NavigationSatellite              | struct                    | **[GpsPosition::NavigationSatellite](#gpspositionnavigationsatellite)** |             |
| latitude                         | double                    | Latitude                                                                | min ... max |
| longitude                        | double                    | Longitude                                                               | min ... max |
| altitude                         | double                    | Altitude in metrs                                                       | min ... max |
| positionCovariance               | std::array<double, 9>     | Position covariance                                                     |             |
| positionCovarianceType           | uint8_t                   | Position covariance type                                                | 0 ... 3     |
|                                  |                           | 0 - *UNKNOWN*                                                           |             |
|                                  |                           | 1 - *APPROXIMATED*                                                      |             |
|                                  |                           | 2 - *DIAGONAL_KNOWN*                                                    |             |
|                                  |                           | 3 - *KNOWN*                                                             |             |
| ---                              | ---                       | ---                                                                     | ---         |

### GpsPosition::NavigationSatellite
| Data                             | RAW Type                  | Format                                                                  | Limits      |
|:---------------------------------|:--------------------------|:------------------------------------------------------------------------|:------------|
| status                           | int8_t                    | Status                                                                  | -1 ... 2    |
|                                  |                           | -1 - *NO_FIX*                                                           |             |
|                                  |                           | 0 - *FIX*                                                               |             |
|                                  |                           | 1 - *SBAS_FIX*                                                          |             |
|                                  |                           | 2 - *GBAS_FIX*                                                          |             |
| ---                              | ---                       | ---                                                                     | ---         |
| service                          | uint16_t                  | Service in bit mask                                                     | 0 ... 15    |
|                                  |                           | 1 - *GPS*       (0 bit)                                                 |             |
|                                  |                           | 2 - *GLONASS*   (1 bit)                                                 |             |
|                                  |                           | 4 - *COMPASS*   (2 bit)  includes BeiDou                                |             |
|                                  |                           | 8 - *GALILEO*   (3 bit)                                                 |             |
| ---                              | ---                       | ---                                                                     | ---         |

### Attitude::Data
| Data                             | RAW Type                  | Format                                                                  | Limits      |
|:---------------------------------|:--------------------------|:------------------------------------------------------------------------|:------------|
| Header                           | struct                    | **[Header](#header)**                                                   |             |
| Quaternion                       | struct                    | **[Quaternion](#quaternion)**                                           |             |
| ---                              | ---                       | ---                                                                     | ---         |

### GimbalAngle::Data
| Data                             | RAW Type                  | Format                                                                  | Limits      |
|:---------------------------------|:--------------------------|:------------------------------------------------------------------------|:------------|
| Header                           | struct                    | **[Header](#header)**                                                   |             |
| Vector3                          | struct                    | **[Vector3](#vector3)**                                                 |             |
| ---                              | ---                       | ---                                                                     | ---         |

### WaypointV2MissionState::Data
| Data                             | RAW Type                  | Format                                                                  | Limits      |
|:---------------------------------|:--------------------------|:------------------------------------------------------------------------|:------------|
| commonDataVersion                | int8_t                    | Common data version                                                     | min ... max |
| commonDataLen                    | uint16_t                  | Common data length                                                      | 0 ... max   |
| curWaypointIndex                 | uint16_t                  | Current waypoint index                                                  | 0 ... max   |
| state                            | uint8_t                   | State                                                                   | 0 ... max   |
| velocity                         | uint16_t                  | Velocity                                                                | 0 ... max   |
| ---                              | ---                       | ---                                                                     | ---         |

#### Header
| Data                             | RAW Type                  | Format                                                                  | Limits      |
|:---------------------------------|:--------------------------|:------------------------------------------------------------------------|:------------|
| sequence                         | uint32_t                  | Sequence ID                                                             | 0 ... max   |
| stamp                            | string                    | Data-time in *YYYY-MM-DDTHH:MM:SS* where T is separator                 |             |
| frameId                          | string                    | Frame this data is associated with                                      |             |
| ---                              | ---                       | ---                                                                     | ---         |

#### Quaternion
| Data                             | RAW Type                  | Format                                                                  | Limits      |
|:---------------------------------|:--------------------------|:------------------------------------------------------------------------|:------------|
| x                                | double                    | X-Axis                                                                  | min ... max |
| y                                | double                    | Y-Axis                                                                  | min ... max |
| z                                | double                    | Z-Axis                                                                  | min ... max |
| w                                | double                    | W-Axis                                                                  | min ... max |
| ---                              | ---                       | ---                                                                     | ---         |

#### Vector3
| Data                             | RAW Type                  | Format                                                                  | Limits      |
|:---------------------------------|:--------------------------|:------------------------------------------------------------------------|:------------|
| x                                | double                    | X-Axis                                                                  | min ... max |
| y                                | double                    | Y-Axis                                                                  | min ... max |
| z                                | double                    | Z-Axis                                                                  | min ... max |
| ---                              | ---                       | ---                                                                     | ---         |