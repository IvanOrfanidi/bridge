## Services

| Service                          | Path                            | RAW Type           | Payload                                                                  |
|:---------------------------------|:--------------------------------|:-------------------|:-------------------------------------------------------------------------|
| SerialNumber                     | serial-number                   | string             | Device Serial No. in string                                              |
| ---                              | ---                             | ---                | ---                                                                      |
| CameraTaskGetEV                  | camera-task-get-ev              | struct             | **[CameraTaskGetEvData](#camerataskgetevdata)**                          |
| ---                              | ---                             | ---                | ---                                                                      |
| CameraTaskGetISO                 | camera-task-get-iso             | struct             | **[CameraTaskGetIsoData](#camerataskgetisodata)**                        |
| ---                              | ---                             | ---                | ---                                                                      |
| GetCameraOpticalZoomFactor       | camera-optical-zoom-factor      | struct             | **[GetCameraOpticalZoomFactorData](#getcameraopticalzoomfactordata)**    |
| ---                              | ---                             | ---                | ---                                                                      |
| CameraTaskGetAperture            | camera-task-set-aperture        | struct             | **[CameraTaskGetApertureData](#camerataskgetaperturedata)**              |
| ---                              | ---                             | ---                | ---                                                                      |
| GetWholeBatteryInfo              | get-whole-battery-battery-info  | struct             | **[WholeBatteryInfoData](#wholebatteryinfodata)**                        |
| ---                              | ---                             | ---                | ---                                                                      |
| GetSingleBatteryDynamicInfo      | get-single-battery-dynamic-info | struct             | **[SingleBatteryDynamicInfoData](#singlebatterydynamicinfodata)**        |
| ---                              | ---                             | ---                | ---                                                                      |
| LoggingInfo                      | log                             | struct             | **[LoggingInfoData](#logginginfo)**                                      |
| ---                              | ---                             | ---                | ---                                                                      |


### CameraTaskGetEvData
| Data                             | RAW Type           |
|:---------------------------------|:-------------------|
| payload_index                    | uint8_t            |
| exposure_compensation            | uint8_t            |
| ---                              | ---                |

### CameraTaskGetIsoData
| Data                             | RAW Type           |
|:---------------------------------|:-------------------|
| payload_index                    | uint8_t            |
| iso_data                         | uint8_t            |
| ---                              | ---                |

### GetCameraOpticalZoomFactorData
| Data                             | RAW Type           |
|:---------------------------------|:-------------------|
| payload_index                    | uint8_t            |
| factor                           | double             |
| ---                              | ---                |

### CameraTaskGetApertureData
| Data                             | RAW Type           |
|:---------------------------------|:-------------------|
| payload_index                    | uint8_t            |
| aperture                         | uint8_t            |
| ---                              | ---                |

### WholeBatteryInfoData
| Data                             | RAW Type           | Payload                                    |
|:---------------------------------|:-------------------|:-------------------------------------------|
| info                             | struct             | **[BatteryInfoData](#batteryinfodata)**    |
| state                            | struct             | **[BatteryStateData](#batterystatedata)**  |
| ---                              | ---                | ---                                        |

### SingleBatteryDynamicInfoData
| Data                             | RAW Type           | Payload                                                     |
|:---------------------------------|:-------------------|:------------------------------------------------------------|
| info                             | struct             | **[SingleBatteryDynamicInfo](#singlebatterydynamicinfo)**   |
| state                            | struct             | **[SingleBatteryDynamicState](#singlebatterydynamicstate)** |
| ---                              | ---                | ---                                                         |

### LoggingInfo
| Data                             | RAW Type           | Payload                                                     |
|:---------------------------------|:-------------------|:------------------------------------------------------------|
| current_date_time                | string             | Data-time in *YYYY-MM-DDTHH:MM:SS* where T is separator     |
|                                  |                    | Data-time the message was sent                              |
| entries                          | array              | **[Entries](#entries)**                                     |
| ---                              | ---                | ---                                                         |

#### BatteryInfoData
| Data                                   | RAW Type           |
|:---------------------------------------|:-------------------|
| remain_fly_time                        | uint16_t           |
| go_home_need_time                      | uint16_t           |
| go_home_need_capacity                  | uint16_t           |
| land_need_capacity                     | uint16_t           |
| safe_fly_radius                        | float              |
| capacity_consume_speed                 | float              |
| go_home_count_down_state               | uint8_t            |
| go_home_count_down_value               | uint8_t            |
| voltage                                | uint16_t           |
| battery_capacity_percentage            | uint8_t            |
| low_battery_alarm_threshold            | uint8_t            |
| low_battery_alarm_enable               | uint8_t            |
| serious_low_battery_alarm_threshold    | uint8_t            |
| serious_low_battery_alarm_enable       | uint8_t            |
| ---                                    | ---                |

#### BatteryStateData
| Data                                   | RAW Type           |
|:---------------------------------------|:-------------------|
| voltage_not_safety                     | uint8_t            |
| very_low_voltage_alarm                 | uint8_t            |
| low_voltage_alarm                      | uint8_t            |
| serious_low_capacity_alarm             | uint8_t            |
| low_capacity_alarm                     | uint8_t            |
| ---                                    | ---                |

#### SingleBatteryDynamicInfo
| Data                                   | RAW Type           |
|:---------------------------------------|:-------------------|
| battery_index                          | uint8_t            |
| current_voltage                        | int32_t            |
| current_electric                       | int32_t            |
| full_capacity                          | uint32_t           |
| remained_capacity                      | uint32_t           |
| battery_temperature                    | int16_t            |
| cell_count                             | uint8_t            |
| battery_capacity_percent               | uint8_t            |
| sop                                    | uint8_t            |
| ---                                    | ---                |

#### SingleBatteryDynamicState
| Data                                   | RAW Type           |
|:---------------------------------------|:-------------------|
| cell_break                             | uint8_t            |
| self_check_error                       | uint8_t            |
| battery_closed_reason                  | uint8_t            |
| bat_soh_state                          | uint8_t            |
| max_cycle_limit                        | uint8_t            |
| has_cell_break                         | uint8_t            |
| heat_state                             | uint8_t            |
| ---                                    | ---                |

#### Entries
| Data                             | RAW Type           | Payload                                                     |
|:---------------------------------|:-------------------|:------------------------------------------------------------|
| date_time                        | string             | Data-time in *YYYY-MM-DDTHH:MM:SS* where T is separator     |
|                                  |                    | Data-time the message was created                           |
| level                            | string             | Levels:                                                     |
|                                  |                    | *Information*                                               |
|                                  |                    | *Warning*                                                   |
|                                  |                    | *Error*                                                     |
|                                  |                    | *Debug*                                                     |
| category                         | string             | Category: **[ *node name* ] [ *function* ]**                |
| message                          | string             | Log message                                                 |
| ---                              | ---                | ---                                                         |