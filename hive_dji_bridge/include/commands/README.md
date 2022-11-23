## Commands

| Command                          | RAW Type           | Payload                                                                          |
|:---------------------------------|:-------------------|:---------------------------------------------------------------------------------|
| flight_task_control              | struct             | **[FlightTaskControlSetting](#flighttaskcontrolsetting)**                        |
| ---                              | ---                | ---                                                                              |
| set_rtk_enable                   | uint8_t            | Set Real Time Kinematic                                                          |
| ---                              | ---                | ---                                                                              |
| set_simulation_on                | struct             | **[SimulationSetting](#simulationsetting)**                                      |
| ---                              | ---                | ---                                                                              |
| obtain_release_control_authority | uint8_t            | Obtain Control                                                                   |
| ---                              | ---                | ---                                                                              |
| gimbal_task_control              | struct             | **[GimbalTaskControlSetting](#gimbaltaskcontrolsetting)**                        |
| ---                              | ---                | ---                                                                              |
| set_camera_optical_zoom_factor   | struct             | **[CameraOpticalZoomFactorSetting](#cameraopticalzoomfactorsetting)**            |
| ---                              | ---                | ---                                                                              |
| camera_start_shoot_single_photo  | uint8_t            | Shoot Single Photo (Value: camera index, for H20 = 0)                            |
| ---                              | ---                | ---                                                                              |
| camera_record_video_action       | struct             | **[CameraRecordVideoSetting](#camerarecordvideosetting)**                        |
| ---                              | ---                | ---                                                                              |
| camera_task_set_ev               | struct             | **[CameraEvSetting](#cameraevsetting)**                                          |
| ---                              | ---                | ---                                                                              |
| camera_task_set_iso              | struct             | **[CameraIsoSetting](#cameraisosetting)**                                        |
| ---                              | ---                | ---                                                                              |
| camera_task_set_aperture         | struct             | **[CameraApertureSetting](#cameraaperturesetting)**                              |
| ---                              | ---                | ---                                                                              |
| camera_task_set_focus_point      | struct             | **[CameraFocusPointSetting](#camerafocuspointsetting)**                          |
| ---                              | ---                | ---                                                                              |
| camera_task_tap_zoom_point       | struct             | **[CameraTapZoomPointSetting](#cameratapzoompointsetting)**                      |
| ---                              | ---                | ---                                                                              |
| camera_task_zoom_ctrl            | struct             | **[CameraZoomCtrltSetting](#camerazoomctrltsetting)**                            |
| ---                              | ---                | ---                                                                              |
| change_camera_h264_source        | struct             | **[ChangeCameraH264SourceSetting](#changecamerah264sourcesetting)**              |
| ---                              | ---                | ---                                                                              |
| waypointv2_pausemission          | void               |                                                                                  |
| ---                              | ---                | ---                                                                              |
| waypointv2_resumemission         | void               |                                                                                  |
| ---                              | ---                | ---                                                                              |
| waypointv2_stopmission           | void               |                                                                                  |
| ---                              | ---                | ---                                                                              |
| waypointv2_startmission          | void               |                                                                                  |
| ---                              | ---                | ---                                                                              |
| flight_control_setpoint_generic  | struct             | **[FlightControlSetpointGenericSetting](#flightcontrolsetpointgenericsetting)**  |
| ---                              | ---                | ---                                                                              |


### FlightTaskControlSetting
| Data                             | RAW Type           | Format                                                                   |
|:---------------------------------|:-------------------|:-------------------------------------------------------------------------|
| task                             | uint8_t            | Set Flight Task                                                          |
|                                  |                    | 1 - *TASK_GOHOME*                                                        |
|                                  |                    | 2 - *TASK_POSITION_AND_YAW_CONTROL*                                      |
|                                  |                    | 3 - *TASK_GOHOME_AND_CONFIRM_LANDING*                                    |
|                                  |                    | 4 - *TASK_TAKEOFF*                                                       |
|                                  |                    | 5 - *TASK_VELOCITY_AND_YAWRATE_CONTROL*                                  |
|                                  |                    | 6 - *TASK_LAND*                                                          |
|                                  |                    | 7 - *START_MOTOR*                                                        |
|                                  |                    | 8 - *STOP_MOTOR*                                                         |
|                                  |                    | 9 - *TASK_EXIT_GO_HOME*                                                  |
|                                  |                    | 14 - *TASK_EXIT_LANDING*                                                 |
|                                  |                    | 30 - *TASK_FORCE_LANDING_AVOID_GROUND*                                   |
|                                  |                    | 31 - *TASK_FORCE_LANDING*                                                |
| joystick_command_x               | float              |                                                                          |
| joystick_command_y               | float              |                                                                          |
| joystick_command_z               | float              |                                                                          |
| joystick_command_yaw             | float              |                                                                          |
| velocity_control_time_ms         | uint32_t           |                                                                          |
| pos_threshold_in_m               | float              |                                                                          |
| yaw_threshold_in_deg             | float              |                                                                          |
| ---                              | ---                | ---                                                                      |


### SimulationSetting
| Data                             | RAW Type           |
|:---------------------------------|:-------------------|
| enable                           | uint8_t            |
| longitude                        | double             |
| lattitude                        | double             |
| ---                              | ---                |

### GimbalTaskControlSetting
| Data                             | RAW Type           |
|:---------------------------------|:-------------------|
| is_reset                         | uint8_t            |
| payload_index                    | uint8_t            |
| rotation_mode                    | uint8_t            |
| pitch                            | float              |
| roll                             | float              |
| yaw                              | float              |
| time                             | double             |
| reference                        | uint8_t            |
| atti_or_joint_ref                | uint8_t            |
| ---                              | ---                |

### CameraOpticalZoomFactorSetting
| Data                             | RAW Type           |
|:---------------------------------|:-------------------|
| payload_index                    | uint8_t            |
| factor                           | double             |
| ---                              | ---                |

### CameraRecordVideoSetting
| Data                             | RAW Type           |
|:---------------------------------|:-------------------|
| payload_index                    | uint8_t            |
| start_stop                       | uint8_t            |
| ---                              | ---                |

### CameraEvSetting
| Data                             | RAW Type           |
|:---------------------------------|:-------------------|
| payload_index                    | uint8_t            |
| exposure_mode                    | uint8_t            |
| exposure_compensation            | uint8_t            |
| ---                              | ---                |

### CameraIsoSetting
| Data                             | RAW Type           |
|:---------------------------------|:-------------------|
| payload_index                    | uint8_t            |
| exposure_mode                    | uint8_t            |
| iso_data                         | uint8_t            |
| ---                              | ---                |

### CameraApertureSetting
| Data                             | RAW Type           |
|:---------------------------------|:-------------------|
| payload_index                    | uint8_t            |
| exposure_mode                    | uint8_t            |
| aperture                         | uint8_t            |
| ---                              | ---                |

### CameraFocusPointSetting
| Data                             | RAW Type           |
|:---------------------------------|:-------------------|
| payload_index                    | uint8_t            |
| x                                | float              |
| y                                | float              |
| ---                              | ---                |

### CameraTapZoomPointSetting
| Data                             | RAW Type           |
|:---------------------------------|:-------------------|
| payload_index                    | uint8_t            |
| multiplier                       | uint8_t            |
| x                                | float              |
| y                                | float              |
| ---                              | ---                |

### CameraZoomCtrltSetting
| Data                             | RAW Type           |
|:---------------------------------|:-------------------|
| start_stop                       | uint8_t            |
| payload_index                    | uint8_t            |
| direction                        | uint8_t            |
| speed                            | uint8_t            |
| ---                              | ---                |

### ChangeCameraH264SourceSetting
| Data                             | RAW Type           |
|:---------------------------------|:-------------------|
| request_view                     | uint8_t            |
| source                           | uint8_t            |
| ---                              | ---                |

### FlightControlSetpointGenericSetting
| Data                             | RAW Type           |
|:---------------------------------|:-------------------|
| roll                             | float              |
| pitch                            | float              |
| yaw                              | float              |
| throttle                         | float              |
| [mode](#mode)                    | float              |
| gear                             | float              |
| ---                              | ---                |

##### mode
| Flight Control Flag in bit mask                                          |
|:-------------------------------------------------------------------------|
| 0 - *HORIZONTAL_ANGLE*                                                   |
| 64 - *HORIZONTAL_VELOCITY*                                               |
| 128 - *HORIZONTAL_POSITION*                                              |
| 192 - *HORIZONTAL_ANGULAR_RATE*                                          |
| 0 - *VERTICAL_VELOCITY*                                                  |
| 16 - *VERTICAL_POSITION*                                                 |
| 32 - *VERTICAL_THRUST*                                                   |
| 0 - *YAW_ANGLE*                                                          |
| 8 - *YAW_RATE*                                                           |
| 0 - *HORIZONTAL_GROUND*                                                  |
| 2 - *YAW_RATE*                                                           |
| 0 - *STABLE_DISABLE*                                                     |
| 1 - *STABLE_ENABLE*                                                      |
| ---                                                                      |

*HORIZONTAL_POSITION* - Horizontal angular rate is supported only by A3/N3 based platform and is NOT supported by M100
