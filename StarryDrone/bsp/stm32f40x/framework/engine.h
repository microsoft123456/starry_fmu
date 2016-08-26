/*
 * File      : engine.h
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2016-07-01     zoujiachi   	the first version
 */
 
#ifndef __ENGINE_H__
#define __ENGINE_H__

#include <rtthread.h>
#include <rtdevice.h>

#define PROTOCOL_VERSION 0 // debug

#define PROTOCOL_START_0                            0x55
#define PROTOCOL_START_1                            0xAA

#define PROTOCOL_RETURN_ATTITUDE_QUATERNION         0x01
#define PROTOCOL_CALIBRATION_RETURN_RAW_AH          0x02
#define PROTOCOL_PERIOD_MARK                        0x03
#define PROTOCOL_RETURN_POSITION                    0x04
#define PROTOCOL_LOCK_ATTITUDE_SET_ATTITUDE         0x05
#define PROTOCOL_GET_ATTITUDE_QUATERNION            0x06
#define PROTOCOL_HEARTBEAT_SEND                     0x07
#define PROTOCOL_HEARTBEAT_RETURN                   0x08
#define PROTOCOL_GET_POSITION                       0x09
#define PROTOCOL_GET_ATTITUDE_ANGLE                 0x0A
#define PROTOCOL_RETURN_ATTITUDE_ANGLE              0x0B
#define PROTOCOL_LOCK_ATTITUDE_CURRENT_AS_TARGET    0x0C
#define PROTOCOL_LOCK_ATTITUDE_ROTATE_WORLD_TIME    0x0D
#define PROTOCOL_LOCK_ATTITUDE_SET_THROTTLE_BASE    0x0E
#define PROTOCOL_SET_PARAMETER                      0x0F
#define PROTOCOL_GET_PARAMETER                      0x10
#define PROTOCOL_RETURN_PARAMETER                   0x11
#define PROTOCOL_SET_CONTROL_MODE                   0x12
#define PROTOCOL_LOCK_THROTTLE_SET_THROTTLE         0x13
#define PROTOCOL_GET_VECTOR                         0x14
#define PROTOCOL_RETURN_VECTOR                      0x15
#define PROTOCOL_RETURN_OSC                         0x16
#define PROTOCOL_LOCK_ATTITUDE_SET_TARGET           0x17
#define PROTOCOL_GET_STATUS                         0x18
#define PROTOCOL_RETURN_STATUS                      0x19
#define PROTOCOL_RETURN_MESSAGE                     0x1A
#define PROTOCOL_BOOTLOADER_CMD                     0x1B
#define PROTOCOL_BOOTLOADER_STATUS                  0x1C
#define PROTOCOL_PARAMETER                          0x1D
#define PROTOCOL_LOCK_ATTITUDE                      0x1E

#define PROTOCOL_PARAMETER_ATTITUDE_CONTROL_P       0x01
#define PROTOCOL_PARAMETER_ATTITUDE_CONTROL_D       0x02
#define PROTOCOL_PARAMETER_ATTITUDE_CONTROL_I       0x03

#define PROTOCOL_SET_CONTROL_MODE_LOCK_THROTTLE     0x01
#define PROTOCOL_SET_CONTROL_MODE_LOCK_ATTITUDE     0x02
#define PROTOCOL_SET_CONTROL_MODE_HALT              0x03

#define PROTOCOL_VECTOR_TYPE_RAW_ACC                0x01
#define PROTOCOL_VECTOR_TYPE_CALIBRATED_ACC         0x02
#define PROTOCOL_VECTOR_TYPE_RAW_GYR                0x03
#define PROTOCOL_VECTOR_TYPE_CALIBRATED_GYR         0x04
#define PROTOCOL_VECTOR_TYPE_RAW_MAG                0x05
#define PROTOCOL_VECTOR_TYPE_CALIBRATED_MAG         0x06

#define PROTOCOL_STATUS_BATERY_VOLTAGE              0x01
#define PROTOCOL_STATUS_CURRENT_THROTTLE            0x02

#define PROTOCOL_BOOTLOADER_CMD_ENTER_BOOTLOADER    0x01
#define PROTOCOL_BOOTLOADER_CMD_BEGIN_REFRESH_APP   0x02
#define PROTOCOL_BOOTLOADER_CMD_APP_LENGTH_CRC      0x03
#define PROTOCOL_BOOTLOADER_CMD_WRITE_APP           0x04
#define PROTOCOL_BOOTLOADER_CMD_QUEST_REMAIN_BLOCK  0x05
#define PROTOCOL_BOOTLOADER_CMD_ENTER_APP           0x06

#define PROTOCOL_BOOTLOADER_STATUS_STAY_AT_BOOTLOADER   0x01
#define PROTOCOL_BOOTLOADER_STATUS_READY_TO_REFRESH_APP 0x02
#define PROTOCOL_BOOTLOADER_STATUS_WRITING_APP          0x03
#define PROTOCOL_BOOTLOADER_STATUS_BLOCK_REMAIN         0x04
#define PROTOCOL_BOOTLOADER_STATUS_APP_CRC_ERROR        0x05
#define PROTOCOL_BOOTLOADER_STATUS_WRITE_APP_SUCCEED    0x06
#define PROTOCOL_BOOTLOADER_STATUS_RUNNING_APP          0x07

#define PROTOCOL_PARAMETER_GET_RAM        0x11
#define PROTOCOL_PARAMETER_RETURN_RAM     0x12
#define PROTOCOL_PARAMETER_SET_RAM        0x13
#define PROTOCOL_PARAMETER_OPERATE_RAM    0x14
#define PROTOCOL_PARAMETER_GET_FLASH      0x21
#define PROTOCOL_PARAMETER_RETURN_FLASH   0x22
#define PROTOCOL_PARAMETER_WRITE_FLASH    0x23
#define PROTOCOL_PARAMETER_FLASH_VERIFY   0x24
#define PROTOCOL_PARAMETER_RESET_FLASH    0x25

#define PROTOCOL_PARAMETER_OPERATE_RAM_MIX_HORIZONTAL_ATTITUDE 0x01

#define PROTOCOL_LOCK_ATTITUDE_GET_READY    0x01
#define PROTOCOL_LOCK_ATTITUDE_READY        0x02
#define PROTOCOL_LOCK_ATTITUDE_NOT_READY    0x03
#define PROTOCOL_LOCK_ATTITUDE_CONTROL      0x04

int32_t engine_init(void);
void    engine_checkEvent(void);
int32_t engine_handlePacket_serial(const uint8_t * packet,int32_t length);
int32_t engine_handlePacket_nrf(const uint8_t * packet,int32_t length);
int32_t engine_transmitBegin(uint8_t packetType);
int32_t engine_transmitContent(const void * part,int32_t length);
int32_t engine_transmitEnd(void);
int32_t engine_maxTransmitContentSize(void);

#endif