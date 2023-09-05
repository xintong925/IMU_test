/*
 * Parameter_setting.h
 *
 *  Created on: Aug 24, 2023
 *      Author: tongx
 */

#ifndef PARAMETER_SETTING_H_
#define PARAMETER_SETTING_H_

#ifdef __cplusplus
extern "C" {
#endif


/* RF transmission */
#define PACKET_SIZE 12
#define NUM_SAMPLES 1
#define PAYLOAD_LENGTH 20

/* Sleep */
#define STANDBY_DURATION 3

/* Bit to read */
#define READ_BIT                  0x8000
#define TEMP_BIT                  0x0004
#define XL_BIT                    0x0001
#define G_BIT                     0x0002
#define ACTIVITY_BIT              0x0010

/* Configuration of XL, G and interrupt */
#define CTRL1_XL_VALUE           0x0050 //ODR_XL = 208 Hz, FS_XL = ±2 g
#define CTRL2_G_VALUE            0x0038 //ODR_G = 52 Hz, FS_G = ±1000 dps
#define WAKE_UP_DUR              0x0068 //last 4 bits: duration for inactivity detection - LSB*512/ODR_XL
#define WAKE_UP_THS              0x0002 // Threshold for wakeup (last 5 bits)
#define TAP_CFG0_VALUE           0x0000 // Select sleep-change notification
#define TAP_CFG2_VALUE           0x00E0 // Enable interrupt; set XL ODR to 12.5 Hz in LP, gyro to power down
#define MD1_CFG_VALUE            0x0080 // Routing activity/inactivity event on INT1 enabled


/* XL and G Value */
const float XL_SCALE_RANGE_2_G = 0.061;
const float XL_SCALE_RANGE_4_G = 0.122;
const float XL_SCALE_RANGE_8_G = 0.244;
const float XL_SCALE_RANGE_16_G = 0.488;

const float G_SCALE_RANGE_250_DPS = 8.75;
const float G_SCALE_RANGE_500_DPS = 17.5;
const float G_SCALE_RANGE_1000_DPS = 35.0;
const float G_SCALE_RANGE_2000_DPS = 70.0;




#ifdef __cplusplus
}
#endif

#endif /* PARAMETER_SETTING_H_ */
