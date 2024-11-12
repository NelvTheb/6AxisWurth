/*
 * 6axis.h
 *
 *  Created on: Nov 12, 2024
 *      Author: nelven
 */

#ifndef INC_6AXIS_H_
#define INC_6AXIS_H_

#include <stdio.h>
#include <stdint.h>

#define SENSOR_ADDRESS  0x6B << 1
#define CTRL1_XL 0x10
#define CTRL2_G  0x11
#define CTRL3_C  0x12
#define CTRL6_C  0x15
#define CTRL7_G  0x16
#define CTRL8_XL 0x17
#define STATUS_REG 0x1E
#define G_X_OUT_L 0x22             // Adresse du premier registre de sortie gyroscope (G_)
#define XL_X_OUT_L 0x28            // Adresse du premier registre de sortie accéléromètre (XL_)
// Sensibilités pour accéléromètre et gyroscope
#define ACC_SENSITIVITY_16G 0.488 // Sensibilité pour ±16g
#define GYRO_SENSITIVITY_2000DPS 70 // Sensibilité pour ±2000 dps

void check_device_communication();
void Init_HighPerf_Mode_6_axis(void);
void Display_6_axis_data(void);

#endif /* INC_6AXIS_H_ */
