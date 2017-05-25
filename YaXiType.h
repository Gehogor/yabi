/*
 *__     __        ____  _____
 *\ \   / / /\    |  _ \|_   _|
 * \ \_/ / /  \   | |_) | | |
 *  \   / / /\ \  |  _ <  | |
 *   | | / ____ \ | |_) |_| |_
 *   |_|/_/    \_\|____/|_____|
 *
 * File:   YaXiType.h
 * Author: Yann Alonso and Jérôme Chemouny
 *
 * Created on 25 may 2017
 */

#ifndef _YAXITYPE_H_
#define	_YAXITYPE_H_

// Inverter parameters -------------------------------------------------------//
#define TYPE_CARD 0x01  // Brushless invertor type.
#define IV_ENCODER  1
#define IV_TACHY   0

// Communication parameters --------------------------------------------------//
#define SPI_START           0x5E
#define SPI_END             0xE5
#define SPI_NO_DATA         0xF5
#define SPI_TARGET          0x01
#define SPI_MODE_READ       0x02
#define SPI_MODE_WRITE      0x03
#define SPI_PID_READ        0x04
#define SPI_PID_WRITE       0x05
#define SPI_POSITION_WRITE  0x06

// Global errors -------------------------------------------------------------//
#define NO_ERROR        0x00
#define SPI_UNCKNOW     0xF6
#define SPI_ERROR_DATA  0xF7

// Mode of state machine -----------------------------------------------------//
#define STOP    0x00
#define OPEN    0x01
#define LOOP    0x02

// 32 bits signed management -------------------------------------------------//

union S32_U8 {
    unsigned char c[4];
    long l;
};

// 16 bits unsigned management -----------------------------------------------//

union U16_U8 {
    unsigned char c[2];
    unsigned int i;
};

#endif	/* _YAXITYPE_H_ */
