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
#define SPI_NO_DATA         0x55
#define SPI_TARGET          0x01
#define SPI_MODE_READ       0x02
#define SPI_MODE_WRITE      0x03
#define SPI_PID_READ        0x04
#define SPI_PID_WRITE       0x05
#define SPI_POSITION_WRITE  0x06

// Global errors -------------------------------------------------------------//
#define SPI_UNCKNOW     0xF6
#define SPI_ERROR_DATA  0xF7

// Mode of state machine -----------------------------------------------------//
#define DRIVER_OPEN 0x00
#define OPEN_LOOP   0x01
#define CLOSE_LOOP  0x02

/* Data management -----------------------------------------------------------*/
union S32_U8 {
    unsigned char c[4];
    long l;
};

union U16_U8 {
    unsigned char c[2];
    unsigned int i;
};

typedef struct {
    union S32_U8 bus;
    volatile long value;
} SafeData_S32;

typedef struct {
    union S32_U8 bus;
    volatile double value;
} SafeData_D32;

typedef struct {
    unsigned char bus;
    volatile unsigned char value;
} SafeData_U8;

/* SPI management ------------------------------------------------------------*/
typedef struct {
    unsigned char functionCount;
    unsigned char index;
    unsigned char function;
} Com_SPI;

/* Position interpolation ----------------------------------------------------*/
typedef struct {
    unsigned int cyclic;
    unsigned int index;
} Interpolation;

#endif	/* _YAXITYPE_H_ */
