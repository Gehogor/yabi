/*
 *__     __        ____  _____
 *\ \   / / /\    |  _ \|_   _|
 * \ \_/ / /  \   | |_) | | |
 *  \   / / /\ \  |  _ <  | |
 *   | | / ____ \ | |_) |_| |_
 *   |_|/_/    \_\|____/|_____|
 *
 * File:   YaXiType.h
 * Author: Yann Alonso and Jerome Chemouny
 *
 * Created on 25 may 2017
 */

#ifndef _YAXITYPE_H_
#define	_YAXITYPE_H_

// Inverter parameters -------------------------------------------------------//
#define TYPE_CARD   0x01  // Brushless invertor type.
#define IV_ENCODER  1
#define IV_TACHY    0

// Communication parameters --------------------------------------------------//
#define SPI_START           0x5E
#define SPI_END             0xE5
#define SPI_TARGET          0x01
#define SPI_MODE_READ       0x02
#define SPI_MODE_WRITE      0x03
#define SPI_PID_READ        0x04
#define SPI_PID_WRITE       0x05
#define SPI_POSITION_WRITE  0x06

#define SPI_NO_DATA         0x00
#define SPI_NO_ERROR        0x01
#define SPI_UNCKNOW         0xF6
#define SPI_ERROR_DATA      0xF7
#define SPI_MAX_SIZE        18

// Mode of state machine -----------------------------------------------------//
#define SIMULATOR   0x00
#define DRIVER_OPEN 0x01
#define OPEN_LOOP   0x02
#define CLOSE_LOOP  0x03
#define NO_SPI_COM  0xFE

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
    double value;
} D32;

/* SPI management ------------------------------------------------------------*/
typedef struct {
    volatile unsigned char index;
    unsigned char rx[SPI_MAX_SIZE];
    unsigned char tx[SPI_MAX_SIZE];
} Com_SPI;

/* Position interpolation ----------------------------------------------------*/
typedef struct {
    volatile unsigned long timer;
    long loopFrequency;
    long busFrequency;
    long stepPos;
} Interpolation;

// Watchdog managmement ------------------------------------------------------//
typedef struct
{
    volatile unsigned long timer;
    union U16_U8 frequency;
} Watchdog;

#endif	/* _YAXITYPE_H_ */
