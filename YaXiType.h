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
#define SPI_START                   0x5E
#define SPI_END                     0xE5
#define SPI_RESET                   0x00
#define SPI_TARGET                  0x01
#define SPI_MODE_READ               0x02
#define SPI_MODE_WRITE              0x03
#define SPI_PID_READ                0x04
#define SPI_PID_WRITE               0x05
#define SPI_POSITION_WRITE          0x06
#define SPI_POS_LAG_ERROR_READ      0x07
#define SPI_POS_LAG_ERROR_WRITE     0x08
#define SPI_LOOP_FREQUENCY_READ     0x09
#define SPI_LOOP_FREQUENCY_WRITE    0x0A
#define SPI_BUS_FREQUENCY_READ      0x0B
#define SPI_BUS_FREQUENCY_WRITE     0x0C
#define SPI_MAX_SIZE                19

// Errors management ---------------------------------------------------------//
#define ALL_OK          0x00
#define SPI_DATA_ERROR  0xF0
#define SPI_UNCKNOW     0xF1
#define POS_LAG_ERROR   0xF2
#define SPI_WATCHDOG    0xF3

// Mode of state machine -----------------------------------------------------//
#define SIMULATOR       0x00
#define DRIVER_OPEN     0x01
#define OPEN_LOOP       0x02
#define CLOSE_LOOP      0x03
#define ERROR_DRIVER    0x04

/* Data management -----------------------------------------------------------*/
union S32_U8 {
    unsigned char c[4];
    long l;
};

union U16_U8 {
    unsigned char c[2];
    unsigned int i;
};

union S16_U8 {
    unsigned char c[2];
    int i;
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

/* Close loop interpolation management ---------------------------------------*/
typedef struct {
    unsigned long timer;
    volatile union U16_U8 loopFrequency;
    volatile union U16_U8 busFrequency;
    volatile long stepPos;
    volatile long currentTargetPos;
} Interpolation;

/* Axis managmement ----------------------------------------------------------*/
typedef struct {
    volatile union S32_U8 pos;
    volatile union S32_U8 speed;
    volatile union S32_U8 accel;
    union S32_U8 targetPos;
    union S32_U8 targetSpeed;
    volatile union S32_U8 posLagErrorMax;
    volatile char direction;
} Axis;

/* Watchdog managmement ------------------------------------------------------*/
typedef struct {
    volatile unsigned long timer;
    union U16_U8 frequency;
} Watchdog;

/* PID managmement -----------------------------------------------------------*/
typedef struct {
    volatile D32 kp;
    volatile D32 ki;
    volatile D32 kd;
} PID;

/* Current management --------------------------------------------------------*/
typedef struct {
    volatile union S16_U8 value;
    union U16_U8 frequency;
    unsigned char count;

    volatile unsigned long timer;
    unsigned char state;
    long measure;
    long average;
} Current;

/* Led managmement -----------------------------------------------------------*/
typedef struct {
    volatile unsigned long timer;
    unsigned long frequency_A;
    unsigned long frequency_B;
} Led;

#endif	/* _YAXITYPE_H_ */
