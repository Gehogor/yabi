/*
 *__     __        ____  _____
 *\ \   / / /\    |  _ \|_   _|
 * \ \_/ / /  \   | |_) | | |
 *  \   / / /\ \  |  _ <  | |
 *   | | / ____ \ | |_) |_| |_
 *   |_|/_/    \_\|____/|_____|
 *
 * File:   main.h
 * Author: Yann Alonso and Jerome Chemouny
 *
 * Created on 25 avril 2016, 19:01
 */

#ifndef _MAIN_H_
#define	_MAIN_H_

// PIC management
#include <adc10.h>

// Timer parameters ----------------------------------------------------------//
#define T1_FREQ     10000   // 10kHz
#define PR_T1       2938    // 10kHz

#define T2_FREQ     10000   // 10kHz
#define PR_T2       2938    // 10kHz

#define PR_T3       1474    // 20kHz

// PWM parameters ------------------------------------------------------------//
#define PWM_MAX         1480
#define HALF_PWM_MAX    PWM_MAX/2

// IOs parameters ------------------------------------------------------------//
#define DRIVER_MODE     _LATE1
#define DRIVER_DIR      _LATE2
#define DRIVER_COAST	_LATE3
#define DRIVER_TACHO	_RD1
#define DRIVER_DIRO     _RE4
#define DRIVER_FF1      _RC13
#define DRIVER_FF2      _RC14
#define DRIVER_ALIMP	_RE4

#define ENCODER_PA      _RB4
#define ENCODER_PB      _RB5
#define ENCODER_PZ      _RB3

#define SDI_SCK         _RE8
#define SDI_SDI         _RF2
#define SDI_SDO         _RF3
#define SDI_SS          _RB2

// Led managmement -----------------------------------------------------------//
#define LED             _LATE5

// Functions -----------------------------------------------------------------//
void initIOs();
void initTimer();
void initSPI();
void initPWM();
void initADC();
void initInterruptFromEncoderSensor();
void initInterruptFromHallSensor();
void initDriver(void);

void process_LED();
void process_current();
void process_mode();
unsigned char process_SPI();
void process_loop();

unsigned char process_SPI_target();
unsigned char process_SPI_modeRead();
unsigned char process_SPI_modeWrite();
unsigned char process_SPI_PID_read();
unsigned char process_SPI_PID_write();
unsigned char process_SPI_positionWrite();
unsigned char process_SPI_positionLafErrorRead();
unsigned char process_SPI_positionLafErrorWrite();
unsigned char process_SPI_loopFrequencyRead();
unsigned char process_SPI_loopFrequencyWrite();
unsigned char process_SPI_busFrequencyRead();
unsigned char process_SPI_busFrequencyWrite();

// Interrupt functions -------------------------------------------------------//
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _T2Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _SPI1Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _QEIInterrupt(void);
void __attribute__((interrupt, no_auto_psv)) _INT2Interrupt(void);

#endif	/* _MAIN_H_ */
