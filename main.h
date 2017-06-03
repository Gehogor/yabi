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
//#define PR_T1   29500       // 1kHz
#define PR_T1       1474    // 20kHz
#define T1_FREQ     20000   // 20kHz

#define PR_T2       2938    // 10kHz
#define PR_T3       1474    // 20kHz
#define PR_T4       11480   // 10Hz

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

#define ENCODER_PA       _RB4
#define ENCODER_PB       _RB5
#define ENCODER_PZ       _RB3

#define SDI_SCK         _RE8
#define SDI_SDI         _RF2
#define SDI_SDO         _RF3
#define SDI_SS          _RB2

#define ADC_CSOUT       (ADC_CH0_POS_SAMPLEA_AN0 & ADC_CH0_NEG_SAMPLEA_NVREF)
#define ADC_REF         (ADC_CH0_POS_SAMPLEA_AN1 & ADC_CH0_NEG_SAMPLEA_NVREF)

// Led managmement -----------------------------------------------------------//
#define LED             _LATE5
#define LED_FREQ_0HZ    0
#define LED_FREQ_10HZ   2
#define LED_FREQ_5HZ    4
#define LED_FREQ_2HZ    10
#define LED_FREQ_1HZ    20

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

// Interrupt functions -------------------------------------------------------//
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _T2Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _T4Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _SPI1Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _QEIInterrupt(void);
void __attribute__((interrupt, no_auto_psv)) _INT2Interrupt(void);

#endif	/* _MAIN_H_ */
