/*
 *__     __        ____  _____
 *\ \   / / /\    |  _ \|_   _|
 * \ \_/ / /  \   | |_) | | |
 *  \   / / /\ \  |  _ <  | |
 *   | | / ____ \ | |_) |_| |_
 *   |_|/_/    \_\|____/|_____|
 *
 * File:   main.c
 * Author: Yann Alonso and Jerome Chemouny
 *
 * Created on 25 avril 2016, 19:01
 */

//---------------//
#include "main.h"//
//---------------//

// PIC management
#include <libpic30.h>
#include <outcompare.h>
#include <p30F4012.h>
#include <ports.h>
#include <qei.h>
#include <spi.h>
#include <timer.h>
#include <xc.h>

// Standard library
#include <stdio.h>
#include <math.h>

// Current project
#include "Config.h"
#include "YaXiType.h"

// Axis parameters -----------------------------------------------------------//
Axis g_axis = {
    .pos.l = 0,
    .speed.l = 0,
    .accel.l = 0,
    .targetPos.l = 0,
    .targetSpeed.l = 0,
    .posLagErrorMax.l = 0
};

volatile long g_encoderU16 = 0;
volatile long g_hallUnit = 0;

volatile unsigned char g_mode = SIMULATOR;
volatile unsigned char g_error = SIMULATOR;

PID g_pid = {
    .kp.value = 0.0,.kp.bus.l = 0,
    .ki.value = 0.0,.ki.bus.l = 0,
    .kd.value = 0.0,.kd.bus.l = 0
};

Led g_led = {.timer = 0,.frequency_A = 10,.frequency_B = 10};

Current g_current = {
    .value.i = 0,
    .frequency.i = 1000,
    .count = 5,
    .timer = 0,
    .state = 0,
    .measure = 0,
    .average = 0
};

Com_SPI g_spi = {.index = 0};

Interpolation g_ctrl = {
    .timer = 0,
    .loopFrequency.i = 2000,
    .busFrequency.i = 200,
    .stepPos = 0,
    .currentTargetPos = 0
};

Watchdog g_wd = {.timer = 0,.frequency.i = 100};

/**
 * Main function, manage all initialization and continuous process.
 * @return 0 at the end of program.
 */
int main( )
{
    unsigned char spiState = 0;

    initIOs();
    initTimer();
    initSPI();
    initPWM();
    initADC();
    initInterruptFromEncoderSensor();
    initInterruptFromHallSensor();
    initDriver();

    while(1)
    {
        process_LED();
        process_current();

        // SPI buffer management, if SPI error is occured we select the safe mode.
        spiState = process_SPI();
        if(spiState != ALL_OK)
        {
            g_mode = ERROR_DRIVER;
            g_error = spiState;
        }

        // If watchdog is timout, we select the safe mode.
        if(g_wd.timer >= (T2_FREQ / g_wd.frequency.i))
        {
            g_mode = ERROR_DRIVER;
            g_error = SPI_WATCHDOG;
            g_spi.index = 0;
        }

        // Manage the driver mode functionning the card state.
        process_mode();
    }

    CloseTimer1();
    CloseTimer2();
    CloseTimer3();
    CloseTimer4();
    CloseOC1();
    CloseADC10();
    CloseQEI();
    return 0;
}

void initIOs( )
{
    ADPCFG = 0xFFFC;

    // Digital I/O configuration.
    PORTB = 0;
    LATB = 0;
    TRISB = 0xFFFF;

    PORTC = 0;
    LATC = 0;
    TRISC = 0xFFFF;

    PORTD = 0;
    LATD = 0;
    TRISD = 0xFFFE;

    PORTE = 0;
    LATE = 0;
    TRISE = 0xFF11;

    PORTF = 0;
    LATF = 0;
    TRISF = 0xFFFB;
}

void initTimer( )
{
    // Timer1 configuration (1khz).
    ConfigIntTimer1(T1_INT_PRIOR_5 & T1_INT_ON);
    WriteTimer1(0);
    OpenTimer1(T1_ON
               & T1_IDLE_STOP
               & T1_GATE_OFF
               & T1_PS_1_1
               & T1_SYNC_EXT_OFF
               & T1_SOURCE_INT
               ,
               PR_T1);

    // Timer2 configuration (10khz).
    ConfigIntTimer2(T2_INT_PRIOR_3 & T2_INT_ON);
    WriteTimer2(0);
    OpenTimer2(T2_ON
               & T2_IDLE_STOP
               & T2_GATE_OFF
               & T2_PS_1_1
               & T2_SOURCE_INT
               ,
               PR_T2);
}

void initSPI( )
{
    CloseSPI1();
    ConfigIntSPI1(SPI_INT_EN & SPI_INT_PRI_7);
    OpenSPI1(FRAME_ENABLE_OFF
             & FRAME_SYNC_INPUT
             & ENABLE_SDO_PIN
             & SPI_MODE16_OFF
             & SPI_SMP_OFF
             & SPI_CKE_OFF
             & SLAVE_ENABLE_ON
             & MASTER_ENABLE_OFF
             & CLK_POL_ACTIVE_HIGH
             & SEC_PRESCAL_3_1
             & PRI_PRESCAL_64_1
             ,
             SPI_ENABLE
             & SPI_IDLE_CON
             & SPI_RX_OVFLOW_CLR);
    _SPIROV = 0;
}

void initPWM( )
{
    OpenTimer3(T3_ON
               & T3_IDLE_STOP
               & T3_GATE_OFF
               & T3_PS_1_1
               & T3_SOURCE_INT
               ,
               PR_T3);
    ConfigIntOC1(OC_INT_OFF & OC_INT_PRIOR_4);
    OpenOC1(OC_IDLE_CON
            & OC_TIMER3_SRC
            & OC_PWM_FAULT_PIN_DISABLE
            ,0,0);
    SetDCOC1PWM(0);
}

void initADC( )
{
    ADCON1bits.ADON = 0;
    SetChanADC10(ADC_CH0_POS_SAMPLEA_AN0 & ADC_CH0_NEG_SAMPLEA_NVREF);
    ConfigIntADC10(ADC_INT_DISABLE);
    OpenADC10(ADC_MODULE_ON
              & ADC_IDLE_CONTINUE
              & ADC_FORMAT_INTG
              & ADC_CLK_MANUAL
              & ADC_AUTO_SAMPLING_OFF
              & ADC_SAMPLE_INDIVIDUAL
              & ADC_SAMP_OFF
              ,
              ADC_VREF_AVDD_AVSS
              & ADC_SCAN_OFF
              & ADC_CONVERT_CH0
              & ADC_SAMPLES_PER_INT_1
              & ADC_ALT_BUF_OFF
              & ADC_ALT_INPUT_OFF
              ,
              ADC_SAMPLE_TIME_0
              & ADC_CONV_CLK_SYSTEM
              & ADC_CONV_CLK_3Tcy
              ,
              ENABLE_AN0_ANA & ENABLE_AN1_ANA
              ,
              SKIP_SCAN_AN2 & SKIP_SCAN_AN3 & SKIP_SCAN_AN4 & SKIP_SCAN_AN5);
}

void initInterruptFromEncoderSensor( )
{
    ConfigIntQEI(QEI_INT_PRI_6 & QEI_INT_ENABLE);
    POSCNT = 0;
    MAXCNT = 0x7FFF;
    OpenQEI(QEI_IDLE_CON
            & QEI_INT_CLK
            & QEI_INDEX_RESET_DISABLE
            & QEI_CLK_PRESCALE_1
            & QEI_GATED_ACC_DISABLE
            & QEI_INPUTS_NOSWAP
            & QEI_MODE_x4_MATCH
            & QEI_DIR_SEL_CNTRL
            ,
            POS_CNT_ERR_INT_DISABLE
            & QEI_QE_CLK_DIVIDE_1_4
            & QEI_QE_OUT_ENABLE
            & MATCH_INDEX_PHASEA_HIGH
            & MATCH_INDEX_PHASEB_HIGH);
}

void initInterruptFromHallSensor( )
{
    ConfigINT2(RISING_EDGE_INT
               & EXT_INT_ENABLE
               & EXT_INT_PRI_6
               & GLOBAL_INT_ENABLE);
}

void initDriver( void )
{
    // Select fast-decay mode, better for start, stop and positioning
    // application.
    DRIVER_MODE = 0;

    // Turn all FETs off.
    DRIVER_COAST = 0;

    // Choose a default direction.
    DRIVER_DIR = 1;
}

void process_LED( )
{
    if(g_led.timer >= (T2_FREQ / g_led.frequency_A) && LED == 1)
    {
        g_led.timer = 0;
        LED = 0;
    }
    else if(g_led.timer >= (T2_FREQ / g_led.frequency_B) && LED == 0)
    {
        g_led.timer = 0;
        LED = 1;
    }
}

void process_current( )
{
    if(g_current.state == 0 && g_current.timer >= (T2_FREQ / g_current.frequency.i))
    {
        ADCON1bits.SAMP = 1;
        g_current.state = 1;
    }
    else if(g_current.state == 1 && ADCON1bits.SAMP)
    {
        ConvertADC10();
        g_current.state = 2;
    }
    else if(g_current.state == 2 && ADCON1bits.DONE == 1)
    {
        if(g_axis.direction == 1)
            g_current.average += ReadADC10(0);
        else
            g_current.average -= ReadADC10(0);

        g_current.measure++;
        g_current.state = 0;
    }

    if(g_current.measure >= g_current.count)
    {
        g_current.value.i = g_current.average / g_current.measure;

        g_current.measure = 0;
        g_current.average = 0;
        g_current.timer = 0;
    }
}

void process_mode( )
{
    static unsigned char lastMode = 0xFF;

    if(g_mode != lastMode)
    {
        switch(g_mode)
        {
            case SIMULATOR:
                DRIVER_COAST = 0;// Motor driver power off.
                SetDCOC1PWM(HALF_PWM_MAX);
                g_led.frequency_A = 1;
                g_led.frequency_B = 1;
                g_error = ALL_OK;
                break;

            case DRIVER_OPEN:
                DRIVER_COAST = 0;// Motor driver power off.
                SetDCOC1PWM(HALF_PWM_MAX);
                g_led.frequency_A = 2;
                g_led.frequency_B = 2;
                g_error = ALL_OK;
                break;

            case OPEN_LOOP:
                DRIVER_COAST = 1;// Motor driver power on.
                SetDCOC1PWM(HALF_PWM_MAX);
                g_led.frequency_A = 5;
                g_led.frequency_B = 5;
                g_error = ALL_OK;
                //g_ctrl.currentTargetPos = g_axis.pos.l;
                break;

            case CLOSE_LOOP:
                DRIVER_COAST = 1;// Motor driver power on.
                g_led.frequency_A = 10;
                g_led.frequency_B = 10;
                g_error = ALL_OK;
                g_ctrl.currentTargetPos = g_axis.pos.l;
                break;

            case ERROR_DRIVER:
            default:
                DRIVER_COAST = 0;// Motor driver power off.
                g_led.frequency_A = 10;
                g_led.frequency_B = 1;
                break;
        }
        lastMode = g_mode;
    }
}

unsigned char process_SPI( )
{
    // Reset the index of SPI buffer.
    if(SDI_SS == 1 && g_spi.index != SPI_MAX_SIZE)
        g_spi.index = 0;

    // No SPI buffer treatment if it is not full.
    if(g_spi.index != SPI_MAX_SIZE)
        return ALL_OK;

    // Reset the index of SPI buffer.
    g_spi.index = 0;

    if(g_spi.rx[0] != SPI_START)
        return SPI_DATA_ERROR;

    switch(g_spi.rx[1])
    {
        case SPI_TARGET:
            return process_SPI_target();
            break;

        case SPI_MODE_READ:
            return process_SPI_modeRead();
            break;

        case SPI_MODE_WRITE:
            return process_SPI_modeWrite();
            break;

        case SPI_PID_READ:
            return process_SPI_PID_read();
            break;

        case SPI_PID_WRITE:
            return process_SPI_PID_write();
            break;

        case SPI_POSITION_WRITE:
            return process_SPI_positionWrite();
            break;

        case SPI_POS_LAG_ERROR_READ:
            return process_SPI_positionLafErrorRead();
            break;

        case SPI_POS_LAG_ERROR_WRITE:
            return process_SPI_positionLafErrorWrite();
            break;

        case SPI_LOOP_FREQUENCY_READ:
            return process_SPI_loopFrequencyRead();
            break;

        case SPI_LOOP_FREQUENCY_WRITE:
            return process_SPI_loopFrequencyWrite();
            break;

        case SPI_BUS_FREQUENCY_READ:
            return process_SPI_busFrequencyRead();
            break;

        case SPI_BUS_FREQUENCY_WRITE:
            return process_SPI_busFrequencyWrite();
            break;

        default:
            return SPI_UNCKNOW;
            break;
    }

    return SPI_DATA_ERROR;
}

void process_loop( )
{
    static long lastPosition = 0;
    static long lastSpeed = 0;
    static double errorSum = 0.0;

    // Manage the time to have a cyclic time configurable.
    if(g_ctrl.timer < T1_FREQ / g_ctrl.loopFrequency.i)
    {
        g_ctrl.timer++;
        return;
    }
    else
        g_ctrl.timer = 0;

    // Compute the target position interpolated.
    g_ctrl.currentTargetPos += g_ctrl.stepPos;

    if(g_mode == SIMULATOR)
        // Put the exact target position to the actual position in order to simulate.
        g_axis.pos.l = g_ctrl.currentTargetPos;
    else
        // Compute the exact position according to 16 bit overflow from QEI module.
        g_axis.pos.l = g_encoderU16 + ReadQEI();

    g_axis.speed.l = (g_axis.pos.l - lastPosition) * g_ctrl.loopFrequency.i;
    g_axis.accel.l = (g_axis.speed.l - lastSpeed) * g_ctrl.loopFrequency.i;

    lastPosition = g_axis.pos.l;
    lastSpeed = g_axis.speed.l;

    if(g_mode != CLOSE_LOOP)
    {
        errorSum = 0.0;
        return;
    }

    // Compute the actual position error.
    const double posError = g_ctrl.currentTargetPos - g_axis.pos.l;

    if(fabs(posError) >= g_axis.posLagErrorMax.l)
    {
        g_mode = ERROR_DRIVER;
        g_error = POS_LAG_ERROR;
        return;
    }

    errorSum += posError;

    double cmd = posError * g_pid.kp.value / (double)g_ctrl.loopFrequency.i
            + errorSum * g_pid.ki.value / (double)g_ctrl.loopFrequency.i;

    if(cmd >= 0)
        g_axis.direction = 1;
    else
        g_axis.direction = -1;

    // Offset for the PWM (0 -> 1480)
    cmd += HALF_PWM_MAX;

    // Limit the PWM resulting.
    if(cmd > PWM_MAX)
        cmd = PWM_MAX;
    else if(cmd < 0.0)
        cmd = 0.0;

    SetDCOC1PWM(cmd);
}

unsigned char process_SPI_target( )
{
    if(g_spi.rx[10] != SPI_END || g_spi.rx[SPI_MAX_SIZE - 1] != SPI_END)
        return SPI_DATA_ERROR;

    // Memorize the last target position command.
    g_ctrl.currentTargetPos = g_axis.targetPos.l;

    // Get target position from SPI.
    g_axis.targetPos.c[0] = g_spi.rx[2];
    g_axis.targetPos.c[1] = g_spi.rx[3];
    g_axis.targetPos.c[2] = g_spi.rx[4];
    g_axis.targetPos.c[3] = g_spi.rx[5];

    // Get target speed from SPI.
    g_axis.targetSpeed.c[0] = g_spi.rx[6];
    g_axis.targetSpeed.c[1] = g_spi.rx[7];
    g_axis.targetSpeed.c[2] = g_spi.rx[8];
    g_axis.targetSpeed.c[3] = g_spi.rx[9];

    g_spi.tx[0] = SPI_START;
    g_spi.tx[1] = SPI_TARGET;

    g_spi.tx[2] = g_axis.pos.c[0];
    g_spi.tx[3] = g_axis.pos.c[1];
    g_spi.tx[4] = g_axis.pos.c[2];
    g_spi.tx[5] = g_axis.pos.c[3];

    g_spi.tx[6] = g_axis.speed.c[0];
    g_spi.tx[7] = g_axis.speed.c[1];
    g_spi.tx[8] = g_axis.speed.c[2];
    g_spi.tx[9] = g_axis.speed.c[3];

    g_spi.tx[10] = g_axis.accel.c[0];
    g_spi.tx[11] = g_axis.accel.c[1];
    g_spi.tx[12] = g_axis.accel.c[2];
    g_spi.tx[13] = g_axis.accel.c[3];

    g_spi.tx[14] = g_current.value.c[0];
    g_spi.tx[15] = g_current.value.c[1];

    g_spi.tx[16] = g_error;
    g_spi.tx[17] = SPI_END;

    // Reset index for the new interpolation.
    const long count = g_ctrl.loopFrequency.i / g_ctrl.busFrequency.i;
    g_ctrl.stepPos = (g_axis.targetPos.l - g_ctrl.currentTargetPos) / count;

    return ALL_OK;
}

unsigned char process_SPI_modeRead( )
{
    if(g_spi.rx[2] != SPI_END || g_spi.rx[SPI_MAX_SIZE - 1] != SPI_END)
        return SPI_DATA_ERROR;

    // Set the current mode to SPI buffer.
    g_spi.tx[0] = SPI_START;
    g_spi.tx[1] = SPI_MODE_READ;
    g_spi.tx[2] = g_mode;
    g_spi.tx[3] = SPI_END;

    return ALL_OK;
}

unsigned char process_SPI_modeWrite( )
{
    if(g_spi.rx[3] != SPI_END || g_spi.rx[SPI_MAX_SIZE - 1] != SPI_END)
        return SPI_DATA_ERROR;

    // Get the new mode from SPI buffer.
    g_mode = g_spi.rx[2];

    g_spi.tx[0] = SPI_START;
    g_spi.tx[1] = SPI_MODE_WRITE;
    g_spi.tx[2] = SPI_END;

    return ALL_OK;
}

unsigned char process_SPI_PID_read( )
{
    if(g_spi.rx[2] != SPI_END || g_spi.rx[SPI_MAX_SIZE - 1] != SPI_END)
        return SPI_DATA_ERROR;

    g_pid.kp.bus.l = g_pid.kp.value * 65536.0;
    g_pid.ki.bus.l = g_pid.ki.value * 65536.0;
    g_pid.kd.bus.l = g_pid.kd.value * 65536.0;

    // Set the current PID values to SPI buffer.
    g_spi.tx[0] = SPI_START;
    g_spi.tx[1] = SPI_PID_READ;

    g_spi.tx[2] = g_pid.kp.bus.c[0];
    g_spi.tx[3] = g_pid.kp.bus.c[1];
    g_spi.tx[4] = g_pid.kp.bus.c[2];
    g_spi.tx[5] = g_pid.kp.bus.c[3];

    g_spi.tx[6] = g_pid.ki.bus.c[0];
    g_spi.tx[7] = g_pid.ki.bus.c[1];
    g_spi.tx[8] = g_pid.ki.bus.c[2];
    g_spi.tx[9] = g_pid.ki.bus.c[3];

    g_spi.tx[10] = g_pid.kd.bus.c[0];
    g_spi.tx[11] = g_pid.kd.bus.c[1];
    g_spi.tx[12] = g_pid.kd.bus.c[2];
    g_spi.tx[13] = g_pid.kd.bus.c[3];

    g_spi.tx[14] = SPI_END;

    return ALL_OK;
}

unsigned char process_SPI_PID_write( )
{
    if(g_spi.rx[14] != SPI_END || g_spi.rx[SPI_MAX_SIZE - 1] != SPI_END)
        return SPI_DATA_ERROR;

    // Set the buffer for response to SPI.
    g_spi.tx[0] = SPI_START;
    g_spi.tx[1] = SPI_PID_WRITE;
    g_spi.tx[2] = SPI_END;

    // Get new value kp of PID from SPI.
    g_pid.kp.bus.c[0] = g_spi.rx[2];
    g_pid.kp.bus.c[1] = g_spi.rx[3];
    g_pid.kp.bus.c[2] = g_spi.rx[4];
    g_pid.kp.bus.c[3] = g_spi.rx[5];

    // Get new value ki of PID from SPI.
    g_pid.ki.bus.c[0] = g_spi.rx[6];
    g_pid.ki.bus.c[1] = g_spi.rx[7];
    g_pid.ki.bus.c[2] = g_spi.rx[8];
    g_pid.ki.bus.c[3] = g_spi.rx[9];

    // Get new value kd of PID from SPI.
    g_pid.kd.bus.c[0] = g_spi.rx[10];
    g_pid.kd.bus.c[1] = g_spi.rx[11];
    g_pid.kd.bus.c[2] = g_spi.rx[12];
    g_pid.kd.bus.c[3] = g_spi.rx[13];

    // Set the kp, ki and kd values.
    g_pid.kp.value = (double)g_pid.kp.bus.l / 65536.0;
    g_pid.ki.value = (double)g_pid.ki.bus.l / 65536.0;
    g_pid.kd.value = (double)g_pid.kd.bus.l / 65536.0;

    return ALL_OK;
}

unsigned char process_SPI_positionWrite( )
{
    if(g_spi.rx[6] != SPI_END || g_spi.rx[SPI_MAX_SIZE - 1] != SPI_END)
        return SPI_DATA_ERROR;

    // Set the buffer for response to SPI.
    g_spi.tx[0] = SPI_START;
    g_spi.tx[1] = SPI_POSITION_WRITE;
    g_spi.tx[2] = SPI_END;

    // Get new value kp of PID from SPI.
    g_axis.pos.c[0] = g_spi.rx[2];
    g_axis.pos.c[1] = g_spi.rx[3];
    g_axis.pos.c[2] = g_spi.rx[4];
    g_axis.pos.c[3] = g_spi.rx[5];

    g_encoderU16 = g_axis.pos.l - ReadQEI();
    g_ctrl.currentTargetPos = g_axis.pos.l;
    return ALL_OK;
}

unsigned char process_SPI_positionLafErrorRead( )
{
    if(g_spi.rx[2] != SPI_END || g_spi.rx[SPI_MAX_SIZE - 1] != SPI_END)
        return SPI_DATA_ERROR;

    // Set the current PID values to SPI buffer.
    g_spi.tx[0] = SPI_START;
    g_spi.tx[1] = SPI_POS_LAG_ERROR_READ;

    g_spi.tx[2] = g_axis.posLagErrorMax.c[0];
    g_spi.tx[3] = g_axis.posLagErrorMax.c[1];
    g_spi.tx[4] = g_axis.posLagErrorMax.c[2];
    g_spi.tx[5] = g_axis.posLagErrorMax.c[3];

    g_spi.tx[6] = SPI_END;

    return ALL_OK;
}

unsigned char process_SPI_positionLafErrorWrite( )
{
    if(g_spi.rx[6] != SPI_END || g_spi.rx[SPI_MAX_SIZE - 1] != SPI_END)
        return SPI_DATA_ERROR;

    // Set the buffer for response to SPI.
    g_spi.tx[0] = SPI_START;
    g_spi.tx[1] = SPI_POS_LAG_ERROR_WRITE;
    g_spi.tx[2] = SPI_END;

    // Get new value kp of PID from SPI.
    g_axis.posLagErrorMax.c[0] = g_spi.rx[2];
    g_axis.posLagErrorMax.c[1] = g_spi.rx[3];
    g_axis.posLagErrorMax.c[2] = g_spi.rx[4];
    g_axis.posLagErrorMax.c[3] = g_spi.rx[5];

    return ALL_OK;
}

unsigned char process_SPI_loopFrequencyRead( )
{
    if(g_spi.rx[2] != SPI_END || g_spi.rx[SPI_MAX_SIZE - 1] != SPI_END)
        return SPI_DATA_ERROR;

    // Set the current PID values to SPI buffer.
    g_spi.tx[0] = SPI_START;
    g_spi.tx[1] = SPI_LOOP_FREQUENCY_READ;

    g_spi.tx[2] = g_ctrl.loopFrequency.c[0];
    g_spi.tx[3] = g_ctrl.loopFrequency.c[1];

    g_spi.tx[4] = SPI_END;

    return ALL_OK;
}

unsigned char process_SPI_loopFrequencyWrite( )
{
    if(g_spi.rx[4] != SPI_END || g_spi.rx[SPI_MAX_SIZE - 1] != SPI_END)
        return SPI_DATA_ERROR;

    // Set the buffer for response to SPI.
    g_spi.tx[0] = SPI_START;
    g_spi.tx[1] = SPI_LOOP_FREQUENCY_WRITE;
    g_spi.tx[2] = SPI_END;

    // Get new value kp of PID from SPI.
    g_ctrl.loopFrequency.c[0] = g_spi.rx[2];
    g_ctrl.loopFrequency.c[1] = g_spi.rx[3];

    return ALL_OK;
}

unsigned char process_SPI_busFrequencyRead( )
{
    if(g_spi.rx[2] != SPI_END || g_spi.rx[SPI_MAX_SIZE - 1] != SPI_END)
        return SPI_DATA_ERROR;

    // Set the current PID values to SPI buffer.
    g_spi.tx[0] = SPI_START;
    g_spi.tx[1] = SPI_BUS_FREQUENCY_READ;

    g_spi.tx[2] = g_ctrl.busFrequency.c[0];
    g_spi.tx[3] = g_ctrl.busFrequency.c[1];

    g_spi.tx[4] = SPI_END;

    return ALL_OK;
}

unsigned char process_SPI_busFrequencyWrite( )
{
    if(g_spi.rx[4] != SPI_END || g_spi.rx[SPI_MAX_SIZE - 1] != SPI_END)
        return SPI_DATA_ERROR;

    // Set the buffer for response to SPI.
    g_spi.tx[0] = SPI_START;
    g_spi.tx[1] = SPI_BUS_FREQUENCY_WRITE;
    g_spi.tx[2] = SPI_END;

    // Get new value kp of PID from SPI.
    g_ctrl.busFrequency.c[0] = g_spi.rx[2];
    g_ctrl.busFrequency.c[1] = g_spi.rx[3];

    return ALL_OK;
}

/**
 * Timer 1 interrupt service routine (20khz).
 */
void __attribute__( (interrupt,no_auto_psv) ) _T1Interrupt( void )
{
    WriteTimer1(0);
    _T1IF = 0;

    process_loop();
}

/**
 * Timer 2 interrupt service routine (10kHz).
 */
void __attribute__( (interrupt,no_auto_psv) ) _T2Interrupt( void )
{
    WriteTimer2(0);
    _T2IF = 0;

    g_led.timer++;
    g_current.timer++;
    g_wd.timer++;
}

/**
 * SPI receive interrupt service routine.
 */
void __attribute__( (interrupt,no_auto_psv) ) _SPI1Interrupt( void )
{
    _SPI1IF = 0;

    // Error, overflow.
    if(SPI1STATbits.SPIROV)
    {
        // clear overflow.
        SPI1STATbits.SPIROV = 0;
        g_spi.index = 0;
    }
    else if(!SPI1STATbits.SPIRBF)
    {
        // SPI error.
        g_spi.index = 0;
    }
    else
    {
        // If SPI bytes are receided, we restart watchdog.
        g_wd.timer = 0;

        if(g_spi.index < SPI_MAX_SIZE)
        {
            g_spi.rx[g_spi.index] = SPI1BUF;
            while(SPI1STATbits.SPITBF);
            SPI1BUF = g_spi.tx[g_spi.index];

            g_spi.index++;
        }
    }
}

/**
 * External interrupt service routine. It is used to intercept encoder sensor.
 */
void __attribute__( (interrupt,no_auto_psv) ) _QEIInterrupt( void )
{
    // Clear QEI interrupt flag.
    _QEIIF = 0;

    // Check if there is an 16 bits overflow.
    if(QEICONbits.UPDN)
        g_encoderU16 += 0x7FFF;
    else
        g_encoderU16 -= 0x7FFF;
}

/**
 * External interrupt 2 service routine. It is used to interpret hall sensor
 * signal.
 */
void __attribute__( (interrupt,no_auto_psv) ) _INT2Interrupt( void )
{
    // Clear INT2 interrupt flag.
    _INT2IF = 0;

    // Counter unit from coder pulse.
    g_hallUnit++;
}
