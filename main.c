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

// Current project
#include "Config.h"
#include "YaXiType.h"

// Axis parameters -----------------------------------------------------------//
SafeData_S32 g_position = {.value = 0};
SafeData_S32 g_speed = {.value = 0};
SafeData_S32 g_accel = {.value = 0};

volatile long g_encoderU16 = 0;
volatile long g_hallUnit = 0;

SafeData_S32 g_targetPos = {.value = 0};
SafeData_S32 g_targetSpeed = {.value = 0};

SafeData_U8 g_mode = {.bus = DRIVER_OPEN,.value = DRIVER_OPEN};
volatile unsigned char g_error = 0;

SafeData_D32 g_kp = {.value = 0.0};
SafeData_D32 g_ki = {.value = 0.0};
SafeData_D32 g_kd = {.value = 0.0};

long g_lastPositionUnit = 0;
long g_lastSpeedUnit = 0;
unsigned char g_lastMode = 0xFF;

// Led managmement -----------------------------------------------------------//
typedef struct
{
    unsigned int timer;
    unsigned int frequency;
} Led;
Led g_led = {.timer = 0,.frequency = LED_FREQ_10HZ};

// Current management --------------------------------------------------------//
typedef struct
{
    union U16_U8 bus;
    unsigned char timer;
    unsigned char state;
    unsigned long measure;
    unsigned long average;
    volatile unsigned int value;
} Current;
Current g_current = {.timer = 0,.state = 0,.measure = 0,.average = 0};

// SPI workflow management ---------------------------------------------------//
Com_SPI g_spi = {.index = 0};

// Close loop interpolation management ---------------------------------------//
Interpolation g_cl = {.frequency = 0,.index = 0};

/**
 * Main function, manage all initialization and continuous process.
 * @return 0 at the end of program.
 */
int main( )
{
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
        process_mode();

        if(process_SPI() != SPI_ERROR_DATA)
        {
            process_monitoring();
            process_loop();
        }
        else
            g_mode.value = DRIVER_OPEN;
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
               &T1_SOURCE_INT
               ,
               PR_T1);

    // Timer2 configuration (10khz).
    ConfigIntTimer2(T2_INT_PRIOR_3 & T2_INT_ON);
    WriteTimer2(0);
    OpenTimer2(T2_ON
               & T2_IDLE_STOP
               & T2_GATE_OFF
               & T2_PS_1_1
               &T2_SOURCE_INT
               ,
               PR_T2);

    // Timer4 configuration (10hz).
    ConfigIntTimer4(T4_INT_PRIOR_1 & T4_INT_ON);
    WriteTimer4(0);
    OpenTimer4(T4_ON
               & T4_IDLE_STOP
               & T4_GATE_OFF
               & T4_PS_1_64
               & T4_SOURCE_INT
               ,
               PR_T4);
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
    if(g_led.timer >= g_led.frequency)
    {
        g_led.timer = 0;
        LED = ~LED;
    }
}

void process_current( )
{
    if(g_current.state == 0 && g_current.timer >= 20)
    {
        // Select the requested channel.
        SetChanADC10(ADC_CSOUT);

        // Start sampling.
        ADCON1bits.SAMP = 1;

        g_current.state = 1;
        g_current.timer = 0;
    }
    else if(g_current.state == 1 && !ADCON1bits.SAMP)
    {
        // Start Converting.
        ConvertADC10();

        g_current.state = 2;
    }
    else if(g_current.state == 2 && ADCON1bits.DONE == 1)
    {
        g_current.average += ReadADC10(0);
        g_current.measure++;
        g_current.state = 0;
    }

    if(g_current.measure >= 5)
    {
        g_current.value = g_current.average / g_current.measure;

        g_current.measure = 0;
        g_current.average = 0;
    }
}

void process_mode( )
{
    if(g_mode.value != g_lastMode)
    {
        switch(g_mode.value)
        {
            case DRIVER_OPEN:
                DRIVER_COAST = 0;// Motor driver power off.
                SetDCOC1PWM(HALF_PWM_MAX);
                g_led.frequency = LED_FREQ_2HZ;
                break;

            case OPEN_LOOP:
                DRIVER_COAST = 1;// Motor driver power on.
                SetDCOC1PWM(HALF_PWM_MAX);
                g_led.frequency = LED_FREQ_10HZ;
                break;

            case CLOSE_LOOP:
                DRIVER_COAST = 1;// Motor driver power on.
                g_led.frequency = LED_FREQ_5HZ;
                break;

            default:
                DRIVER_COAST = 0;// Motor driver power off.
                g_led.frequency = LED_FREQ_1HZ;
                break;
        }
        g_lastMode = g_mode.value;
    }
}

unsigned char process_SPI( )
{
    if(g_spi.index != SPI_MAX_SIZE)
        return SPI_ERROR_DATA;

    if(g_spi.rx[0] != SPI_START)
        return SPI_ERROR_DATA;

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

        default:
            g_spi.tx[0] = SPI_START;
            g_spi.tx[1] = SPI_UNCKNOW;
            g_spi.tx[2] = SPI_END;
            break;
    }

    return SPI_ERROR_DATA;
}

void process_monitoring( )
{
    // Compute the exact position according to 16 bit overflow from QEI module.
    g_position.value = g_encoderU16 + ReadQEI();

    g_speed.value = (g_position.value - g_lastPositionUnit) * g_cl.frequency;
    g_accel.value = (g_speed.value - g_lastSpeedUnit) * g_cl.frequency;

    g_lastPositionUnit = g_position.value;
    g_lastSpeedUnit = g_speed.value;
}

void process_loop( )
{
    if(g_mode.value != CLOSE_LOOP)
        return;

    double posError = g_targetPos.value - g_position.value;
    double cmd = posError * g_kp.value;

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
    if(g_spi.rx[16] != SPI_END)
        return SPI_ERROR_DATA;

    // Get target position from SPI.
    g_targetPos.bus.c[0] = g_spi.rx[2];
    g_targetPos.bus.c[1] = g_spi.rx[3];
    g_targetPos.bus.c[2] = g_spi.rx[4];
    g_targetPos.bus.c[3] = g_spi.rx[5];

    // Get target speed from SPI.
    g_targetSpeed.bus.c[0] = g_spi.rx[6];
    g_targetSpeed.bus.c[1] = g_spi.rx[7];
    g_targetSpeed.bus.c[2] = g_spi.rx[8];
    g_targetSpeed.bus.c[3] = g_spi.rx[9];

    // Set current position, speed, acceleration and current to SPI buffer.
    g_position.bus.l = g_position.value;
    g_speed.bus.l = g_speed.value;
    g_accel.bus.l = g_accel.value;
    g_current.bus.i = g_current.value;

    g_spi.tx[0] = SPI_START;
    g_spi.tx[1] = SPI_TARGET;

    g_spi.tx[2] = g_position.bus.c[0];
    g_spi.tx[3] = g_position.bus.c[1];
    g_spi.tx[4] = g_position.bus.c[2];
    g_spi.tx[5] = g_position.bus.c[3];

    g_spi.tx[6] = g_speed.bus.c[0];
    g_spi.tx[7] = g_speed.bus.c[1];
    g_spi.tx[8] = g_speed.bus.c[2];
    g_spi.tx[9] = g_speed.bus.c[3];

    g_spi.tx[10] = g_accel.bus.c[0];
    g_spi.tx[11] = g_accel.bus.c[1];
    g_spi.tx[12] = g_accel.bus.c[2];
    g_spi.tx[13] = g_accel.bus.c[3];

    g_spi.tx[14] = g_current.bus.c[0];
    g_spi.tx[15] = g_current.bus.c[1];

    g_spi.tx[16] = SPI_END;

    return SPI_NO_ERROR;
}

unsigned char process_SPI_modeRead( )
{
    if(g_spi.rx[2] != SPI_END)
        return SPI_ERROR_DATA;

    // Set the current mode to SPI buffer.
    g_spi.tx[0] = SPI_START;
    g_spi.tx[1] = SPI_MODE_READ;
    g_spi.tx[2] = g_mode.value;
    g_spi.tx[3] = SPI_END;

    return SPI_NO_ERROR;
}

unsigned char process_SPI_modeWrite( )
{
    if(g_spi.rx[3] != SPI_END)
        return SPI_ERROR_DATA;

    // Get the new mode from SPI buffer.
    g_mode.value = g_spi.rx[2];

    g_spi.tx[0] = SPI_START;
    g_spi.tx[1] = SPI_MODE_WRITE;
    g_spi.tx[2] = SPI_END;

    return SPI_NO_ERROR;
}

unsigned char process_SPI_PID_read( )
{
    if(g_spi.rx[2] != SPI_END)
        return SPI_ERROR_DATA;

    g_kp.bus.l = g_kp.value * 65536.0;
    g_ki.bus.l = g_ki.value * 65536.0;
    g_kd.bus.l = g_kd.value * 65536.0;

    // Set the current PID values to SPI buffer.
    g_spi.tx[0] = SPI_START;
    g_spi.tx[1] = SPI_PID_READ;

    g_spi.tx[2] = g_kp.bus.c[0];
    g_spi.tx[3] = g_kp.bus.c[1];
    g_spi.tx[4] = g_kp.bus.c[2];
    g_spi.tx[5] = g_kp.bus.c[3];

    g_spi.tx[6] = g_ki.bus.c[0];
    g_spi.tx[7] = g_ki.bus.c[1];
    g_spi.tx[8] = g_ki.bus.c[2];
    g_spi.tx[9] = g_ki.bus.c[3];

    g_spi.tx[10] = g_kd.bus.c[0];
    g_spi.tx[11] = g_kd.bus.c[1];
    g_spi.tx[12] = g_kd.bus.c[2];
    g_spi.tx[13] = g_kd.bus.c[3];

    g_spi.tx[14] = SPI_END;

    return SPI_NO_ERROR;
}

unsigned char process_SPI_PID_write( )
{
    if(g_spi.rx[14] != SPI_END)
        return SPI_ERROR_DATA;

    // Set the buffer for response to SPI.
    g_spi.tx[0] = SPI_START;
    g_spi.tx[1] = SPI_PID_WRITE;
    g_spi.tx[3] = SPI_END;

    // Get new value kp of PID from SPI.
    g_kp.bus.c[0] = g_spi.rx[2];
    g_kp.bus.c[1] = g_spi.rx[3];
    g_kp.bus.c[2] = g_spi.rx[4];
    g_kp.bus.c[3] = g_spi.rx[5];

    // Get new value ki of PID from SPI.
    g_ki.bus.c[0] = g_spi.rx[6];
    g_ki.bus.c[1] = g_spi.rx[7];
    g_ki.bus.c[2] = g_spi.rx[8];
    g_ki.bus.c[3] = g_spi.rx[9];

    // Get new value kd of PID from SPI.
    g_kd.bus.c[0] = g_spi.rx[10];
    g_kd.bus.c[1] = g_spi.rx[11];
    g_kd.bus.c[2] = g_spi.rx[12];
    g_kd.bus.c[3] = g_spi.rx[13];

    // Set the kp, ki and kd values.
    g_kp.value = g_kp.bus.l / 65536.0;
    g_ki.value = g_ki.bus.l / 65536.0;
    g_kd.value = g_kd.bus.l / 65536.0;

    return SPI_NO_ERROR;
}

unsigned char process_SPI_positionWrite( )
{
    if(g_spi.rx[6] != SPI_END)
        return SPI_ERROR_DATA;

    // Set the buffer for response to SPI.
    g_spi.tx[0] = SPI_START;
    g_spi.tx[1] = SPI_POSITION_WRITE;
    g_spi.tx[3] = SPI_END;

    // Get new value kp of PID from SPI.
    g_position.bus.c[0] = g_spi.rx[2];
    g_position.bus.c[1] = g_spi.rx[3];
    g_position.bus.c[2] = g_spi.rx[4];
    g_position.bus.c[3] = g_spi.rx[5];

    g_encoderU16 = g_position.bus.l & 0xFFFF0000;
    WriteQEI(g_position.bus.l);

    return SPI_NO_ERROR;
}

/**
 * Timer 1 interrupt service routine.
 */
void __attribute__( (interrupt,no_auto_psv) ) _T1Interrupt( void )
{
    WriteTimer1(0);
    _T1IF = 0;

    process_monitoring(1000);
    process_loop(1000);
}

/**
 * Timer 2 interrupt service routine.
 */
void __attribute__( (interrupt,no_auto_psv) ) _T2Interrupt( void )
{
    WriteTimer2(0);
    _T2IF = 0;
    g_current.timer++;
}

/**
 * Timer 4 interrupt service routine.
 */
void __attribute__( (interrupt,no_auto_psv) ) _T4Interrupt( void )
{
    WriteTimer4(0);
    _T4IF = 0;
    g_led.timer++;
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
