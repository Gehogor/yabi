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
#include "EepromLib.h"
#include "SharedVarLib.h"
#include "SafeVariable.h"

// Axis parameters
volatile long g_positionUnit = 0;
volatile long g_speedUnit = 0;
volatile long g_accelUnit = 0;
volatile long g_hallUnit = 0;

volatile long g_targetPositionUnit = 0;
volatile long g_targetSpeedUnit = 0;

volatile unsigned char g_mode = STOP;
volatile unsigned char g_error = 0;

volatile double g_kp = 0;
volatile double g_ki = 0;
volatile double g_kd = 0;

long g_lastPositionUnit = 0;
long g_lastSpeedUnit = 0;
unsigned char g_lastMode = 0;

// Led managmement
typedef struct
{
    unsigned int timer;
    unsigned int frequency;
} Led;
Led g_led = {.timer = 0,.frequency = LED_FREQ_10HZ};

// Current management
typedef struct
{
    unsigned char timer;
    unsigned char state;
    unsigned long measure;
    unsigned long average;
    volatile unsigned int value;
} Current;
Current g_current = {.timer = 0,.state = 0,.measure = 0,.average = 0};

// SPI management
typedef struct
{
    unsigned char functionCount;
    unsigned char index;
    unsigned char function;
} Com_SPI;
Com_SPI g_spi = {.functionCount = 0,.index = 0,.function = 0};

// Loop PID managmement
typedef struct
{
    unsigned int timer;
} ControlLoop;
ControlLoop g_cl = {.timer = 0};

// SPI Data management
union S32_U8 spi_targetPos;
union S32_U8 spi_targetSpeed;
union S32_U8 spi_position;
union S32_U8 spi_speed;
union S32_U8 spi_accel;

unsigned char spi_mode;
union U16_U8 spi_current;

union S32_U8 spi_kp;
union S32_U8 spi_ki;
union S32_U8 spi_kd;


// Main function.

int main( )
{
    initIOs();
    initTimer();
    initSPI();
    initADC();
    initInterruptFromEncoderSensor();
    initInterruptFromHallSensor();
    initDriver();

    while(1)
    {
        process_LED();
        process_current();
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
               & T4_PS_1_256
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
            & QEI_GATED_ACC_DISABLE & QEI_INPUTS_NOSWAP
            & QEI_MODE_x4_MATCH
            & QEI_DIR_SEL_CNTRL
            ,
            POS_CNT_ERR_INT_DISABLE
            & QEI_QE_CLK_DIVIDE_1_4
            & QEI_QE_OUT_ENABLE
            & MATCH_INDEX_PHASEA_HIGH
            & MATCH_INDEX_PHASEB_HIGH);
    QEICONbits.UPDN = 1;
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
    DRIVER_MODE = 0;
    DRIVER_COAST = 0;
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
    if(g_mode != g_lastMode)
    {
        switch(g_mode)
        {
            case STOP:
                DRIVER_COAST = 0;//Mise off du Driver
                SetDCOC1PWM(HALF_PWM_MAX);
                g_led.frequency = LED_FREQ_10HZ;
                break;

            case OPEN:
                DRIVER_COAST = 1;//Mise on du Driver
                g_led.frequency = LED_FREQ_2HZ;
                break;

            case LOOP:
                DRIVER_COAST = 1;//Mise on du Driver
                g_led.frequency = LED_FREQ_5HZ;
                break;

            default:
                DRIVER_COAST = 0;//Mise off du Driver
                g_led.frequency = LED_FREQ_1HZ;
                break;
        }
        g_lastMode = g_mode;
    }
}

unsigned char process_SPI( unsigned char data )
{
    unsigned char result = 0;

    if(g_spi.functionCount == 0 && data == SPI_START)
    {
        g_spi.functionCount++;
    }
    else if(g_spi.functionCount == 1)
    {
        g_spi.function = SPI1BUF;
        g_spi.functionCount++;
        g_spi.index = 0;
        result = checkIfFunctionExist();
    }
    else if(g_spi.functionCount == 2)
    {
        switch(g_spi.function)
        {
            case SPI_TARGET:
                result = process_SPI_target(data);
                break;

            case SPI_MODE_READ:
                result = process_SPI_modeRead(data);
                break;

            case SPI_MODE_WRITE:
                result = process_SPI_modeWrite(data);
                break;

            case SPI_KP_READ:
                result = process_SPI_kpRead(data);
                break;

            case SPI_KP_WRITE:
                result = process_SPI_kpWrite(data);
                break;

            case SPI_KI_READ:
                result = process_SPI_kiRead(data);
                break;

            case SPI_KI_WRITE:
                result = process_SPI_kiWrite(data);
                break;

            case SPI_KD_READ:
                result = process_SPI_kdRead(data);
                break;

            case SPI_KD_WRITE:
                result = process_SPI_kdWrite(data);
                break;
        }

        g_spi.index++;
    }

    return result;
}

unsigned char process_SPI_target( unsigned char data )
{
    if(g_spi.index < 4)
    {
        if(g_spi.index == 0)
        {
            spi_position.l = g_positionUnit;
            spi_speed.l = g_speedUnit;
            spi_accel.l = g_accelUnit;
            spi_current.i = g_current.value;
        }

        spi_targetPos.c[g_spi.index] = data;
        return spi_position.c[g_spi.index];
    }
    else if(g_spi.index >= 4 && g_spi.index < 8)
    {
        spi_targetSpeed.c[g_spi.index - 4] = data;
        return spi_speed.c[g_spi.index - 4];
    }
    else if(g_spi.index >= 8 && g_spi.index < 12)
    {
        return spi_accel.c[g_spi.index - 8];
    }
    else if(g_spi.index >= 12 && g_spi.index < 14)
    {
        return spi_current.c[g_spi.index - 12];
    }
    else if(g_spi.index == 14 && data == SPI_END)
    {
        g_targetPositionUnit = spi_targetPos.l;
        g_targetSpeedUnit = spi_targetSpeed.l;
        g_spi.functionCount = 0;
        return NO_ERROR;
    }

    g_spi.functionCount = 0;
    return SPI_ERROR_DATA;
}

unsigned char process_SPI_modeRead( unsigned char data )
{
    g_spi.functionCount = 0;

    if(g_spi.index == 0 && data == SPI_END)
        return g_mode;

    return SPI_ERROR_DATA;
}

unsigned char process_SPI_modeWrite( unsigned char data )
{
    if(g_spi.index == 0)
        spi_mode = data;
    else if(g_spi.index == 1 && data == SPI_END)
    {
        g_mode = spi_mode;
        g_spi.functionCount = 0;
        return NO_ERROR;
    }

    g_spi.functionCount = 0;
    return SPI_ERROR_DATA;
}

unsigned char process_SPI_kpRead( unsigned char data )
{
    if(g_spi.index < 4)
    {
        if(g_spi.index == 0)
            spi_kp.l = g_kp;
        return spi_kp.c[g_spi.index];
    }
    else if(g_spi.index == 4 && data == SPI_END)
    {
        g_spi.functionCount = 0;
        return NO_ERROR;
    }
    
    g_spi.functionCount = 0;
    return SPI_ERROR_DATA;
}

unsigned char process_SPI_kpWrite( unsigned char data )
{

}

unsigned char process_SPI_kiRead( unsigned char data )
{

}

unsigned char process_SPI_kiWrite( unsigned char data )
{

}

unsigned char process_SPI_kdRead( unsigned char data )
{

}

unsigned char process_SPI_kdWrite( unsigned char data )
{

}

void processMonitoring( long frequency )
{
    g_speedUnit = (g_positionUnit - g_lastPositionUnit) * frequency;
    g_accelUnit = (g_speedUnit - g_lastSpeedUnit) * frequency;

    g_lastPositionUnit = g_positionUnit;
    g_lastSpeedUnit = g_speedUnit;
}

void processLoop( long frequency )
{
    if(g_mode != LOOP)
        return;

    double posError = g_targetPositionUnit - g_positionUnit;
    double cmd = posError * toS32_s32(&spi_kp) / 65536.0;

    // Offset for the PWM (0 -> 1480)
    cmd += HALF_PWM_MAX;

    // Limit the PWM resulting.
    if(cmd > PWM_MAX)
        cmd = PWM_MAX;
    else if(cmd < 0.0)
        cmd = 0.0;

    SetDCOC1PWM(cmd);
}

unsigned char checkIfFunctionExist( )
{
    switch(g_spi.function)
    {
        case SPI_TARGET:
        case SPI_MODE_READ:
        case SPI_MODE_WRITE:
        case SPI_KP_READ:
        case SPI_KP_WRITE:
        case SPI_KI_READ:
        case SPI_KI_WRITE:
        case SPI_KD_READ:
        case SPI_KD_WRITE:
            return g_spi.function;
    }

    g_spi.functionCount = 0;
    return SPI_UNCKNOW;
}

/**
 * Timer 1 interrupt service routine.
 */
void __attribute__( (interrupt,no_auto_psv) ) _T1Interrupt( void )
{
    WriteTimer1(0);
    _T1IF = 0;

    processMonitoring(1000);
    processLoop(1000);

    /*if(g_timerSpeed >= g_timePrecision)
    {
        g_timerSpeed = 0;
        if(g_interface_mesure_vitesse_SPI == IV_ENCODER)
        {
            //recup position
            s32MotorPosition.s32_data_APP = (((signed long)g_encoderUnit) << 15)+((signed long)ReadQEI());
            WriteSharedVarS32_APP(&s32MotorPosition);
            //Sauvegarde de la valeur ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â  soustraire pour la prochain calcul
            g_motor_pos_mem = s32MotorPosition.s32_data_APP;

            //Determine le nombre de pas depuis la derniere acquisition
            g_motor_vitesse = s32MotorPosition.s32_data_APP - g_motor_pos_mem;
            g_motor_vitesse *= 1000;// ms -> s
            g_motor_vitesse /= g_timePrecision;// pulse/s

            //Copie la vitesse calculer dans la variable partagÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½e
            s32MesuredSpeed.s32_data_APP = g_motor_vitesse;
            WriteSharedVarS32_APP(&s32MesuredSpeed);
        }
        else
        {
            g_motor_vitesse = g_hallUnit;
            g_hallUnit = 0;// Reset the counter
            g_motor_vitesse *= 1000;// ms -> s
            g_motor_vitesse /= g_timePrecision;// pulse/s

            if(DRIVER_DIRO == 1)
                g_motor_vitesse *= -1;

            //RPS    driver = 24 pulses / tr
            //            g_motor_vitesse /= 24;
            s32MesuredSpeed.s32_data_APP = g_motor_vitesse;
            WriteSharedVarS32_APP(&s32MesuredSpeed);
        }

        ReadSharedVarS32_APP(&s32SetpointSpeed);
        g_dutycycle = s32SetpointSpeed.s32_data_APP;

        if(g_flag_asser == LOOP)
        {
            // Erreur Proportionnelle
            g_Erreur_P = s32SetpointSpeed.s32_data_APP - g_motor_vitesse;
            s32MesuredAcceleration.s32_data_APP = g_Erreur_P;
            WriteSharedVarS32_APP(&s32MesuredAcceleration);
            // Cumul IntÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½grale
            g_Erreur_I += g_Erreur_P;

            // Calcul Asservissement Kp
            ReadSharedVarU16_APP(&u16KpNum);
            ReadSharedVarU16_APP(&u16KpDenum);
            g_dutycycle += g_Erreur_P;
            g_dutycycle *= (signed long)u16KpNum.u16_data_APP;
            g_dutycycle /= (signed long)u16KpDenum.u16_data_APP;

            // Calcul Asservissement Ki
            ReadSharedVarU16_APP(&u16KiNum);
            ReadSharedVarU16_APP(&u16KiDenum);
            g_dutycycle += g_Erreur_I;
            g_dutycycle *= (signed long)u16KiNum.u16_data_APP;
            g_dutycycle /= (signed long)u16KiDenum.u16_data_APP;
        }

        // Remise ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½ l'ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½chelle en fonction de
        // la vitesse maximale, PWM : Max 1480 -> +/- 740.
        g_dutycycle *= (PWM_VAL_MAX / 2);
        ReadSharedVarS32_APP(&s32ConfMaxSpeed);
        g_dutycycle /= s32ConfMaxSpeed.s32_data_APP;

        // Offset for the PWM (0 -> 1480)
        g_dutycycle += PWM_VAL_MAX / 2;

        SetDCOC1PWM((unsigned int)(g_dutycycle));
    }*/
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
    unsigned char result = 0;
    _SPI1IF = 0;

    // Error, overflow.
    if(SPI1STATbits.SPIROV)
    {
        // clear overflow.
        SPI1STATbits.SPIROV = 0;
        g_spi.functionCount = 0;
    }
    else if(!SPI1STATbits.SPIRBF)
    {
        // SPI error.
        g_spi.functionCount = 0;
    }
    else
    {
        result = process_SPI(SPI1BUF);
        while(SPI1STATbits.SPITBF);
        SPI1BUF = result;
    }
}

/**
 * External interrupt service routine. It is used to intercept encoder sensor.
 */
void __attribute__( (interrupt,no_auto_psv) ) _QEIInterrupt( void )
{
    // Clear QEI interrupt flag.
    _QEIIF = 0;

    // Counter unit from coder pulse.
    if(QEICONbits.UPDN)
        g_positionUnit++;
    else
        g_positionUnit--;
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
