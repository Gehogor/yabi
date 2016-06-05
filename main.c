/*
 *__     __        ____  _____
 *\ \   / / /\    |  _ \|_   _|
 * \ \_/ / /  \   | |_) | | |
 *  \   / / /\ \  |  _ <  | |
 *   | | / ____ \ | |_) |_| |_
 *   |_|/_/    \_\|____/|_____|
 *
 * File:   main.c
 * Author: Yann
 *
 * Created on 25 avril 2016, 19:01
 */

// PIC management
#include <adc10.h>
#include <libpic30.h>
#include <outcompare.h>
#include <p30F4012.h>
#include <ports.h>
#include <spi.h>
#include <timer.h>
#include <uart.h>
#include <qei.h>

// Standard
#include <stdio.h>

// Current project
#include "Config.h"
#include "main.h"
#include "SharedVarLib.h"


unsigned long g_ID_board = 43981;

// Global timer variable
unsigned int g_timerSpeed = 0;
unsigned int g_timer_AcqCourant = 0;
unsigned int g_timer_led = 0;
unsigned int g_timerControl = 0;

unsigned int g_multi_timeMesureSpeed_s = 0;
unsigned char g_flag_asser = OPEN;
unsigned char g_var_led = LED_FREQ_5HZ;

trame_SPI g_SPI_RX_trame;
unsigned char g_SPI_RX_flag = 0;
unsigned char g_SPI_RX_trame_valide = 0;
unsigned char g_SPI_RX_clearReg = 0;
unsigned char g_SPI_RX_NbData[NB_FONCTION + 1];

signed int g_motor_pos_H = 0;
signed long g_motor_pos_mem = 0;
signed long g_motor_vitesse = 0;

signed long g_dutycycle = 0;
unsigned long g_pulse_drive = 0;

unsigned long g_mesure_courant = 0;
unsigned char g_stateAcq = 0;
unsigned char g_nb_mes_courant = 0;

signed long g_Erreur_P = 0;
signed long g_Erreur_I = 0;

//Initialisation du Mode (Mode_mem différent pour lancer l'init du mode Stop)
unsigned char g_mode;
unsigned char g_mode_mem;

signed long g_motor_pos_APP = 0;
signed long g_vitesse_desiree_APP = 0;
signed int g_Vitesse_Cour = 0;

unsigned char g_timeMesureSpeed = 0;
unsigned char g_timeControlLoop = 0;

unsigned char g_TimeOut_Recep = 0;
unsigned char g_Erreur_Cour = 0x00;

/*
unsigned int g_nom_Kp = 1;
unsigned int g_denom_Kp = 2000;
unsigned int g_nom_Ki = 1;
unsigned int g_denom_Ki = 4500;
unsigned int g_nom_Kd = 1;
unsigned int g_denom_Kd_ = 1;
*/
//signed long g_erreur_limite_I = 0;

unsigned char g_interface_mesure_vitesse_SPI = IV_TACHY;
/*
unsigned int g_motor_conf_codeur_ppt_SPI = 0;
unsigned int g_motor_conf_courant_max_SPI = 2330;
signed int g_motor_conf_vitesse_max_SPI = 111;
signed long g_vitesse_desiree_SPI=0;
signed long g_motor_pos_SPI=0;
*/

/*
unsigned int g_mesure_courant_moy_SPI = 0;
signed int g_Acceleration_Cour_SPI = 0;
signed int g_Vitesse_Cour_SPI = 0;
*/

//Variables partagées info driver
//u8_shared_var u8ControlMode; Pas utilisé pour l'intant
u16_shared_var u16KpNum;
u16_shared_var u16KpDenum;
u16_shared_var u16KiNum;
u16_shared_var u16KiDenum;
u16_shared_var u16KdNum;
u16_shared_var u16KdDenum;
s32_shared_var s32IErrorMax;

//Variables partagées info fonctionnement moteur
u16_shared_var u16MesuredCurrent;
s16_shared_var s16MesuredAcceleration;
s16_shared_var s16MesuredSpeed;
s16_shared_var s16SetpointSpeed;
s32_shared_var s32MotorPosition;

//Variables partagées info fonctionnement moteur
//Variables partagées parametres PID

//Variables partagées parametres Moteur
u16_shared_var u16ConfCodeurPPT;
u16_shared_var u16ConfMaxCurrent;
s16_shared_var s16ConfMaxSpeed;

int main()
{
    Configure_pins();
    InitDriver();
    InitFonctionRecep();
    InitVariable();
    
    g_timeMesureSpeed = 10;
    g_multi_timeMesureSpeed_s = 1000 / ((unsigned int) g_timeMesureSpeed);
    g_timeControlLoop = g_timeMesureSpeed;

    LED = LED_OFF;

    while(1)
    {
        Gestion_LED(g_var_led);

        Gestion_Courant();

        if (g_mode != g_mode_mem) {
            switch (g_mode) {
                case MODE_STOP:
                    DRIVER_COAST = 0; //Mise off du Driver
                    SetDCOC1PWM(PWM_VAL_CENTRE);
                    g_flag_asser = STOP;
                    g_var_led = LED_FREQ_1HZ;
                break;
                case MODE_OPEN:
                    DRIVER_COAST = 1; //Mise on du Driver
                    g_flag_asser = OPEN;
                    g_var_led = LED_FREQ_2HZ;
                break;
                case MODE_ASSER:
                    DRIVER_COAST = 1; //Mise on du Driver
                    g_flag_asser = LOOP;
                    g_var_led = LED_FREQ_5HZ;
                break;
                default:
                    DRIVER_COAST = 0; //Mise off du Driver
                    g_flag_asser = STOP;
                    g_var_led = LED_FREQ_1HZ;
                break;
            }
            g_mode_mem = g_mode;
        }
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

void Configure_pins()
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

    // Timer1 configuration (1khz).
    ConfigIntTimer1(T1_INT_PRIOR_5 & T1_INT_ON);
    WriteTimer1(0);
    OpenTimer1( T1_ON &
                T1_IDLE_STOP &
                T1_GATE_OFF &
                T1_PS_1_1 &
                T1_SYNC_EXT_OFF &
                T1_SOURCE_INT,
                PR_T1);

    // Timer2 configuration (10khz).
    ConfigIntTimer2(T2_INT_PRIOR_3 & T2_INT_ON);
    WriteTimer2(0);
    OpenTimer2( T2_ON & T2_IDLE_STOP & T2_GATE_OFF & T2_PS_1_1 & T2_SOURCE_INT,
                PR_T2);

    // Timer4 configuration (10hz).
    ConfigIntTimer4(T4_INT_PRIOR_1 & T4_INT_ON);
    WriteTimer4(0);
    OpenTimer4( T4_ON & T4_IDLE_STOP & T4_GATE_OFF & T4_PS_1_256 &
                T4_SOURCE_INT, PR_T4);

    // SPI configuration.
    CloseSPI1();
    ConfigIntSPI1(SPI_INT_EN & SPI_INT_PRI_7);
    OpenSPI1(FRAME_ENABLE_OFF & FRAME_SYNC_INPUT & ENABLE_SDO_PIN &
             SPI_MODE16_OFF & SPI_SMP_OFF & SPI_CKE_OFF & SLAVE_ENABLE_ON &
             MASTER_ENABLE_OFF & CLK_POL_ACTIVE_HIGH & SEC_PRESCAL_3_1 &
             PRI_PRESCAL_64_1, SPI_ENABLE & SPI_IDLE_CON & SPI_RX_OVFLOW_CLR);
    _SPIROV = 0;

    // PWM configuration.
    OpenTimer3( T3_ON & T3_IDLE_STOP & T3_GATE_OFF & T3_PS_1_1 & T3_SOURCE_INT,
                PR_T3);
    ConfigIntOC1(OC_INT_OFF & OC_INT_PRIOR_4);
    OpenOC1(OC_IDLE_CON & OC_TIMER3_SRC & OC_PWM_FAULT_PIN_DISABLE, 0, 0);
    SetDCOC1PWM(0);

    // ADC configuration.
    SetChanADC10(ADC_CH0_POS_SAMPLEA_AN0 & ADC_CH0_NEG_SAMPLEA_NVREF);
    ConfigIntADC10(ADC_INT_DISABLE);
    OpenADC10(  ADC_MODULE_ON & ADC_IDLE_CONTINUE & ADC_FORMAT_INTG &
                ADC_CLK_MANUAL & ADC_AUTO_SAMPLING_OFF & ADC_SAMPLE_INDIVIDUAL &
                ADC_SAMP_OFF,
                ADC_VREF_AVDD_AVSS & ADC_SCAN_OFF & ADC_CONVERT_CH0 &
                ADC_SAMPLES_PER_INT_1 & ADC_ALT_BUF_OFF & ADC_ALT_INPUT_OFF,
                ADC_SAMPLE_TIME_0 & ADC_CONV_CLK_SYSTEM & ADC_CONV_CLK_3Tcy,
                ENABLE_AN0_ANA & ENABLE_AN1_ANA,
                SKIP_SCAN_AN2 & SKIP_SCAN_AN3 & SKIP_SCAN_AN4 & SKIP_SCAN_AN5);

    // QEI configuration.
    ConfigIntQEI(QEI_INT_PRI_6 & QEI_INT_ENABLE);
    POSCNT = 0;
    MAXCNT = 0x7FFF;
    OpenQEI(QEI_IDLE_CON & QEI_INT_CLK & QEI_INDEX_RESET_DISABLE &
            QEI_CLK_PRESCALE_1 & QEI_GATED_ACC_DISABLE & QEI_INPUTS_NOSWAP &
            QEI_MODE_x4_MATCH & QEI_DIR_SEL_CNTRL,
            POS_CNT_ERR_INT_DISABLE & QEI_QE_CLK_DIVIDE_1_4 &
            QEI_QE_OUT_ENABLE & MATCH_INDEX_PHASEA_HIGH &
            MATCH_INDEX_PHASEB_HIGH);
    QEICONbits.UPDN = 1;

    ConfigINT2( RISING_EDGE_INT & EXT_INT_ENABLE & EXT_INT_PRI_6 &
                GLOBAL_INT_ENABLE);
}

void InitFonctionRecep(void)
{
    g_SPI_RX_NbData[COM_W_CONSIGNE] = DATA_W_CONSIGNE;
    g_SPI_RX_NbData[COM_W_MODE] = DATA_W_MODE;
    g_SPI_RX_NbData[COM_W_POSITION] = DATA_W_POSITION;
    g_SPI_RX_NbData[COM_W_CONFIG] = DATA_W_CONFIG;
    g_SPI_RX_NbData[COM_R_POSITION] = DATA_R_POSITION;
    g_SPI_RX_NbData[COM_R_VITESSE] = DATA_R_VITESSE;
    g_SPI_RX_NbData[COM_R_ACCELERATION] = DATA_R_ACCELERATION;
    g_SPI_RX_NbData[COM_R_COURANT] = DATA_R_COURANT;
    g_SPI_RX_NbData[COM_R_ERREUR] = DATA_R_ERREUR;
    g_SPI_RX_NbData[COM_R_CONFIG] = DATA_R_CONFIG;
    g_SPI_RX_NbData[COM_R_ID_BOARD] = DATA_R_ID_BOARD;
    g_SPI_RX_NbData[COM_W_MOT_CONF] = DATA_W_MOT_CONF;
    g_SPI_RX_NbData[COM_R_MOT_CONF] = DATA_R_MOT_CONF;
}

void InitDriver(void)
{
    DRIVER_MODE = 0;
    DRIVER_COAST = 0;
    DRIVER_DIR = 1;
}

void InitVariable(void)
{
    g_mode = MODE_STOP;
    g_mode_mem = MODE_OPEN;

    InitSharedVarU16(&u16MesuredCurrent, 0);
    
    InitSharedVarU16(&u16KpNum, 1);
    InitSharedVarU16(&u16KpDenum, 2000);
    
    InitSharedVarU16(&u16KiNum, 1);
    InitSharedVarU16(&u16KiDenum, 4500);

    InitSharedVarU16(&u16KdNum, 1);
    InitSharedVarU16(&u16KdDenum, 1);
    
    g_interface_mesure_vitesse_SPI = IV_TACHY;

    InitSharedVarU16(&u16MesuredCurrent, 0);
    InitSharedVarS16(&s16MesuredAcceleration, 0);
    InitSharedVarS16(&s16MesuredSpeed, 0);
    InitSharedVarS16(&s16SetpointSpeed, 0);
    InitSharedVarS32(&s32MotorPosition, 0);

    InitSharedVarU16(&u16ConfCodeurPPT, 0);
    InitSharedVarU16(&u16ConfMaxCurrent, 2330);
    InitSharedVarS16(&s16ConfMaxSpeed, 111);

    InitSharedVarS32(&s32IErrorMax, 0);
}

unsigned int read_analog_channel(int channel)
{
    SetChanADC10(channel);  // Select the requested channel.
    ADCON1bits.SAMP = 1;    // Start sampling.
    __delay32(30);          // 1us delay @ 30 MIPS.
    ConvertADC10();         // Start converting.
    while (BusyADC10());
    return ReadADC10(0);
}

void WriteSPI1_c(unsigned char c)
{
    WriteSPI1(c);
    while (SPI1STATbits.SPITBF);
}

unsigned int received_spi1(void)
{
    return SPI1BUF;
}

void send_spi1(unsigned int mtdata)
{
    SPI1BUF = mtdata;
}

void Gestion_RW_Wconsigne(void)
{
    if (g_SPI_RX_flag == 1)
    {
        ReadSharedVarS32_SPI(&s32MotorPosition);
        s16SetpointSpeed.s16_data_SPI = (signed int) ((((unsigned int) g_SPI_RX_clearReg) << 8)&0xFF00);
        send_spi1((unsigned char) (s32MotorPosition.s32_data_SPI >> 24));
    }
    else if (g_SPI_RX_flag == 2)
    {
        s16SetpointSpeed.s16_data_SPI |= (signed int) (((unsigned int) g_SPI_RX_clearReg)&0x00FF);
        send_spi1((unsigned char) (s32MotorPosition.s32_data_SPI >> 16));
        WriteSharedVarS16_SPI(&s16SetpointSpeed);
    }
    else if (g_SPI_RX_flag == 3)
    {
        send_spi1((unsigned char) (s32MotorPosition.s32_data_SPI >> 8));
    }
    else if (g_SPI_RX_flag == 4)
    {
        send_spi1((unsigned char) s32MotorPosition.s32_data_SPI);
    }
    else if (g_SPI_RX_flag == 5)
    {
        ReadSharedVarS16_SPI(&s16MesuredSpeed);
        send_spi1((unsigned char) (s16MesuredSpeed.s16_data_SPI >> 8));
    }
    else if (g_SPI_RX_flag == 6)
    {
        send_spi1((unsigned char) (s16MesuredSpeed.s16_data_SPI));
    }
    else if (g_SPI_RX_flag == 7)
    {
        ReadSharedVarS16_SPI(&s16MesuredAcceleration);
        send_spi1((unsigned char) (s16MesuredAcceleration.s16_data_SPI >> 8));
    }
    else if (g_SPI_RX_flag == 8)
    {
        send_spi1((unsigned char) (s16MesuredAcceleration.s16_data_SPI));
    }
    else if (g_SPI_RX_flag == 9)
    {
        ReadSharedVarU16_SPI(&u16MesuredCurrent);
        send_spi1((unsigned char) (u16MesuredCurrent.u16_data_SPI >> 8));
    }
    else if (g_SPI_RX_flag == 10)
    {
        send_spi1((unsigned char) (u16MesuredCurrent.u16_data_SPI));
    }
    else if (g_SPI_RX_flag == 11)
    {
        send_spi1(ACK_SLAVE);
    }
}

void Gestion_RW_Wmode(void)
{
    if (g_SPI_RX_flag == 1)
    {
        g_mode = g_SPI_RX_clearReg;
        send_spi1(ACK_SLAVE);
    }
}

void Gestion_RW_Wposition(void)
{
    if (g_SPI_RX_flag == 1)
    {
        s32MotorPosition.s32_data_SPI = (signed long) ((((unsigned long) g_SPI_RX_clearReg) << 24)&0xFF000000);
        send_spi1(0);
    }
    else if (g_SPI_RX_flag == 2)
    {
        s32MotorPosition.s32_data_SPI |= (signed long) ((((unsigned long) g_SPI_RX_clearReg) << 16)&0x00FF0000);
        send_spi1(0);
    }
    else if (g_SPI_RX_flag == 3)
    {
        s32MotorPosition.s32_data_SPI |= (signed long) ((((unsigned long) g_SPI_RX_clearReg) << 8)&0x0000FF00);
        send_spi1(0);
    }
    else if (g_SPI_RX_flag == 4)
    {
        s32MotorPosition.s32_data_SPI |= (signed long) (((unsigned long) g_SPI_RX_clearReg)&0x000000FF);
        send_spi1(ACK_SLAVE);
        WriteSharedVarS32_SPI(&s32MotorPosition);
    }
}

void Gestion_RW_Wconfig(void)
{
    if (g_SPI_RX_flag == 1)
    {
        u16KpNum.u16_data_SPI= (unsigned int) ((((unsigned int) g_SPI_RX_clearReg) << 8)&0xFF00);
        send_spi1(0);
    }
    else if (g_SPI_RX_flag == 2)
    {
        u16KpNum.u16_data_SPI |= (unsigned int) (((unsigned int) g_SPI_RX_clearReg)&0x00FF);
        send_spi1(0);
        WriteSharedVarU16_SPI(&u16KpNum);
    }
    else if (g_SPI_RX_flag == 3)
    {
        u16KpDenum.u16_data_SPI = (unsigned int) ((((unsigned int) g_SPI_RX_clearReg) << 8)&0xFF00);
        send_spi1(0);
    }
    else if (g_SPI_RX_flag == 4)
    {
        u16KpDenum.u16_data_SPI |= (unsigned int) (((unsigned int) g_SPI_RX_clearReg)&0x00FF);
        send_spi1(0);
        WriteSharedVarU16_SPI(&u16KpDenum);
    }
    else if (g_SPI_RX_flag == 5)
    {
        u16KiNum.u16_data_SPI = (unsigned int) ((((unsigned int) g_SPI_RX_clearReg) << 8)&0xFF00);
        send_spi1(0);
    }
    else if (g_SPI_RX_flag == 6)
    {
        u16KiNum.u16_data_SPI |= (unsigned int) (((unsigned int) g_SPI_RX_clearReg)&0x00FF);
        send_spi1(0);
        WriteSharedVarU16_SPI(&u16KiNum);
    }
    else if (g_SPI_RX_flag == 7)
    {
        u16KiDenum.u16_data_SPI = (unsigned int) ((((unsigned int) g_SPI_RX_clearReg) << 8)&0xFF00);
        send_spi1(0);
    }
    else if (g_SPI_RX_flag == 8)
    {
        u16KiDenum.u16_data_SPI |= (unsigned int) (((unsigned int) g_SPI_RX_clearReg)&0x00FF);
        send_spi1(0);
        WriteSharedVarU16_SPI(&u16KiDenum);
    }
    else if (g_SPI_RX_flag == 9)
    {
        u16KdNum.u16_data_SPI = (unsigned int) ((((unsigned int) g_SPI_RX_clearReg) << 8)&0xFF00);
        send_spi1(0);
    }
    else if (g_SPI_RX_flag == 10)
    {
        u16KdNum.u16_data_SPI |= (unsigned int) (((unsigned int) g_SPI_RX_clearReg)&0x00FF);
        send_spi1(0);
        WriteSharedVarU16_SPI(&u16KdNum);
    }
    else if (g_SPI_RX_flag == 11)
    {
        u16KdDenum.u16_data_SPI = (unsigned int) ((((unsigned int) g_SPI_RX_clearReg) << 8)&0xFF00);
        send_spi1(0);
    }
    else if (g_SPI_RX_flag == 12)
    {
        u16KdDenum.u16_data_SPI |= (unsigned int) (((unsigned int) g_SPI_RX_clearReg)&0x00FF);
        send_spi1(0);
        WriteSharedVarU16_SPI(&u16KdDenum);
    }
    else if (g_SPI_RX_flag == 13)
    {
        s32IErrorMax.s32_data_SPI = (signed long) ((((unsigned long) g_SPI_RX_clearReg) << 24)&0xFF000000);
        send_spi1(0);
    }
    else if (g_SPI_RX_flag == 14)
    {
        s32IErrorMax.s32_data_SPI |= (signed long) ((((unsigned long) g_SPI_RX_clearReg) << 16)&0x00FF0000);
        send_spi1(0);
    }
    else if (g_SPI_RX_flag == 15)
    {
        s32IErrorMax.s32_data_SPI |= (signed long) ((((unsigned long) g_SPI_RX_clearReg) << 8)&0x0000FF00);
        send_spi1(0);
    }
    else if (g_SPI_RX_flag == 16)
    {
        s32IErrorMax.s32_data_SPI |= (signed long) (((unsigned long) g_SPI_RX_clearReg)&0x000000FF);
        send_spi1(0);
        WriteSharedVarS32_SPI(&s32IErrorMax);
    }
    else if (g_SPI_RX_flag == 17)
    {
        g_TimeOut_Recep = g_SPI_RX_clearReg;
        send_spi1(0);
    }
    else if (g_SPI_RX_flag == 18)
    {
        g_timeMesureSpeed = g_SPI_RX_clearReg;
        send_spi1(ACK_SLAVE);
        g_multi_timeMesureSpeed_s = 1000 / ((unsigned int) g_timeMesureSpeed);
        g_timeControlLoop = g_timeMesureSpeed;
    }
}

void Gestion_RW_Rposition(void)
{
    if (g_SPI_RX_flag == 1)
    {
        ReadSharedVarS32_SPI(&s32MotorPosition);
        send_spi1((unsigned char) (s32MotorPosition.s32_data_SPI >> 24));
    }
    else if (g_SPI_RX_flag == 2)
    {
        send_spi1((unsigned char) (s32MotorPosition.s32_data_SPI >> 16));
    }
    else if (g_SPI_RX_flag == 3)
    {
        send_spi1((unsigned char) (s32MotorPosition.s32_data_SPI >> 8));
    }
    else if (g_SPI_RX_flag == 4)
    {
        send_spi1((unsigned char) s32MotorPosition.s32_data_SPI);
    }
    else if (g_SPI_RX_flag == 5)
    {
        send_spi1(ACK_SLAVE);
    }
}

void Gestion_RW_Rvitesse(void)
{
    if (g_SPI_RX_flag == 1)
    {
        ReadSharedVarS16_SPI(&s16MesuredSpeed);
        send_spi1((unsigned char) (s16MesuredSpeed.s16_data_SPI >> 8));
    }
    else if (g_SPI_RX_flag == 2)
    {
        send_spi1((unsigned char) s16MesuredSpeed.s16_data_SPI);
    }
    else if (g_SPI_RX_flag == 3)
    {
        send_spi1(ACK_SLAVE);
    }
}

void Gestion_RW_Racceleration(void)
{
    if (g_SPI_RX_flag == 1)
    {
        ReadSharedVarS16_SPI(&s16MesuredAcceleration);
        send_spi1((unsigned char) (s16MesuredAcceleration.s16_data_SPI >> 8));
    }
    else if (g_SPI_RX_flag == 2)
    {
        send_spi1((unsigned char) s16MesuredAcceleration.s16_data_SPI);
    }
    else if (g_SPI_RX_flag == 3)
    {
        send_spi1(ACK_SLAVE);
    }
}

void Gestion_RW_Rcourant(void)
{
    if (g_SPI_RX_flag == 1)
    {
        ReadSharedVarU16_SPI(&u16MesuredCurrent);
        send_spi1((unsigned char) (u16MesuredCurrent.u16_data_SPI >> 8));
    }
    else if (g_SPI_RX_flag == 2)
    {
        send_spi1((unsigned char) u16MesuredCurrent.u16_data_SPI);
    }
    else if (g_SPI_RX_flag == 3)
    {
        send_spi1(ACK_SLAVE);
    }
}

void Gestion_RW_Rerreur(void)
{
    if (g_SPI_RX_flag == 1)
    {
        send_spi1(ACK_SLAVE);
    }
}

void Gestion_RW_Rconfig(void)
{
    if (g_SPI_RX_flag == 1)
    {
        ReadSharedVarU16_SPI(&u16KpNum);
        send_spi1((unsigned char) (u16KpNum.u16_data_SPI >> 8));
    }
    else if (g_SPI_RX_flag == 2)
    {
        send_spi1((unsigned char) (u16KpNum.u16_data_SPI));
    }
    else if (g_SPI_RX_flag == 3)
    {
        ReadSharedVarU16_SPI(&u16KpDenum);
        send_spi1((unsigned char) (u16KpDenum.u16_data_SPI >> 8));
    }
    else if (g_SPI_RX_flag == 4)
    {
        send_spi1((unsigned char) (u16KpDenum.u16_data_SPI));
    }
    else if (g_SPI_RX_flag == 5)
    {
        ReadSharedVarU16_SPI(&u16KiNum);
        send_spi1((unsigned char) (u16KiNum.u16_data_SPI >> 8));
    }
    else if (g_SPI_RX_flag == 6)
    {
        send_spi1((unsigned char) (u16KiNum.u16_data_SPI));
    }
    else if (g_SPI_RX_flag == 7)
    {
        ReadSharedVarU16_SPI(&u16KiDenum);
        send_spi1((unsigned char) (u16KiDenum.u16_data_SPI >> 8));
    }
    else if (g_SPI_RX_flag == 8)
    {
        send_spi1((unsigned char) (u16KiDenum.u16_data_SPI));
    }
    else if (g_SPI_RX_flag == 9)
    {
        ReadSharedVarU16_SPI(&u16KdNum);
        send_spi1((unsigned char) (u16KdNum.u16_data_SPI >> 8));
    }
    else if (g_SPI_RX_flag == 10)
    {
        send_spi1((unsigned char) (u16KdNum.u16_data_SPI));
    }
    else if (g_SPI_RX_flag == 11)
    {
        ReadSharedVarU16_SPI(&u16KdDenum);
        send_spi1((unsigned char) (u16KdDenum.u16_data_SPI >> 8));
    }
    else if (g_SPI_RX_flag == 12)
    {
        send_spi1((unsigned char) (u16KdDenum.u16_data_SPI));
    }
    else if (g_SPI_RX_flag == 13)
    {
        ReadSharedVarS32_SPI(&s32IErrorMax);
        send_spi1((unsigned char) (s32IErrorMax.s32_data_SPI >> 24));
    }
    else if (g_SPI_RX_flag == 14)
    {
        send_spi1((unsigned char) (s32IErrorMax.s32_data_SPI >> 16));
    }
    else if (g_SPI_RX_flag == 15)
    {
        send_spi1((unsigned char) (s32IErrorMax.s32_data_SPI >> 8));
    }
    else if (g_SPI_RX_flag == 16)
    {
        send_spi1((unsigned char) (s32IErrorMax.s32_data_SPI));
    }
    else if (g_SPI_RX_flag == 17)
    {
        send_spi1((unsigned char) (g_TimeOut_Recep));
    }
    else if (g_SPI_RX_flag == 18)
    {
        send_spi1((unsigned char) (g_timeMesureSpeed));
    }
    else if (g_SPI_RX_flag == 19)
    {
        send_spi1(ACK_SLAVE);
    }
}

void Gestion_RW_Rid(void)
{
    if (g_SPI_RX_flag == 1)
    {
        send_spi1(TYPE_CARD);
    }
    else if (g_SPI_RX_flag == 2)
    {
        send_spi1((unsigned char) (g_ID_board >> 24));
    }
    else if (g_SPI_RX_flag == 3)
    {
        send_spi1((unsigned char) (g_ID_board >> 16));
    }
    else if (g_SPI_RX_flag == 4)
    {
        send_spi1((unsigned char) (g_ID_board >> 8));
    }
    else if (g_SPI_RX_flag == 5)
    {
        send_spi1((unsigned char) g_ID_board);
    }
    else if (g_SPI_RX_flag == 6)
    {
        send_spi1(ACK_SLAVE);
    }
}

void Gestion_RW_Wmot_conf(void)
{
    if (g_SPI_RX_flag == 1)
    {
        g_interface_mesure_vitesse_SPI = g_SPI_RX_clearReg;
        send_spi1(0);
    }
    else if (g_SPI_RX_flag == 2)
    {
        u16ConfCodeurPPT.u16_data_SPI = (unsigned int) ((((unsigned int) g_SPI_RX_clearReg) << 8)&0xFF00);
        send_spi1(0);
    }
    else if (g_SPI_RX_flag == 3)
    {
        u16ConfCodeurPPT.u16_data_SPI |= (unsigned int) (((unsigned int) g_SPI_RX_clearReg)&0x00FF);
        send_spi1(0);
        WriteSharedVarU16_SPI(&u16ConfCodeurPPT);
    }
    else if (g_SPI_RX_flag == 4)
    {
        s16ConfMaxSpeed.s16_data_SPI = (signed int) ((((unsigned int) g_SPI_RX_clearReg) << 8)&0xFF00);
        send_spi1(0);
    }
    else if (g_SPI_RX_flag == 5)
    {
        s16ConfMaxSpeed.s16_data_SPI |= (signed int) (((unsigned int) g_SPI_RX_clearReg)&0x00FF);
        send_spi1(0);
        WriteSharedVarS16_SPI(&s16ConfMaxSpeed);
    }
    else if (g_SPI_RX_flag == 6)
    {
        u16ConfMaxCurrent.u16_data_SPI = (unsigned int) ((((unsigned int) g_SPI_RX_clearReg) << 8)&0xFF00);
        send_spi1(0);
    }
    else if (g_SPI_RX_flag == 7)
    {
        u16ConfMaxCurrent.u16_data_SPI |= (unsigned int) (((unsigned int) g_SPI_RX_clearReg)&0x00FF);
        send_spi1(ACK_SLAVE);
        WriteSharedVarU16_SPI(&u16ConfMaxCurrent);
    }
}

void Gestion_RW_Rmot_conf(void)
{
    if (g_SPI_RX_flag == 1)
    {
        send_spi1(g_interface_mesure_vitesse_SPI);
    }
    else if (g_SPI_RX_flag == 2)
    {
        ReadSharedVarU16_SPI(&u16ConfCodeurPPT);
        send_spi1((unsigned char) (u16ConfCodeurPPT.u16_data_SPI >> 8));
    }
    else if (g_SPI_RX_flag == 3)
    {
        send_spi1((unsigned char) (u16ConfCodeurPPT.u16_data_SPI));
    }
    else if (g_SPI_RX_flag == 4)
    {
        ReadSharedVarS16_SPI(&s16ConfMaxSpeed);
        send_spi1((unsigned char) s16ConfMaxSpeed.s16_data_SPI >> 8);
    }
    else if (g_SPI_RX_flag == 5)
    {
        send_spi1((unsigned char) (s16ConfMaxSpeed.s16_data_SPI));
    }
    else if (g_SPI_RX_flag == 6)
    {
        ReadSharedVarU16_SPI(&u16ConfMaxCurrent);
        send_spi1((unsigned char) (u16ConfMaxCurrent.u16_data_SPI >> 8));
    }
    else if (g_SPI_RX_flag == 7)
    {
        send_spi1((unsigned char) (u16ConfMaxCurrent.u16_data_SPI));
    }
    else if (g_SPI_RX_flag == 8)
    {
        send_spi1(ACK_SLAVE);
    }
}

void Gestion_LED(unsigned int freq)
{
    if (freq != 0)
    {
        if (g_timer_led >= freq)
        {
            g_timer_led = 0;
            LED = ~LED;
        }
    }
}

void Gestion_Courant(void)
{
    if ((g_stateAcq == 0) && (g_timer_AcqCourant >= 20)) //cadencé à 100us
    {
        g_timer_AcqCourant = 0;
        SetChanADC10(ADC_CSOUT); // Select the requested channel
        ADCON1bits.SAMP = 1; // start sampling
        g_stateAcq = 1;
    }
    else if ((g_stateAcq == 1) && (g_timer_AcqCourant >= 1))
    {
        ConvertADC10(); // start Converting
        g_stateAcq = 2;
    }
    else if ((g_stateAcq == 2) && (ADCON1bits.DONE == 1))
    {
        u16MesuredCurrent.u16_data_APP += ReadADC10(0);
        g_nb_mes_courant++;

        g_stateAcq = 0;
    }
    else if ((g_stateAcq == 0) && (g_nb_mes_courant >= 5))
    {
        // Somme des mesures de courant effectué diviser par le nb de mesures
        u16MesuredCurrent.u16_data_APP /= (unsigned long) g_nb_mes_courant;
        // Reset des variables necessaires à la nouvelles serie d'acquisition
        WriteSharedVarU16_APP(&u16MesuredCurrent);
        u16MesuredCurrent.u16_data_APP = 0;
        g_nb_mes_courant = 0;
    }
}

// Int2 interrupt service routine --------------------------------------------//
void __attribute__((interrupt, no_auto_psv)) _INT2Interrupt(void)
{
    _INT2IF = 0;
    g_pulse_drive++;
}

// Timer 1 interrupt service routine -----------------------------------------//
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void)
{
    WriteTimer1(0);
    _T1IF = 0;
    g_timerSpeed++;
    g_timerControl++;

    if (g_timerSpeed >= g_timeMesureSpeed)
    {
        g_timerSpeed = 0;
        if (g_interface_mesure_vitesse_SPI == IV_CODEUR)
        {
            //recup position
            s32MotorPosition.s32_data_APP = (((signed long) g_motor_pos_H) << 15)+((signed long) ReadQEI());
            WriteSharedVarS32_APP(&s32MotorPosition);
            //Determine le nombre de pas depuis la derniere acquisition
            g_motor_vitesse = s32MotorPosition.s32_data_APP - g_motor_pos_mem;
            //ramène en pas par seconde
            g_motor_vitesse *= g_multi_timeMesureSpeed_s;
            //divise par 32 pour que ca entre dans int et le rende transportable
            ReadSharedVarU16_APP(&u16ConfCodeurPPT);
            s16MesuredSpeed.s16_data_APP = (signed int)(g_motor_vitesse / (signed long)u16ConfCodeurPPT.u16_data_APP);
            WriteSharedVarS16_APP(&s16MesuredSpeed);
            //Sauvegarde de la valeur à soustraire pour la prochain calcul
            g_motor_pos_mem = s32MotorPosition.s32_data_APP;

        }
        else if(g_interface_mesure_vitesse_SPI == IV_TACHY)
        {
            g_motor_vitesse = ((signed long) g_pulse_drive)*((signed long) g_multi_timeMesureSpeed_s);
            g_pulse_drive = 0;
            if (DRIVER_DIRO == 1)
                g_motor_vitesse *= -1;
            
            //RPS    driver = 24 pulses / tr
//            g_motor_vitesse /= 24;
            s16MesuredSpeed.s16_data_APP = (signed int)(g_motor_vitesse/24);// /24);          
            WriteSharedVarS16_APP(&s16MesuredSpeed);
        }
        else
        {
            g_Erreur_Cour++;
        }
    }

    if (g_timerControl >= g_timeControlLoop)
    {
        g_timerControl = 0;
        
        if (g_flag_asser == LOOP)
        {
            // Erreur Proportionnelle
            ReadSharedVarS16_APP(&s16SetpointSpeed);
            g_Erreur_P = ((signed long int)s16SetpointSpeed.s16_data_APP) - g_motor_vitesse;

            // Cumul Int�grale
            g_Erreur_I += g_Erreur_P;
            
            // Calcul Asservissement Kp
            ReadSharedVarU16_APP(&u16KpNum);
            ReadSharedVarU16_APP(&u16KpDenum);
            g_dutycycle = g_Erreur_P * u16KpNum.u16_data_APP / u16KpDenum.u16_data_APP;
            // Calcul Asservissement Ki
            ReadSharedVarU16_APP(&u16KiNum);
            ReadSharedVarU16_APP(&u16KiDenum);
            g_dutycycle += g_Erreur_I * u16KiNum.u16_data_APP / u16KiDenum.u16_data_APP;
            
            g_dutycycle *= (signed long int)PWM_VAL_CENTRE;
        }
        else
        {
            ReadSharedVarS16_APP(&s16SetpointSpeed);
            g_dutycycle  = ((signed long int)s16SetpointSpeed.s16_data_APP);
            g_dutycycle *= ((signed long int)PWM_VAL_CENTRE);           
        }

        g_dutycycle /= 1000;
        g_dutycycle += PWM_VAL_CENTRE;
        
        s16MesuredAcceleration.s16_data_APP = (unsigned int)g_dutycycle;
        WriteSharedVarS16_APP(&s16MesuredAcceleration);

        SetDCOC1PWM((unsigned int) (g_dutycycle));
    }
}

// Timer 2 interrupt service routine -----------------------------------------//
void __attribute__((interrupt, no_auto_psv)) _T2Interrupt(void)
{
    WriteTimer2(0);
    _T2IF = 0;
    //Timer reglé sur 100us
    g_timer_AcqCourant++;

}

// Timer 4 interrupt service routine -----------------------------------------//
void __attribute__((interrupt, no_auto_psv)) _T4Interrupt(void)
{
    WriteTimer4(0);
    _T4IF = 0;
    //Timer reglé sur 100ms
    g_timer_led++;
}

// This is Output Compare (PWM) receive interrupt service routine
//void __attribute__((interrupt, no_auto_psv)) _OC1Interrupt(void)
//{
//    IFS0bits.OC1IF = 0;
//}

// SPI receive interrupt service routine
void __attribute__((interrupt, no_auto_psv)) _SPI1Interrupt(void)
{
    _SPI1IF = 0;
    if (SPI1STATbits.SPIROV)
    {
        SPI1STATbits.SPIROV = 0;
        g_Erreur_Cour++;
    }

    while (DataRdySPI1())
    {
        g_SPI_RX_clearReg = SPI1BUF;
        if ((g_SPI_RX_flag == 0))
        {
            if ((g_SPI_RX_clearReg > 0)&&(g_SPI_RX_clearReg <= NB_FONCTION))
            {
                send_spi1(g_Erreur_Cour);
                g_SPI_RX_trame.commande = g_SPI_RX_clearReg;
                g_SPI_RX_flag++; // si oui incremente l'etape de gestion rx
            }
        }// si non verifie les autres condition
        else if (g_SPI_RX_flag >= 1 && g_SPI_RX_flag <= (g_SPI_RX_NbData[g_SPI_RX_trame.commande]))
        {
            switch (g_SPI_RX_trame.commande)
            {
                case COM_W_CONSIGNE:
                    Gestion_RW_Wconsigne();
                break;
                case COM_W_MODE:
                    Gestion_RW_Wmode();
                break;
                case COM_W_POSITION:
                    Gestion_RW_Wposition();
                break;
                case COM_W_CONFIG:
                    Gestion_RW_Wconfig();
                break;
                case COM_R_POSITION:
                    Gestion_RW_Rposition();
                break;
                case COM_R_VITESSE:
                    Gestion_RW_Rvitesse();
                break;
                case COM_R_COURANT:
                    Gestion_RW_Rcourant();
                break;
                case COM_R_ERREUR:
                    Gestion_RW_Rerreur();
                break;
                case COM_R_CONFIG:
                    Gestion_RW_Rconfig();
                break;
                case COM_R_ID_BOARD:
                    Gestion_RW_Rid();
                break;
                case COM_W_MOT_CONF:
                    Gestion_RW_Wmot_conf();
                break;
                case COM_R_MOT_CONF:
                    Gestion_RW_Rmot_conf();
                break;
            }

            g_SPI_RX_flag++; // increment du flag
        }
        else if ((g_SPI_RX_flag == (g_SPI_RX_NbData[g_SPI_RX_trame.commande] + 1)) && g_SPI_RX_clearReg == ACK_MASTER)
        {
            send_spi1(0); // si le dernier octet est le ender la trame est bonne
            //            g_SPI_RX_trame_valide=1;
            g_SPI_RX_flag = 0;
        }
        else
        {
            send_spi1(0);
            g_SPI_RX_flag = 0;
        }
    }
}

void __attribute__((interrupt, no_auto_psv)) _QEIInterrupt(void)
{
    _QEIIF = 0; /* Clear QEI interrupt flag */
    if (QEICONbits.UPDN)g_motor_pos_H++;
    else g_motor_pos_H--;
}
