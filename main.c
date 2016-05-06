#include <libpic30.h>
#include <p30F4012.h>

#include <stdio.h>

#include "Config.h"
#include "main.h"
#include "SharedVarLib.h"

#include <timer.h>
#include <uart.h>
#include <outcompare.h>
#include <adc10.h>
#include <qei.h>
#include <spi.h>
#include <ports.h>

unsigned long g_ID_board = 43981;

//variables de Timer
unsigned int g_timerSpeed = 0, g_timer_AcqCourant = 0, g_timer_led = 0;
unsigned int g_timerControl = 0;

unsigned int g_multi_timeMesureSpeed_s = 0;
unsigned char g_flag_asser = OPEN;
unsigned char g_var_led = LED_FREQ_5HZ;
unsigned char g_interface_mesure_vitesse = CODEUR_OFF;

trame_SPI g_SPI_RX_trame;
unsigned char g_SPI_RX_flag = 0, g_SPI_RX_trame_valide = 0, g_SPI_RX_clearReg = 0;
unsigned char g_SPI_RX_NbData[NB_FONCTION + 1];


signed int g_motor_pos_H = 0;
signed long g_motor_pos_mem = 0;
signed long g_motor_vitesse = 0;

signed long g_dutycycle = 0;
signed long g_pulse_drive = 0;

unsigned long g_mesure_courant = 0;
unsigned char g_stateAcq = 0, g_nb_mes_courant = 0;

signed long g_Erreur_P = 0, g_Erreur_I = 0;

unsigned char g_mode = MODE_STOP, g_mode_mem = MODE_OPEN; //Initialisation du Mode (Mode_mem différent pour lancer l'init du mode Stop)

signed long g_motor_pos = 0;
signed long g_vitesse_desiree = 0;
signed int g_Vitesse_Cour = 0;


unsigned char g_timeMesureSpeed = 0;
unsigned char g_timeControlLoop = 0;

unsigned char g_TimeOut_Recep = 0;
unsigned char g_Erreur_Cour = 0x00;

unsigned int g_nom_Kp = 1;
unsigned int g_denom_Kp = 2000;
unsigned int g_nom_Ki = 1;
unsigned int g_denom_Ki = 4500;
unsigned int g_nom_Kd = 1;
unsigned int g_denom_Kd = 1;

signed long g_erreur_limite_I = 0;

signed long g_motor_conf_codeur_ppt = 0;
unsigned int g_motor_conf_courant_max = 2330;
signed long g_motor_conf_vitesse_max = 111;


unsigned int g_mesure_courant_moy_SPI = 0; 
unsigned int g_Acceleration_Cour_SPI = 0;
signed int g_Vitesse_Cour_SPI = 0;


//Variables partagées info fonctionnement moteur
u16_shared_var u16CurrentValue;
u16_shared_var u16AccelerationCour;
s16_shared_var s16VitesseCour;

//Variables partagées info fonctionnement moteur
//Variables partagées parametres PID
//Variables partagées parametres Moteur

int main() {
    // Set up which pins are which
    Configure_pins();

    InitDriver();

    InitFonctionRecep();

    //Récupération des parametres dans l'eeprom    
    
    WriteSharedVarU16_APP(&u16CurrentValue, 0);

    g_timeMesureSpeed = 10;
    g_multi_timeMesureSpeed_s = 1000 / ((unsigned int) g_timeMesureSpeed);
    g_timeControlLoop = g_timeMesureSpeed;

    LED = LED_OFF;

    while (1) {
        Gestion_LED(g_var_led);

        Gestion_Courant();

        if (g_mode != g_mode_mem) {
            switch (g_mode) {
                case MODE_STOP:
                    DRIVER_COAST =0;          //Mise off du Driver
                    SetDCOC1PWM(PWM_VAL_CENTRE);
                    g_flag_asser = STOP;
                    g_var_led = LED_FREQ_1HZ;
                    break;
                case MODE_OPEN:
                    DRIVER_COAST =1;          //Mise on du Driver
                    g_flag_asser = OPEN;
                    g_var_led = LED_FREQ_2HZ;
                    break;
                case MODE_ASSER:
                    DRIVER_COAST =1;          //Mise on du Driver
                    g_flag_asser = LOOP;
                    g_var_led = LED_FREQ_5HZ;
                    break;
                default:
                    DRIVER_COAST =0;          //Mise off du Driver
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

void Configure_pins() {
    ADPCFG = 0xFFFC;
    // Configure digital I/O
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

    //      Config & Réglage Timer1 à 1khz
    ConfigIntTimer1(T1_INT_PRIOR_5 & T1_INT_ON);
    WriteTimer1(0);
    OpenTimer1(T1_ON & T1_IDLE_STOP & T1_GATE_OFF & T1_PS_1_1 & T1_SYNC_EXT_OFF & T1_SOURCE_INT, PR_T1);

    //      Config & Réglage Timer2 à 10khz
    ConfigIntTimer2(T2_INT_PRIOR_3 & T2_INT_ON);
    WriteTimer2(0);
    OpenTimer2(T2_ON & T2_IDLE_STOP & T2_GATE_OFF & T2_PS_1_1 & T2_SOURCE_INT, PR_T2);

    //      Config & Réglage Timer4 à 10hz
    ConfigIntTimer4(T4_INT_PRIOR_1 & T4_INT_ON);
    WriteTimer4(0);
    OpenTimer4(T4_ON & T4_IDLE_STOP & T4_GATE_OFF & T4_PS_1_256 & T4_SOURCE_INT, PR_T4);

    //      Config SPI   
    CloseSPI1();
    ConfigIntSPI1(SPI_INT_EN & SPI_INT_PRI_7);
    OpenSPI1(FRAME_ENABLE_OFF & FRAME_SYNC_INPUT & ENABLE_SDO_PIN &
            SPI_MODE16_OFF & SPI_SMP_OFF & SPI_CKE_OFF & SLAVE_ENABLE_ON &
            MASTER_ENABLE_OFF & CLK_POL_ACTIVE_HIGH & SEC_PRESCAL_3_1 &
            PRI_PRESCAL_64_1,
            SPI_ENABLE & SPI_IDLE_CON & SPI_RX_OVFLOW_CLR);
    _SPIROV = 0;

    //      Config PWM
    OpenTimer3(T3_ON & T3_IDLE_STOP & T3_GATE_OFF & T3_PS_1_1 & T3_SOURCE_INT, PR_T3);
    ConfigIntOC1(OC_INT_OFF & OC_INT_PRIOR_4);
    OpenOC1(OC_IDLE_CON & OC_TIMER3_SRC & OC_PWM_FAULT_PIN_DISABLE, 0, 0);
    SetDCOC1PWM(0);

    //      Config ADC
    SetChanADC10(ADC_CH0_POS_SAMPLEA_AN0 & ADC_CH0_NEG_SAMPLEA_NVREF);
    ConfigIntADC10(ADC_INT_DISABLE);
    OpenADC10(ADC_MODULE_ON & ADC_IDLE_CONTINUE & ADC_FORMAT_INTG & ADC_CLK_MANUAL & ADC_AUTO_SAMPLING_OFF & ADC_SAMPLE_INDIVIDUAL & ADC_SAMP_OFF,
            ADC_VREF_AVDD_AVSS & ADC_SCAN_OFF & ADC_CONVERT_CH0 & ADC_SAMPLES_PER_INT_1 & ADC_ALT_BUF_OFF & ADC_ALT_INPUT_OFF,
            ADC_SAMPLE_TIME_0 & ADC_CONV_CLK_SYSTEM & ADC_CONV_CLK_3Tcy,
            ENABLE_AN0_ANA & ENABLE_AN1_ANA,
            SKIP_SCAN_AN2 & SKIP_SCAN_AN3 & SKIP_SCAN_AN4 & SKIP_SCAN_AN5);

    //      Config QEI

    ConfigIntQEI(QEI_INT_PRI_6 & QEI_INT_ENABLE);
    POSCNT = 0;
    MAXCNT = 0x7FFF;
    OpenQEI(QEI_IDLE_CON &
            QEI_INT_CLK &
            QEI_INDEX_RESET_DISABLE &
            QEI_CLK_PRESCALE_1 &
            QEI_GATED_ACC_DISABLE &
            QEI_INPUTS_NOSWAP &
            QEI_MODE_x4_MATCH &
            QEI_DIR_SEL_CNTRL,
            POS_CNT_ERR_INT_DISABLE &
            QEI_QE_CLK_DIVIDE_1_4 &
            QEI_QE_OUT_ENABLE &
            MATCH_INDEX_PHASEA_HIGH &
            MATCH_INDEX_PHASEB_HIGH);
    QEICONbits.UPDN = 1;

    ConfigINT2(RISING_EDGE_INT &
            EXT_INT_ENABLE &
            EXT_INT_PRI_6 &
            GLOBAL_INT_ENABLE);
}

void InitFonctionRecep(void) {
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

void InitDriver(void) {
    DRIVER_MODE = 0;
    DRIVER_COAST = 0;
    DRIVER_DIR = 1;
}

unsigned int read_analog_channel(int channel) {
    SetChanADC10(channel); // Select the requested channel
    ADCON1bits.SAMP = 1; // start sampling
    __delay32(30); // 1us delay @ 30 MIPS
    ConvertADC10(); // start Converting
    while (BusyADC10());
    return ReadADC10(0);
}

void WriteSPI1_c(unsigned char c) {
    WriteSPI1(c);
    while (SPI1STATbits.SPITBF);
}

unsigned int received_spi1(void) {
    return SPI1BUF;
}

void send_spi1(unsigned int mtdata) {
    SPI1BUF = mtdata;
}

void Gestion_RW_Wconsigne(void) {
    if (g_SPI_RX_flag == 1) {
        g_vitesse_desiree = (signed long) ((signed int) ((((unsigned int) g_SPI_RX_clearReg) << 8)&0xFF00));
        send_spi1((unsigned char) (g_motor_pos >> 24));
    } else if (g_SPI_RX_flag == 2) {
        g_vitesse_desiree |= (signed long) ((signed int) (((unsigned int) g_SPI_RX_clearReg)&0x00FF));
        send_spi1((unsigned char) (g_motor_pos >> 16));
    } else if (g_SPI_RX_flag == 3) {
        send_spi1((unsigned char) (g_motor_pos >> 8));
    } else if (g_SPI_RX_flag == 4) {
        send_spi1((unsigned char) g_motor_pos);
    } else if (g_SPI_RX_flag == 5) {
        g_Vitesse_Cour_SPI = ReadSharedVarS16_SPI(&s16VitesseCour);
        send_spi1((unsigned char) (g_Vitesse_Cour_SPI >> 8));
    } else if (g_SPI_RX_flag == 6) {
        send_spi1((unsigned char) (g_Vitesse_Cour_SPI));
    } else if (g_SPI_RX_flag == 7) {
        g_Acceleration_Cour_SPI = ReadSharedVarU16_SPI(&u16AccelerationCour);
        send_spi1((unsigned char) (g_Acceleration_Cour_SPI >> 8));
    } else if (g_SPI_RX_flag == 8) {
        send_spi1((unsigned char) (g_Acceleration_Cour_SPI));
    } else if (g_SPI_RX_flag == 9) {
        g_mesure_courant_moy_SPI = ReadSharedVarU16_SPI(&u16CurrentValue);
        send_spi1((unsigned char) (g_mesure_courant_moy_SPI >> 8));
    } else if (g_SPI_RX_flag == 10) {
        send_spi1((unsigned char) (g_mesure_courant_moy_SPI));
    } else if (g_SPI_RX_flag == 11) {
        send_spi1(ACK_SLAVE);
    }
}

void Gestion_RW_Wmode(void) {
    if (g_SPI_RX_flag == 1) {
        g_mode = g_SPI_RX_clearReg;
        send_spi1(ACK_SLAVE);
    }
}

void Gestion_RW_Wposition(void) {
    if (g_SPI_RX_flag == 1) {
        g_motor_pos = (signed long) ((((unsigned long) g_SPI_RX_clearReg) << 24)&0xFF000000);
        send_spi1(0);
    } else if (g_SPI_RX_flag == 2) {
        g_motor_pos |= (signed long) ((((unsigned long) g_SPI_RX_clearReg) << 16)&0x00FF0000);
        send_spi1(0);
    } else if (g_SPI_RX_flag == 3) {
        g_motor_pos |= (signed long) ((((unsigned long) g_SPI_RX_clearReg) << 8)&0x0000FF00);
        send_spi1(0);
    } else if (g_SPI_RX_flag == 4) {
        g_motor_pos |= (signed long) (((unsigned long) g_SPI_RX_clearReg)&0x000000FF);
        send_spi1(ACK_SLAVE);
    }
}

void Gestion_RW_Wconfig(void) {
    if (g_SPI_RX_flag == 1) {
        g_nom_Kp = (unsigned int) ((((unsigned int) g_SPI_RX_clearReg) << 8)&0xFF00);
        send_spi1(0);
    } else if (g_SPI_RX_flag == 2) {
        g_nom_Kp |= (unsigned int) (((unsigned int) g_SPI_RX_clearReg)&0x00FF);
        send_spi1(0);
    } else if (g_SPI_RX_flag == 3) {
        g_denom_Kp = (unsigned int) ((((unsigned int) g_SPI_RX_clearReg) << 8)&0xFF00);
        send_spi1(0);
    } else if (g_SPI_RX_flag == 4) {
        g_denom_Kp |= (unsigned int) (((unsigned int) g_SPI_RX_clearReg)&0x00FF);
        send_spi1(0);
    } else if (g_SPI_RX_flag == 5) {
        g_nom_Ki = (unsigned int) ((((unsigned int) g_SPI_RX_clearReg) << 8)&0xFF00);
        send_spi1(0);
    } else if (g_SPI_RX_flag == 6) {
        g_nom_Ki |= (unsigned int) (((unsigned int) g_SPI_RX_clearReg)&0x00FF);
        send_spi1(0);
    } else if (g_SPI_RX_flag == 7) {
        g_denom_Ki = (unsigned int) ((((unsigned int) g_SPI_RX_clearReg) << 8)&0xFF00);
        send_spi1(0);
    } else if (g_SPI_RX_flag == 8) {
        g_denom_Ki |= (unsigned int) (((unsigned int) g_SPI_RX_clearReg)&0x00FF);
        send_spi1(0);
    } else if (g_SPI_RX_flag == 9) {
        g_nom_Kd = (unsigned int) ((((unsigned int) g_SPI_RX_clearReg) << 8)&0xFF00);
        send_spi1(0);
    } else if (g_SPI_RX_flag == 10) {
        g_nom_Kd |= (unsigned int) (((unsigned int) g_SPI_RX_clearReg)&0x00FF);
        send_spi1(0);
    } else if (g_SPI_RX_flag == 11) {
        g_denom_Kd = (unsigned int) ((((unsigned int) g_SPI_RX_clearReg) << 8)&0xFF00);
        send_spi1(0);
    } else if (g_SPI_RX_flag == 12) {
        g_denom_Kd |= (unsigned int) (((unsigned int) g_SPI_RX_clearReg)&0x00FF);
        send_spi1(0);
    } else if (g_SPI_RX_flag == 13) {
        g_erreur_limite_I = (signed long) ((((unsigned long) g_SPI_RX_clearReg) << 24)&0xFF000000);
        send_spi1(0);
    } else if (g_SPI_RX_flag == 14) {
        g_erreur_limite_I |= (signed long) ((((unsigned long) g_SPI_RX_clearReg) << 16)&0x00FF0000);
        send_spi1(0);
    } else if (g_SPI_RX_flag == 15) {
        g_erreur_limite_I |= (signed long) ((((unsigned long) g_SPI_RX_clearReg) << 8)&0x0000FF00);
        send_spi1(0);
    } else if (g_SPI_RX_flag == 16) {
        g_erreur_limite_I |= (signed long) (((unsigned long) g_SPI_RX_clearReg)&0x000000FF);
        send_spi1(0);
    } else if (g_SPI_RX_flag == 17) {
        g_TimeOut_Recep = g_SPI_RX_clearReg;
        send_spi1(0);
    } else if (g_SPI_RX_flag == 18) {
        g_timeMesureSpeed = g_SPI_RX_clearReg;
        send_spi1(ACK_SLAVE);
        g_multi_timeMesureSpeed_s = 1000 / ((unsigned int) g_timeMesureSpeed);
        g_timeControlLoop = g_timeMesureSpeed;
    }
}

void Gestion_RW_Rposition(void) {
    if (g_SPI_RX_flag == 1) {
        send_spi1((unsigned char) (g_motor_pos >> 24));
    } else if (g_SPI_RX_flag == 2) {
        send_spi1((unsigned char) (g_motor_pos >> 16));
    } else if (g_SPI_RX_flag == 3) {
        send_spi1((unsigned char) (g_motor_pos >> 8));
    } else if (g_SPI_RX_flag == 4) {
        send_spi1((unsigned char) g_motor_pos);
    } else if (g_SPI_RX_flag == 5) {
        send_spi1(ACK_SLAVE);
    }
}

void Gestion_RW_Rvitesse(void) {
    if (g_SPI_RX_flag == 1) {
        g_Vitesse_Cour_SPI = ReadSharedVarS16_SPI(&s16VitesseCour);
        send_spi1((unsigned char) (g_Vitesse_Cour_SPI >> 8));
    } else if (g_SPI_RX_flag == 2) {
        send_spi1((unsigned char) g_Vitesse_Cour_SPI);
    } else if (g_SPI_RX_flag == 3) {
        send_spi1(ACK_SLAVE);
    }
}

void Gestion_RW_Racceleration(void) {
    if (g_SPI_RX_flag == 1) {
        g_Acceleration_Cour_SPI = ReadSharedVarU16_SPI(&u16AccelerationCour);
        send_spi1((unsigned char) (g_Acceleration_Cour_SPI >> 8));
    } else if (g_SPI_RX_flag == 2) {
        send_spi1((unsigned char) g_Acceleration_Cour_SPI);
    } else if (g_SPI_RX_flag == 3) {
        send_spi1(ACK_SLAVE);
    }
}

void Gestion_RW_Rcourant(void) {
    if (g_SPI_RX_flag == 1) {
        g_mesure_courant_moy_SPI = ReadSharedVarU16_SPI(&u16CurrentValue);
        send_spi1((unsigned char) (g_mesure_courant_moy_SPI >> 8));
    } else if (g_SPI_RX_flag == 2) {
        send_spi1((unsigned char) g_mesure_courant_moy_SPI);
    } else if (g_SPI_RX_flag == 3) {
        send_spi1(ACK_SLAVE);
    }
}

void Gestion_RW_Rerreur(void) {
    if (g_SPI_RX_flag == 1) {
        send_spi1(ACK_SLAVE);
    }
}

void Gestion_RW_Rconfig(void) {
    if (g_SPI_RX_flag == 1) {
        send_spi1((unsigned char) (g_nom_Kp >> 8));
    } else if (g_SPI_RX_flag == 2) {
        send_spi1((unsigned char) (g_nom_Kp));
    } else if (g_SPI_RX_flag == 3) {
        send_spi1((unsigned char) (g_denom_Kp >> 8));
    } else if (g_SPI_RX_flag == 4) {
        send_spi1((unsigned char) (g_denom_Kp));
    } else if (g_SPI_RX_flag == 5) {
        send_spi1((unsigned char) (g_nom_Ki >> 8));
    } else if (g_SPI_RX_flag == 6) {
        send_spi1((unsigned char) (g_nom_Ki));
    } else if (g_SPI_RX_flag == 7) {
        send_spi1((unsigned char) (g_denom_Ki >> 8));
    } else if (g_SPI_RX_flag == 8) {
        send_spi1((unsigned char) (g_denom_Ki));
    } else if (g_SPI_RX_flag == 9) {
        send_spi1((unsigned char) (g_nom_Kd >> 8));
    } else if (g_SPI_RX_flag == 10) {
        send_spi1((unsigned char) (g_nom_Kd));
    } else if (g_SPI_RX_flag == 11) {
        send_spi1((unsigned char) (g_denom_Kd >> 8));
    } else if (g_SPI_RX_flag == 12) {
        send_spi1((unsigned char) (g_denom_Kd));
    } else if (g_SPI_RX_flag == 13) {
        send_spi1((unsigned char) (g_erreur_limite_I >> 24));
    } else if (g_SPI_RX_flag == 14) {
        send_spi1((unsigned char) (g_erreur_limite_I >> 16));
    } else if (g_SPI_RX_flag == 15) {
        send_spi1((unsigned char) (g_erreur_limite_I >> 8));
    } else if (g_SPI_RX_flag == 16) {
        send_spi1((unsigned char) (g_erreur_limite_I));
    } else if (g_SPI_RX_flag == 17) {
        send_spi1((unsigned char) (g_TimeOut_Recep));
    } else if (g_SPI_RX_flag == 18) {
        send_spi1((unsigned char) (g_timeMesureSpeed));
    } else if (g_SPI_RX_flag == 19) {
        send_spi1(ACK_SLAVE);
    }
}

void Gestion_RW_Rid(void) {
    if (g_SPI_RX_flag == 1) {
        send_spi1(TYPE_CARD);
    } else if (g_SPI_RX_flag == 2) {
        send_spi1((unsigned char) (g_ID_board >> 24));
    } else if (g_SPI_RX_flag == 3) {
        send_spi1((unsigned char) (g_ID_board >> 16));
    } else if (g_SPI_RX_flag == 4) {
        send_spi1((unsigned char) (g_ID_board >> 8));
    } else if (g_SPI_RX_flag == 5) {
        send_spi1((unsigned char) g_ID_board);
    } else if (g_SPI_RX_flag == 6) {
        send_spi1(ACK_SLAVE);
    }
}

void Gestion_RW_Wmot_conf(void) {
    if (g_SPI_RX_flag == 1) {
        g_motor_conf_vitesse_max = (signed long) ((((unsigned long) g_SPI_RX_clearReg) << 24)&0xFF000000);
        send_spi1(0);
    } else if (g_SPI_RX_flag == 2) {
        g_motor_conf_vitesse_max |= (signed long) ((((unsigned long) g_SPI_RX_clearReg) << 16)&0x00FF0000);
        send_spi1(0);
    } else if (g_SPI_RX_flag == 3) {
        g_motor_conf_vitesse_max |= (signed long) ((((unsigned long) g_SPI_RX_clearReg) << 8)&0x0000FF00);
        send_spi1(0);
    } else if (g_SPI_RX_flag == 4) {
        g_motor_conf_vitesse_max |= (signed long) (((unsigned long) g_SPI_RX_clearReg)&0x000000FF);
        send_spi1(0);
    } else if (g_SPI_RX_flag == 5) {
        g_motor_conf_courant_max = (unsigned int) ((((unsigned int) g_SPI_RX_clearReg) << 8)&0xFF00);
        send_spi1(0);
    } else if (g_SPI_RX_flag == 6) {
        g_motor_conf_courant_max |= (unsigned int) (((unsigned int) g_SPI_RX_clearReg)&0x00FF);
        send_spi1(0);
    }
    if (g_SPI_RX_flag == 7) {
        g_motor_conf_codeur_ppt = (signed long) ((((unsigned long) g_SPI_RX_clearReg) << 24)&0xFF000000);
        send_spi1(0);
    } else if (g_SPI_RX_flag == 8) {
        g_motor_conf_codeur_ppt |= (signed long) ((((unsigned long) g_SPI_RX_clearReg) << 16)&0x00FF0000);
        send_spi1(0);
    } else if (g_SPI_RX_flag == 9) {
        g_motor_conf_codeur_ppt |= (signed long) ((((unsigned long) g_SPI_RX_clearReg) << 8)&0x0000FF00);
        send_spi1(0);
    } else if (g_SPI_RX_flag == 10) {
        g_motor_conf_codeur_ppt |= (signed long) (((unsigned long) g_SPI_RX_clearReg)&0x000000FF);
        send_spi1(ACK_SLAVE);
    }
}

void Gestion_RW_Rmot_conf(void) {
    if (g_SPI_RX_flag == 1) {
        send_spi1((unsigned char) (g_motor_conf_vitesse_max >> 24));
    } else if (g_SPI_RX_flag == 2) {
        send_spi1((unsigned char) (g_motor_conf_vitesse_max >> 16));
    } else if (g_SPI_RX_flag == 3) {
        send_spi1((unsigned char) (g_motor_conf_vitesse_max >> 8));
    } else if (g_SPI_RX_flag == 4) {
        send_spi1((unsigned char) g_motor_conf_vitesse_max);
    } else if (g_SPI_RX_flag == 5) {
        send_spi1((unsigned char) (g_motor_conf_courant_max >> 8));
    } else if (g_SPI_RX_flag == 6) {
        send_spi1((unsigned char) (g_motor_conf_courant_max));
    } else if (g_SPI_RX_flag == 7) {
        send_spi1((unsigned char) (g_motor_conf_codeur_ppt >> 24));
    } else if (g_SPI_RX_flag == 8) {
        send_spi1((unsigned char) (g_motor_conf_codeur_ppt >> 16));
    } else if (g_SPI_RX_flag == 9) {
        send_spi1((unsigned char) (g_motor_conf_codeur_ppt >> 8));
    } else if (g_SPI_RX_flag == 10) {
        send_spi1((unsigned char) (g_motor_conf_codeur_ppt));
        if (g_motor_conf_codeur_ppt == 0)
            g_interface_mesure_vitesse = CODEUR_OFF;
        else 
            g_interface_mesure_vitesse = CODEUR_ON;
    } else if (g_SPI_RX_flag == 11) {
        send_spi1(ACK_SLAVE);
    }
}

void Gestion_LED(unsigned int freq) {
    if (freq != 0) {
        if (g_timer_led >= freq) {
            g_timer_led = 0;
            LED = ~LED;
        }
    }
}

void Gestion_Courant(void) {
    if ((g_stateAcq == 0) && (g_timer_AcqCourant >= 20)) //cadencé à 100us
    {
        g_timer_AcqCourant = 0;
        SetChanADC10(ADC_CSOUT); // Select the requested channel
        ADCON1bits.SAMP = 1; // start sampling
        g_stateAcq = 1;
    } else if ((g_stateAcq == 1) && (g_timer_AcqCourant >= 1)) {
        ConvertADC10(); // start Converting
        g_stateAcq = 2;
    } else if ((g_stateAcq == 2) && (ADCON1bits.DONE == 1)) {
        g_mesure_courant += ReadADC10(0);
        g_nb_mes_courant++;

        g_stateAcq = 0;
    } else if ((g_stateAcq == 0) && (g_nb_mes_courant >= 5)) {
        // Somme des mesures de courant effectué diviser par le nb de mesures
        g_mesure_courant /= (unsigned long) g_nb_mes_courant;
        // Reset des variables necessaires à la nouvelles serie d'acquisition
        WriteSharedVarU16_APP(&u16CurrentValue, (unsigned int)g_mesure_courant);
        g_mesure_courant = 0;
        g_nb_mes_courant = 0;
    }
}
// Int2 interrupt service routine

void __attribute__((interrupt, no_auto_psv)) _INT2Interrupt(void) {
    _INT2IF = 0;
    g_pulse_drive++;
}
// Timer 1 interrupt service routine

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void) {
    WriteTimer1(0);
    _T1IF = 0;
    g_timerSpeed++;
    g_timerControl++;

    if (g_timerSpeed >= g_timeMesureSpeed) {
        g_timerSpeed = 0;

        if (g_interface_mesure_vitesse == CODEUR_ON) {
            g_motor_pos = (((signed long) g_motor_pos_H) << 15)+((signed long) ReadQEI()); //recup position
            g_motor_vitesse = (((signed long) g_motor_pos)-((signed long) g_motor_pos_mem)); //Determine le nombre de pas depuis la derniere acquisition
            g_motor_vitesse *= g_multi_timeMesureSpeed_s; //ramène en pas par seconde

            g_Vitesse_Cour = (signed int) (g_motor_vitesse / g_motor_conf_codeur_ppt); //divise par 32 pour que ca entre dans int et le rende transportable  

            g_motor_pos_mem = g_motor_pos; //Sauvegarde de la valeur à soustraire pour la prochain calcul
        } else {
            g_motor_vitesse = ((signed long) g_pulse_drive)*((signed long) g_multi_timeMesureSpeed_s);
            g_pulse_drive = 0;
            if (DRIVER_DIRO == 1)g_motor_vitesse *= -1;

            g_Vitesse_Cour = (signed int) (g_motor_vitesse / 24); //RPS    driver = 24 pulses / tr
        }
        WriteSharedVarS16_APP(&s16VitesseCour, g_Vitesse_Cour);
    }

    if (g_timerControl >= g_timeControlLoop) {
        g_timerControl = 0;

        if (g_flag_asser == LOOP) {
            g_Erreur_P = (g_vitesse_desiree) - g_motor_vitesse; // Erreur Proportionnelle
            g_Erreur_I += g_Erreur_P; // Cumul Intégral

            g_dutycycle = g_Erreur_P * g_nom_Kp / g_denom_Kp; // Calcul Asservissement Kp
            g_dutycycle += g_Erreur_I * g_nom_Ki / g_denom_Ki; // Calcul Asservissement Ki
        } else {
            g_dutycycle = g_vitesse_desiree*PWM_VAL_CENTRE;
        }
        g_dutycycle /= 1000;
        g_dutycycle += PWM_VAL_CENTRE;
        SetDCOC1PWM((unsigned int) (g_dutycycle));
    }
}

// Timer 2 interrupt service routine

void __attribute__((interrupt, no_auto_psv)) _T2Interrupt(void) {
    WriteTimer2(0);
    _T2IF = 0;
    //Timer reglé sur 100us
    g_timer_AcqCourant++;

}

// Timer 4 interrupt service routine

void __attribute__((interrupt, no_auto_psv)) _T4Interrupt(void) {
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

void __attribute__((interrupt, no_auto_psv)) _SPI1Interrupt(void) {
    _SPI1IF = 0;
    if (SPI1STATbits.SPIROV) {
        SPI1STATbits.SPIROV = 0;
        g_Erreur_Cour++;
    }

    while (DataRdySPI1()) {
        g_SPI_RX_clearReg = SPI1BUF;
        if ((g_SPI_RX_flag == 0)) {
            if ((g_SPI_RX_clearReg > 0)&&(g_SPI_RX_clearReg <= NB_FONCTION)) {
                send_spi1(g_Erreur_Cour);
                g_SPI_RX_trame.commande = g_SPI_RX_clearReg;
                g_SPI_RX_flag++; // si oui incremente l'etape de gestion rx
            }
        }// si non verifie les autres condition
        else if (g_SPI_RX_flag >= 1 && g_SPI_RX_flag <= (g_SPI_RX_NbData[g_SPI_RX_trame.commande])) {
            switch (g_SPI_RX_trame.commande) {
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
        } else if ((g_SPI_RX_flag == (g_SPI_RX_NbData[g_SPI_RX_trame.commande] + 1)) && g_SPI_RX_clearReg == ACK_MASTER) {
            send_spi1(0); // si le dernier octet est le ender la trame est bonne
            //            g_SPI_RX_trame_valide=1;
            g_SPI_RX_flag = 0;
        } else {
            send_spi1(0);
            g_SPI_RX_flag = 0;
        }
    }
}

void __attribute__((interrupt, no_auto_psv)) _QEIInterrupt(void) {
    _QEIIF = 0; /* Clear QEI interrupt flag */
    if (QEICONbits.UPDN)g_motor_pos_H++;
    else g_motor_pos_H--;
}

