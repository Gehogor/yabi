/* 
 * File:   Driver_Dspic.h
 * Author: Yann
 *
 * Created on 1 mai 2015, 10:05
 */
#include <adc10.h>

#ifndef MAIN_H
#define	MAIN_H

/**** Paramètres Carte d'axe****/
#define TYPE_CARD 0x01  //Carte d'axe Moteur Brushless
//#define TYPE_CARD 0x02  //Carte d'axe Moteur CC
//#define TYPE_CARD 0x03  //Carte d'axe Moteur Pas à Pas
//#define TYPE_CARD 0x04  //Carte d'axe Servo moteur modélisme

#define CODEUR_OFF  1
#define CODEUR_ON   0

/**** Paramètres généraux ****/
#define FREQUENCY   117968000		//Fréquence de fonctionement xPLL16  =>  XTAL=7.373Mhz x16
#define TOSC_GENE   (1/FREQUENCY)
#define TCY         0.000000033907      //(TOSC_GENE*4)
#define FCY         29492000            //(1/TCY) ~30MIPS
#define BAUD 115200				//Bibliothèque et paramètres pour la liaison RS232
#define BRG ((FCY / (BAUD * 16))-1)

#define TMRX_PRES_VAL_1     1
#define TMRX_PRES_VAL_8     8
#define TMRX_PRES_VAL_64    64
#define TMRX_PRES_VAL_256   256

#define TMRX_PRES_PARAM_1     0
#define TMRX_PRES_PARAM_8     1
#define TMRX_PRES_PARAM_64    2
#define TMRX_PRES_PARAM_256   3

#define PR_T1   29500       //1kHz
#define PR_T2   2938        //10kHz
#define PR_T3   1474        //20kHz
#define PR_T4   11480       //10Hz


/**** Paramètres Outout Compare ****/
//PWM Period = [(PRy) + 1] x TCY x (TMRy Prescale Value)
//PWM Frequency = 1/[PWM Period]
#define PWM_FREQ    20000
#define PWM_PER     (1/PWM_FREQ)

#define PWM_PRES_VAL   TMRX_PRES_VAL_1
#define PWM_PRES_PARAM TMRX_PRES_PARAM_1

#define PWM_VAL_MAX     1480
#define PWM_VAL_CENTRE  PWM_VAL_MAX/2

#define PRX_REG     (PWM_PER/(TCY*PWM_PRES_VAL))

/**** Paramètres I/O ****/
#define DRIVER_MODE     _LATE1
#define DRIVER_DIR      _LATE2
#define DRIVER_COAST	_LATE3
#define DRIVER_TACHO	_RD1
#define DRIVER_DIRO     _RE4
#define DRIVER_FF1      _RC13
#define DRIVER_FF2      _RC14
#define DRIVER_ALIMP	_RE4

#define CODEUR_PA       _RB4
#define CODEUR_PB       _RB5
#define CODEUR_PZ       _RB3

#define SDI_SCK         _RE8
#define SDI_SDI         _RF2
#define SDI_SDO         _RF3
#define SDI_SS          _RB2

#define ADC_CSOUT       (ADC_CH0_POS_SAMPLEA_AN0 & ADC_CH0_NEG_SAMPLEA_NVREF)
#define ADC_REF         (ADC_CH0_POS_SAMPLEA_AN1 & ADC_CH0_NEG_SAMPLEA_NVREF)

#define LED		_LATE5

#define LED_FREQ_0HZ    0
#define LED_FREQ_10HZ   1
#define LED_FREQ_5HZ    2
#define LED_FREQ_2HZ    5
#define LED_FREQ_1HZ    10

#define LED_ON  0
#define LED_OFF 1

/**** Paramètres USART ****/

#define ACK_MASTER 0x5E
#define ACK_SLAVE  0xE5

#define COM_W_CONSIGNE      0x01
#define COM_W_MODE          0x02
#define COM_W_POSITION      0x03
#define COM_W_CONFIG        0x04
#define COM_R_POSITION      0x05
#define COM_R_VITESSE       0x06
#define COM_R_ACCELERATION  0x07
#define COM_R_COURANT       0x08
#define COM_R_ERREUR        0x09
#define COM_R_CONFIG        0x0A
#define COM_R_ID_BOARD      0x0B
#define COM_W_MOT_CONF      0x0C
#define COM_R_MOT_CONF      0x0D

#define DATA_W_CONSIGNE      11
#define DATA_W_MODE          1
#define DATA_W_POSITION      4
#define DATA_W_CONFIG        18
#define DATA_R_POSITION      5
#define DATA_R_VITESSE       3
#define DATA_R_ACCELERATION  3
#define DATA_R_COURANT       3
#define DATA_R_ERREUR        1
#define DATA_R_CONFIG        19
#define DATA_R_ID_BOARD      6
#define DATA_W_MOT_CONF      10
#define DATA_R_MOT_CONF      11

#define NB_FONCTION          13

#define COM_ERREUR_FUNC_UNCKNOW     0xF6

/**** Machine à état de la carte Yabi ******/

#define MODE_STOP      0x00
#define MODE_OPEN      0x01
#define MODE_ASSER     0x02

#define NB_MODE        3

#define STOP    0x00
#define OPEN    0x01
#define LOOP    0x02

// Déclaration Structures 
typedef struct     //Struct trame de transmission
{
    unsigned char commande;
    unsigned char data[25];
} trame_SPI;

// Function prototype for Timer 1 interrupt service routine
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _T2Interrupt(void);
//void __attribute__((interrupt, no_auto_psv)) _T3Interrupt(void)
//void __attribute__((interrupt, no_auto_psv)) _U1TXInterrupt(void);
void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt(void);
//void __attribute__((interrupt, no_auto_psv)) _OC1Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _SPI1Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _QEIInterrupt(void);
void __attribute__((interrupt, no_auto_psv)) _INT2Interrupt(void);


void Configure_pins(void);
void InitFonctionRecep(void);
void InitDriver(void);
unsigned int read_analog_channel(int channel);
void Envoi_Char( unsigned char Octet);
void WriteSPI1_c(unsigned char c);
void Gestion_RW_Wconsigne(void);
void Gestion_RW_Wmode(void);
void Gestion_RW_Wposition(void);
void Gestion_RW_Wconfig(void);
void Gestion_RW_Rposition(void);
void Gestion_RW_Rvitesse(void);
void Gestion_RW_Racceleration(void);
void Gestion_RW_Rcourant(void);
void Gestion_RW_Rerreur(void);
void Gestion_RW_Rconfig(void);
void Gestion_RW_Rid(void);
void Gestion_RW_Wmot_conf(void);
void Gestion_RW_Rmot_conf(void);
void Gestion_LED(unsigned int freq);
void Gestion_Courant(void);
unsigned int received_spi1(void);
void send_spi1(unsigned int mtdata);
void InitSharedVar(void);

#endif	/* DRIVER_DSPIC_H */

