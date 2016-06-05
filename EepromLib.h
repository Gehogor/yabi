/* 
 * File:   EepromLib.h
 * Author: Yann
 *
 * Created on 29 mai 2016, 00:53
 */

#ifndef EEPROMLIB_H
#define	EEPROMLIB_H


// Data Eeprom parameters ----------------------------------------------------//
#define ADD_EE_KP_NUM           0x0000		//Add Eeprom u16 (0x0000 a 0x0001)
#define ADD_EE_KP_DENUM         0x0002		//Add Eeprom u16 (0x0002 a 0x0003)
#define ADD_EE_KI_NUM           0x0004		//Add Eeprom u16 (0x0004 a 0x0005)
#define ADD_EE_KI_DENUM         0x0006		//Add Eeprom u16 (0x0006 a 0x0007)
#define ADD_EE_KD_NUM           0x0008		//Add Eeprom u16 (0x0008 a 0x0009)
#define ADD_EE_KD_DENUM     	0x000A		//Add Eeprom u16 (0x000A a 0x000B)
#define ADD_EE_I_ERROR_MAX      0x000C		//Add Eeprom s32 (0x000C a 0x000F)

#define ADD_EE_CONF_CODEUR  	0x0010		//Add Eeprom u16 (0x0010 a 0x0011)
#define ADD_EE_CONF_MAXCUR      0x0012		//Add Eeprom u16 (0x0012 a 0x0013)
#define ADD_EE_CONF_MAXSPEED	0x0014		//Add Eeprom s16 (0x0014 a 0x0015)

#define ADD_EE_TIME_MES_SPEED   0x0016	//Add Eeprom u8
#define ADD_EE_TIME_CTRL_LOOP   0x0017  //Add Eeprom u8

#define ADD_EE_TIMEOUT_RECEP    0x0018  //Add Eeprom u8
#define ADD_EE_LAST_ERROR       0x0019  //Add Eeprom u8

#define ADD_EE_ID_board         0x00A0  //Add Eeprom u32 (0x00A0 a 0x00A3)


#endif	/* EEPROMLIB_H */

