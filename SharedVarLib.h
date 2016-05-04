/* 
 * File:   SharedVarLib.h
 * Author: Yann
 *
 * Created on 25 avril 2016, 19:01
 */

#ifndef SHAREDVARLIB_H
#define	SHAREDVARLIB_H


#define LOCK    1
#define UNLOCK  0

typedef struct 
{
    unsigned char mutex;
    unsigned char u8_data[2];
} u8_shared_var;

typedef struct
{
    unsigned char mutex;
    unsigned int u16_data[2];
} u16_shared_var;

typedef struct 
{
    unsigned char mutex;
    unsigned long u32_data[2];
} u32_shared_var;

typedef struct 
{
    unsigned char mutex;
    char u8_data[2];
} s8_shared_var;

typedef struct 
{
    unsigned char mutex;
    int u16_data[2];
} s16_shared_var;

typedef struct 
{
    unsigned char mutex;
    long u32_data[2];
} s32_shared_var;

void WriteSharedVarU8_APP(u8_shared_var *pt_u8ShVar, unsigned char val);
unsigned char ReadSharedVarU8_APP(u8_shared_var *pt_u8ShVar);
void WriteSharedVarU8_SPI(u8_shared_var *pt_u8ShVar, unsigned char val);
unsigned char ReadSharedVarU8_SPI(u8_shared_var *pt_u8ShVar);

void WriteSharedVarU16_APP(u16_shared_var *pt_u16ShVar, unsigned int val);
unsigned int ReadSharedVarU16_APP(u16_shared_var *pt_u16ShVar);
void WriteSharedVarU16_SPI(u16_shared_var *pt_u16ShVar, unsigned int val);
unsigned int ReadSharedVarU16_SPI(u16_shared_var *pt_u16ShVar);

void WriteSharedVarU32_APP(u32_shared_var *pt_u32ShVar, unsigned long val);
unsigned long ReadSharedVarU32_APP(u32_shared_var *pt_u32ShVar);
void WriteSharedVarU32_SPI(u32_shared_var *pt_u32ShVar, unsigned long val);
unsigned long ReadSharedVarU32_SPI(u32_shared_var *pt_u32ShVar);

void WriteSharedVarS8_APP(s8_shared_var *pt_s8ShVar, unsigned char val);
unsigned char ReadSharedVarS8_APP(s8_shared_var *pt_s8ShVar);
void WriteSharedVarS8_SPI(s8_shared_var *pt_s8ShVar, unsigned char val);
unsigned char ReadSharedVarS8_SPI(s8_shared_var *pt_s8ShVar);

void WriteSharedVarS16_APP(s16_shared_var *pt_s16ShVar, int val);
int ReadSharedVarS16_APP(s16_shared_var *pt_s16ShVar);
void WriteSharedVarS16_SPI(s16_shared_var *pt_s16ShVar, int val);
int ReadSharedVarS16_SPI(s16_shared_var *pt_s16ShVar);

void WriteSharedVarS32_APP(s32_shared_var *pt_s32ShVar, long int val);
long int ReadSharedVarS32_APP(s32_shared_var *pt_s32ShVar);
void WriteSharedVarS32_SPI(s32_shared_var *pt_s32ShVar, long int val);
long int ReadSharedVarS32_SPI(s32_shared_var *pt_s32ShVar);

#endif	/* SHAREDVARLIB_H */

