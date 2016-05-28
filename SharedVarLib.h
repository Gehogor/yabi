/*
 *__     __        ____  _____
 *\ \   / / /\    |  _ \|_   _|
 * \ \_/ / /  \   | |_) | | |
 *  \   / / /\ \  |  _ <  | |
 *   | | / ____ \ | |_) |_| |_
 *   |_|/_/    \_\|____/|_____|
 *
 * File:   SharedVarLib.h
 * Author: Yann
 *
 * Created on 25 avril 2016, 19:01
 */

#ifndef SHAREDVARLIB_H
#define	SHAREDVARLIB_H

#define LOCK    1
#define UNLOCK  0

typedef struct {
    unsigned char mutex;
    unsigned char u8_data[2];
    unsigned char u8_data_APP;
    unsigned char u8_data_SPI;
} u8_shared_var;

typedef struct {
    unsigned char mutex;
    unsigned int u16_data[2];
    unsigned int u16_data_APP;
    unsigned int u16_data_SPI;
} u16_shared_var;

typedef struct {
    unsigned char mutex;
    unsigned long int u32_data[2];
    unsigned long int u32_data_APP;
    unsigned long int u32_data_SPI;
} u32_shared_var;

typedef struct {
    unsigned char mutex;
    signed char s8_data[2];
    signed char s8_data_APP;
    signed char s8_data_SPI;
} s8_shared_var;

typedef struct {
    unsigned char mutex;
    signed int s16_data[2];
    signed int s16_data_APP;
    signed int s16_data_SPI;
} s16_shared_var;

typedef struct {
    unsigned char mutex;
    signed long int s32_data[2];
    signed long int s32_data_APP;
    signed long int s32_data_SPI;
} s32_shared_var;

void InitSharedVarU8(u8_shared_var *pt_u8ShVar, unsigned char u8InitVar);
void WriteSharedVarU8_APP(u8_shared_var *pt_u8ShVar);
void ReadSharedVarU8_APP(u8_shared_var *pt_u8ShVar);
void WriteSharedVarU8_SPI(u8_shared_var *pt_u8ShVar);
void ReadSharedVarU8_SPI(u8_shared_var *pt_u8ShVar);

void InitSharedVarU16(u16_shared_var *pt_u16ShVar, unsigned int u16InitVar);
void WriteSharedVarU16_APP(u16_shared_var *pt_u16ShVar);
void ReadSharedVarU16_APP(u16_shared_var *pt_u16ShVar);
void WriteSharedVarU16_SPI(u16_shared_var *pt_u16ShVar);
void ReadSharedVarU16_SPI(u16_shared_var *pt_u16ShVar);

void InitSharedVarU32(u32_shared_var *pt_u32ShVar, unsigned long int u32InitVar);
void WriteSharedVarU32_APP(u32_shared_var *pt_u32ShVar);
void ReadSharedVarU32_APP(u32_shared_var *pt_u32ShVar);
void WriteSharedVarU32_SPI(u32_shared_var *pt_u32ShVar);
void ReadSharedVarU32_SPI(u32_shared_var *pt_u32ShVar);

void InitSharedVarS8(s8_shared_var *pt_s8ShVar, signed char s8InitVar);
void WriteSharedVarS8_APP(s8_shared_var *pt_s8ShVar);
void ReadSharedVarS8_APP(s8_shared_var *pt_s8ShVar);
void WriteSharedVarS8_SPI(s8_shared_var *pt_s8ShVar);
void ReadSharedVarS8_SPI(s8_shared_var *pt_s8ShVar);

void InitSharedVarS16(s16_shared_var *pt_s16ShVar, signed int s16InitVar);
void WriteSharedVarS16_APP(s16_shared_var *pt_s16ShVar);
void ReadSharedVarS16_APP(s16_shared_var *pt_s16ShVar);
void WriteSharedVarS16_SPI(s16_shared_var *pt_s16ShVar);
void ReadSharedVarS16_SPI(s16_shared_var *pt_s16ShVar);

void InitSharedVarS32(s32_shared_var *pt_s32ShVar, signed long int s32InitVar);
void WriteSharedVarS32_APP(s32_shared_var *pt_s32ShVar);
void ReadSharedVarS32_APP(s32_shared_var *pt_s32ShVar);
void WriteSharedVarS32_SPI(s32_shared_var *pt_s32ShVar);
void ReadSharedVarS32_SPI(s32_shared_var *pt_s32ShVar);

#endif	/* SHAREDVARLIB_H */
