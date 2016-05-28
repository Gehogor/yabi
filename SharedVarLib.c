/*
 *__     __        ____  _____
 *\ \   / / /\    |  _ \|_   _|
 * \ \_/ / /  \   | |_) | | |
 *  \   / / /\ \  |  _ <  | |
 *   | | / ____ \ | |_) |_| |_
 *   |_|/_/    \_\|____/|_____|
 *
 * File:   SharedVarLib.c
 * Author: Yann
 *
 * Created on 25 avril 2016, 19:01
 */

#include "SharedVarLib.h"


// Management of unsigned char -----------------------------------------------//
void InitSharedVarU8(u8_shared_var *pt_u8ShVar, unsigned char u8InitVar)
{
    pt_u8ShVar->mutex = UNLOCK;
    pt_u8ShVar->u8_data[0]=u8InitVar;
    pt_u8ShVar->u8_data[1]=u8InitVar;
    pt_u8ShVar->u8_data_APP=u8InitVar;
    pt_u8ShVar->u8_data_SPI=u8InitVar;
}

void WriteSharedVarU8_APP(u8_shared_var *pt_u8ShVar)
{
    while( pt_u8ShVar->mutex == LOCK );

    pt_u8ShVar->mutex = LOCK;
    pt_u8ShVar->u8_data[0] = pt_u8ShVar->u8_data_APP;
    pt_u8ShVar->mutex = UNLOCK;
    pt_u8ShVar->u8_data[1] = pt_u8ShVar->u8_data_APP;
}

void ReadSharedVarU8_APP(u8_shared_var *pt_u8ShVar)
{
    unsigned char val;
    while( pt_u8ShVar->mutex == LOCK );

    pt_u8ShVar->mutex = LOCK;
    val = pt_u8ShVar->u8_data[0];
    pt_u8ShVar->mutex = UNLOCK;
    pt_u8ShVar->u8_data_APP = val;
}

void WriteSharedVarU8_SPI(u8_shared_var *pt_u8ShVar)
{
    pt_u8ShVar->u8_data[pt_u8ShVar->mutex] = pt_u8ShVar->u8_data_SPI;
}

void ReadSharedVarU8_SPI(u8_shared_var *pt_u8ShVar)
{
    pt_u8ShVar->u8_data_SPI = pt_u8ShVar->u8_data[pt_u8ShVar->mutex];
}


// Management of unsigned int ------------------------------------------------//
void InitSharedVarU16(u16_shared_var *pt_u16ShVar, unsigned int u16InitVar)
{
    pt_u16ShVar->mutex = UNLOCK;
    pt_u16ShVar->u16_data[0]=u16InitVar;
    pt_u16ShVar->u16_data[1]=u16InitVar;
    pt_u16ShVar->u16_data_APP=u16InitVar;
    pt_u16ShVar->u16_data_SPI=u16InitVar;
}

void WriteSharedVarU16_APP(u16_shared_var *pt_u16ShVar)
{
    while( pt_u16ShVar->mutex == LOCK );

    pt_u16ShVar->mutex = LOCK;
    pt_u16ShVar->u16_data[0] = pt_u16ShVar->u16_data_APP;
    pt_u16ShVar->mutex = UNLOCK;
    pt_u16ShVar->u16_data[1] = pt_u16ShVar->u16_data_APP;
}

void ReadSharedVarU16_APP(u16_shared_var *pt_u16ShVar)
{
    unsigned int val;
    while( pt_u16ShVar->mutex == LOCK );

    pt_u16ShVar->mutex = LOCK;
    val = pt_u16ShVar->u16_data[0];
    pt_u16ShVar->mutex = UNLOCK;
    pt_u16ShVar->u16_data_APP = val;
}

void WriteSharedVarU16_SPI(u16_shared_var *pt_u16ShVar)
{
    pt_u16ShVar->u16_data[pt_u16ShVar->mutex] = pt_u16ShVar->u16_data_SPI;
}

void ReadSharedVarU16_SPI(u16_shared_var *pt_u16ShVar)
{
    pt_u16ShVar->u16_data_SPI = pt_u16ShVar->u16_data[pt_u16ShVar->mutex];
}


// Management of unsigned long -----------------------------------------------//
void InitSharedVarU32(u32_shared_var *pt_u32ShVar, unsigned long int u32InitVar)
{
    pt_u32ShVar->mutex = UNLOCK;
    pt_u32ShVar->u32_data[0]=u32InitVar;
    pt_u32ShVar->u32_data[1]=u32InitVar;
    pt_u32ShVar->u32_data_APP=u32InitVar;
    pt_u32ShVar->u32_data_SPI=u32InitVar;
}

void WriteSharedVarU32_APP(u32_shared_var *pt_u32ShVar)
{
    while( pt_u32ShVar->mutex == LOCK );

    pt_u32ShVar->mutex = LOCK;
    pt_u32ShVar->u32_data[0] = pt_u32ShVar->u32_data_APP;
    pt_u32ShVar->mutex = UNLOCK;
    pt_u32ShVar->u32_data[1] = pt_u32ShVar->u32_data_APP;
}

void ReadSharedVarU32_APP(u32_shared_var *pt_u32ShVar)
{
    unsigned long int val;
    while( pt_u32ShVar->mutex == LOCK );

    pt_u32ShVar->mutex = LOCK;
    val = pt_u32ShVar->u32_data[0];
    pt_u32ShVar->mutex = UNLOCK;
    pt_u32ShVar->u32_data_APP = val;
}

void WriteSharedVarU32_SPI(u32_shared_var *pt_u32ShVar)
{
    pt_u32ShVar->u32_data[pt_u32ShVar->mutex] = pt_u32ShVar->u32_data_SPI;
}

void ReadSharedVarU32_SPI(u32_shared_var *pt_u32ShVar)
{
    pt_u32ShVar->u32_data_SPI = pt_u32ShVar->u32_data[pt_u32ShVar->mutex];
}


// Management of signed char -------------------------------------------------//
void InitSharedVarS8(s8_shared_var *pt_s8ShVar, signed char s8InitVar)
{
    pt_s8ShVar->mutex = UNLOCK;
    pt_s8ShVar->s8_data[0]=s8InitVar;
    pt_s8ShVar->s8_data[1]=s8InitVar;
    pt_s8ShVar->s8_data_APP=s8InitVar;
    pt_s8ShVar->s8_data_SPI=s8InitVar;
}

void WriteSharedVarS8_APP(s8_shared_var *pt_s8ShVar)
{
    while( pt_s8ShVar->mutex == LOCK );

    pt_s8ShVar->mutex = LOCK;
    pt_s8ShVar->s8_data[0] = pt_s8ShVar->s8_data_APP;
    pt_s8ShVar->mutex = UNLOCK;
    pt_s8ShVar->s8_data[1] = pt_s8ShVar->s8_data_APP;
}

void ReadSharedVarS8_APP(s8_shared_var *pt_s8ShVar)
{
    signed char val;
    while( pt_s8ShVar->mutex == LOCK );

    pt_s8ShVar->mutex = LOCK;
    val = pt_s8ShVar->s8_data[0];
    pt_s8ShVar->mutex = UNLOCK;
    pt_s8ShVar->s8_data_APP = val;
}

void WriteSharedVarS8_SPI(s8_shared_var *pt_s8ShVar)
{
    pt_s8ShVar->s8_data[pt_s8ShVar->mutex] = pt_s8ShVar->s8_data_SPI;
}

void ReadSharedVarS8_SPI(s8_shared_var *pt_s8ShVar)
{
    pt_s8ShVar->s8_data_SPI = pt_s8ShVar->s8_data[pt_s8ShVar->mutex];
}


// Management of signed int --------------------------------------------------//
void InitSharedVarS16(s16_shared_var *pt_s16ShVar, signed int s16InitVar)
{
    pt_s16ShVar->mutex = UNLOCK;
    pt_s16ShVar->s16_data[0]=s16InitVar;
    pt_s16ShVar->s16_data[1]=s16InitVar;
    pt_s16ShVar->s16_data_APP=s16InitVar;
    pt_s16ShVar->s16_data_SPI=s16InitVar;
}

void WriteSharedVarS16_APP(s16_shared_var *pt_s16ShVar)
{
    while( pt_s16ShVar->mutex == LOCK );

    pt_s16ShVar->mutex = LOCK;
    pt_s16ShVar->s16_data[0] = pt_s16ShVar->s16_data_APP;
    pt_s16ShVar->mutex = UNLOCK;
    pt_s16ShVar->s16_data[1] = pt_s16ShVar->s16_data_APP;
}

void ReadSharedVarS16_APP(s16_shared_var *pt_s16ShVar)
{
    signed int val;
    while( pt_s16ShVar->mutex == LOCK );

    pt_s16ShVar->mutex = LOCK;
    val = pt_s16ShVar->s16_data[0];
    pt_s16ShVar->mutex = UNLOCK;
    pt_s16ShVar->s16_data_APP = val;
}

void WriteSharedVarS16_SPI(s16_shared_var *pt_s16ShVar)
{
    pt_s16ShVar->s16_data[pt_s16ShVar->mutex] = pt_s16ShVar->s16_data_SPI;
}

void ReadSharedVarS16_SPI(s16_shared_var *pt_s16ShVar)
{
    pt_s16ShVar->s16_data_SPI = pt_s16ShVar->s16_data[pt_s16ShVar->mutex];
}


// Management of signed long -------------------------------------------------//
void InitSharedVarS32(s32_shared_var *pt_s32ShVar, signed long int s32InitVar)
{
    pt_s32ShVar->mutex = UNLOCK;
    pt_s32ShVar->s32_data[0]=s32InitVar;
    pt_s32ShVar->s32_data[1]=s32InitVar;
    pt_s32ShVar->s32_data_APP=s32InitVar;
    pt_s32ShVar->s32_data_SPI=s32InitVar;
}

void WriteSharedVarS32_APP(s32_shared_var *pt_s32ShVar)
{
    while( pt_s32ShVar->mutex == LOCK );

    pt_s32ShVar->mutex = LOCK;
    pt_s32ShVar->s32_data[0] = pt_s32ShVar->s32_data_APP;
    pt_s32ShVar->mutex = UNLOCK;
    pt_s32ShVar->s32_data[1] = pt_s32ShVar->s32_data_APP;
}

void ReadSharedVarS32_APP(s32_shared_var *pt_s32ShVar)
{
    signed long int val;
    while( pt_s32ShVar->mutex == LOCK );

    pt_s32ShVar->mutex = LOCK;
    val = pt_s32ShVar->s32_data[0];
    pt_s32ShVar->mutex = UNLOCK;
    pt_s32ShVar->s32_data_APP = val;
}

void WriteSharedVarS32_SPI(s32_shared_var *pt_s32ShVar)
{
    pt_s32ShVar->s32_data[pt_s32ShVar->mutex] = pt_s32ShVar->s32_data_SPI;
}

void ReadSharedVarS32_SPI(s32_shared_var *pt_s32ShVar)
{
    pt_s32ShVar->s32_data_SPI = pt_s32ShVar->s32_data[pt_s32ShVar->mutex];
}
