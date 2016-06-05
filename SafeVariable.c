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

#include "SafeVariable.h"


// Management of unsigned char -----------------------------------------------//
void init_u8(safe_u8* var, unsigned char value)
{
    var->index = 0;
    var->data[0] = value;
    var->data[1] = value;
}

void write_u8(safe_u8* var, unsigned char value)
{
    var->index = inverseIndex(var->index);
    var->data[ var->index ] = value;
}

unsigned char read_u8(safe_u8* var)
{
    if( var->index == 0 )
        return var->data[1];
    return var->data[0];
}


// Management of unsigned int ------------------------------------------------//
void init_u16(safe_u16* var, unsigned int value)
{
    var->index = 0;
    var->data[0] = value;
    var->data[1] = value;
}

void write_u16(safe_u16* var, unsigned int value)
{
    var->index = inverseIndex(var->index);
    var->data[ var->index ] = value;
}

unsigned int read_u16(safe_u16* var)
{
    if( var->index == 0 )
        return var->data[1];
    return var->data[0];
}


// Management of unsigned long int -------------------------------------------//
void init_u32(safe_u32* var, unsigned long int value)
{
    var->index = 0;
    var->data[0] = value;
    var->data[1] = value;
}

void write_u32(safe_u32* var, unsigned long int value)
{
    var->index = inverseIndex(var->index);
    var->data[ var->index ] = value;
}

unsigned long int read_u32(safe_u32* var)
{
    if( var->index == 0 )
        return var->data[1];
    return var->data[0];
}


// Management of signed char -------------------------------------------------//
void init_s8(safe_s8* var, signed char value)
{
    var->index = 0;
    var->data[0] = value;
    var->data[1] = value;
}

void write_s8(safe_s8* var, signed char value)
{
    var->index = inverseIndex(var->index);
    var->data[ var->index ] = value;
}

signed char read_s8(safe_s8* var)
{
    if( var->index == 0 )
        return var->data[1];
    return var->data[0];
}


// Management of signed int --------------------------------------------------//
void init_s16(safe_s16* var, signed int value)
{
    var->index = 0;
    var->data[0] = value;
    var->data[1] = value;
}

void write_s16(safe_s16* var, signed int value)
{
    var->index = inverseIndex(var->index);
    var->data[ var->index ] = value;
}

signed int read_s16(safe_s16* var)
{
    if( var->index == 0 )
        return var->data[1];
    return var->data[0];
}


// Management of signed long -------------------------------------------------//
void init_s32(safe_s32* var, signed long int value)
{
    var->index = 0;
    var->data[0] = value;
    var->data[1] = value;
}

void write_s32(safe_s32* var, signed long int value)
{
    var->index = inverseIndex(var->index);
    var->data[ var->index ] = value;
}

signed long int read_s32(safe_s32* var)
{
    if( var->index == 0 )
        return var->data[1];
    return var->data[0];
}


// Tools ---------------------------------------------------------------------//
unsigned char inverseIndex(unsigned char index)
{
    if( index == 0 )
        return 1;
    return 0;
}
