/*
 *__     __        ____  _____
 *\ \   / / /\    |  _ \|_   _|
 * \ \_/ / /  \   | |_) | | |
 *  \   / / /\ \  |  _ <  | |
 *   | | / ____ \ | |_) |_| |_
 *   |_|/_/    \_\|____/|_____|
 *
 * File:   SafeVariable.c
 * Author: Yann Alonso and Jerome Chemouny
 *
 * Created on 25 avril 2016, 19:01
 */

//---------------------//
#include "SafeVariable.h"
//---------------------//

long toS32_s32( SafeData_s32* data )
{
    data->reader = ~data->writer;
    return data->buffer[data->reader].l;
}

void fromS32_s32( SafeData_s32* data,long value )
{
    data->writer = ~data->reader;
    data->buffer[data->writer].l = value;
}

unsigned char toU8_s32( SafeData_s32* data,unsigned char index )
{
    if(index == 0)
        data->reader = ~data->writer;

    return data->buffer[data->reader].c[index];
}

void fromU8_s32( SafeData_s32* data,unsigned char value,unsigned char index )
{
    if(index == 0)
        data->writer = ~data->reader;

    data->buffer[data->writer].c[index] = value;
}

unsigned int toU16_u16( SafeData_u16* data )
{
    data->reader = ~data->writer;
    return data->buffer[data->reader].i;
}

unsigned char toU8_u16( SafeData_u16* data,unsigned char index )
{
    if(index == 0)
        data->reader = ~data->writer;

    return data->buffer[data->reader].c[index];
}

void fromU16_u16( SafeData_u16* data,unsigned int value )
{
    data->writer = ~data->reader;
    data->buffer[data->writer].i = value;
}
