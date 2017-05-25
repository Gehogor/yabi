/*
 *__     __        ____  _____
 *\ \   / / /\    |  _ \|_   _|
 * \ \_/ / /  \   | |_) | | |
 *  \   / / /\ \  |  _ <  | |
 *   | | / ____ \ | |_) |_| |_
 *   |_|/_/    \_\|____/|_____|
 *
 * File:   SafeVarLib.h
 * Yann Alonso and Jerome Chemouny
 *
 * Created on 25 avril 2016, 19:01
 */

#ifndef _SAFEVARIABLE_H_
#define	_SAFEVARIABLE_H_

// 32 bits signed management -------------------------------------------------//

union S32_U8 {
    unsigned char c[4];
    long l;
};

typedef struct {
    unsigned char reader;
    unsigned char writer;
    union S32_U8 buffer[2];
} SafeData_s32;

long toS32_s32(SafeData_s32* data);
void fromS32_s32(SafeData_s32* data, long value);
unsigned char toU8_s32(SafeData_s32* data, unsigned char index);
void fromU8_s32(SafeData_s32* data, unsigned char value, unsigned char index);

// 16 bits unsigned management -----------------------------------------------//

union U16_U8 {
    unsigned char c[2];
    unsigned int i;
};

typedef struct {
    unsigned char reader;
    unsigned char writer;
    union U16_U8 buffer[2];
} SafeData_u16;

unsigned int toU16_u16(SafeData_u16* data);
unsigned char toU8_u16(SafeData_u16* data, unsigned char index);
void fromU16_u16(SafeData_u16* data, unsigned int value);

#endif	/* _SAFEVARIABLE_H_ */
