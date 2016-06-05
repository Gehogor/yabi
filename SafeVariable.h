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

#ifndef _SHAREDVARLIB_H_
#define	_SHAREDVARLIB_H_

typedef struct {
    unsigned char index;
    unsigned char data[2];
} safe_u8;

typedef struct {
    unsigned char index;
    unsigned int data[2];
} safe_u16;

typedef struct {
    unsigned char index;
    unsigned long int data[2];
} safe_u32;

typedef struct {
    unsigned char index;
    signed char data[2];
} safe_s8;

typedef struct {
    unsigned char index;
    signed int data[2];
} safe_s16;

typedef struct {
    unsigned char index;
    signed long int data[2];
} safe_s32;

void init_u8(safe_u8* var, unsigned char value);
void write_u8(safe_u8* var, unsigned char value);
unsigned char read_u8(safe_u8* var);

void init_u16(safe_u16* var, unsigned int value);
void write_u16(safe_u16* var, unsigned int value);
unsigned int read_u16(safe_u16* var);

void init_u32(safe_u32* var, unsigned long int value);
void write_u32(safe_u32* var, unsigned long int value);
unsigned long int read_u32(safe_u32* var);

void init_s8(safe_s8* var, signed char value);
void write_s8(safe_s8* var, signed char value);
signed char read_s8(safe_s8* var);

void init_s16(safe_s16* var, signed int value);
void write_s16(safe_s16* var, signed int value);
signed int read_s16(safe_s16* var);

void init_s32(safe_s32* var, signed long int value);
void write_s32(safe_s32* var, signed long int value);
signed long int read_u32(safe_s32* var);

unsigned char inverseIndex(unsigned char index);

#endif	/* _SHAREDVARLIB_H_ */
